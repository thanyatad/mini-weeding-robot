# firmware/esp32

ESP32 low-level controller — **motion + safety ที่ต้องทำงานได้เองโดยไม่พึ่ง Pi**

รายละเอียดเต็ม: [../../docs/hardware.md](../../docs/hardware.md)

---

## Layout

```text
firmware/esp32/
├── platformio.ini
│
├── src/
│   ├── main.cpp                # setup / loop, latest-wins dispatch
│   ├── mixing.cpp              # (v, omega) → v_left, v_right + saturation
│   ├── motors.cpp              # PWM + direction + enable
│   ├── safety.cpp              # command timeout + estop latch
│   └── communication.cpp       # UART2 + JSON line framing
│
├── include/
│   ├── mixing.h
│   ├── motors.h
│   ├── safety.h
│   └── protocol.h              # ต้องตรงกับ protocol/schema.json
│
└── test/
    └── test_mixing.cpp         # native env — อ่าน config/drive_mixing_vectors.csv
```

---

## Scope

```text
ESP32
│
├── Skid-steer mixing        (v, omega) → v_left, v_right → PWM
├── Motor PWM                4 มอเตอร์ ต่อขนานเป็น 2 ช่อง
├── Command timeout          ◄── safety ชั้นที่ต้องเป็นอิสระจาก Pi
├── Motor enable             latch จาก estop / timeout
├── E-stop sense             อ่านไฟจริง ไม่ใช่ตัวแปร software
├── Communication            UART2, latest-wins
└── Watchdog
```

❌ **ไม่** ทำ: weed detection, AI, row following (มันไม่เห็นภาพ),
high-level workflow, การตัดสินใจว่า `row_end` หรือ `row_lost`

**ตัดออกจาก firmware ของ design gantry ทั้งหมด**: stepper STEP/DIR · TMC2209 UART ·
endstop × 4 · servo PWM · soil contact

---

## Command Flow

```text
Pi ──drive seq=N (10 Hz)──► ESP32
                              ├── parse ทุก line ใน buffer
                              ├── ใช้ seq สูงสุด  ทิ้งที่เหลือ
                              ├── mixing → saturation → PWM
                              └── reset timeout timer

Pi ◄──rover_state (20 Hz)──── commanded / last_seq / estop / motors_enabled
```

---

## Latest-wins — ต้องบังคับที่นี่ ไม่ใช่ที่ Pi

serial buffer สะสมได้ ถ้าประมวลผลเรียงตามคิว มันจะเลี้ยวตามคำสั่งเก่า
ที่ Pi ยกเลิกความคิดไปแล้ว

```text
ทุกรอบ loop:  อ่าน line ทั้งหมดที่มีใน buffer
              ใช้ drive ตัวที่ seq สูงสุด  ทิ้งที่เหลือ
              seq <= last_seq  →  ทิ้ง (out of order)
```

`stop` / `emergency_stop` ที่พบใน buffer ต้อง **ทำทันทีไม่ว่าลำดับไหน**
และ override `drive` ทุกตัวในรอบนั้น

`drive` ไม่มี `ack` — ยืนยัน liveness ด้วย `last_seq` ใน telemetry แทน
ถ้าตอบ ack ทุกตัวจะมี traffic 10 Hz ที่ไม่มีใครใช้ และเฟรมที่ตกจะกลายเป็น ERROR

---

## Command Timeout — safety ที่ต้องไม่พึ่ง Pi

```c
if (millis() - last_drive_ms > COMMAND_TIMEOUT_MS) {
    motors_disable();
    fault = COMMAND_TIMEOUT;     // latch — ต้อง reset
}
```

`COMMAND_TIMEOUT_MS` มาจาก `config/safety.yaml` (300 ms) — **ห้าม hard-code**

```text
v_max × command_timeout / 1000 <= runaway_budget    100 × 0.3 = 30 mm <= 60 mm
```

Timeout เป็น **latch**: เมื่อ link กลับมา ต้องรายงาน `command_timeout` แล้วรอ `reset`
**ไม่ใช่ออกตัวต่อเองเมื่อ `drive` ตัวถัดไปมาถึง**

### ทำไมต้องอยู่ฝั่ง firmware

```text
Pi แครช / สาย UART หลุด  →  ESP32 ยังมีไฟ  →  ดับมอเตอร์เอง  ✓
```

ถ้า timeout อยู่ฝั่ง Pi แล้ว Pi แครช จะไม่มีใครสั่งหยุด

⚠️ **ESP32 ต้องมี buck ของตัวเอง ไม่กินไฟจาก USB ของ Pi** — ไม่งั้น safety ชั้นนี้
พึ่ง Pi ทางไฟฟ้าอยู่ดี ดู [../../hardware/electrical/power.md](../../hardware/electrical/power.md#️-esp32-ต้องไม่กินไฟจาก-pi)

---

## Skid-Steer Mixing

```c
omega_rad = omega_deg * PI / 180.0f;
v_left    = v - omega_rad * TRACK_WIDTH_MM / 2.0f;
v_right   = v + omega_rad * TRACK_WIDTH_MM / 2.0f;
```

ล้อหน้าและหลังข้างเดียวกันได้ค่าเดียวกัน (ต่อขนานทางไฟอยู่แล้ว)

### Saturation ต้อง scale ไม่ clip

```c
float peak = fmaxf(fabsf(v_left), fabsf(v_right));
if (peak > WHEEL_V_MAX_MM_S) {
    float k = WHEEL_V_MAX_MM_S / peak;
    v_left  *= k;
    v_right *= k;                 // อัตราส่วนเดิม → omega ไม่เปลี่ยน
}
```

การ clip ข้างเดียวจะเปลี่ยน `omega` ที่ได้จริงโดยเงียบ

### ⚠️ Deadband — ต้องรู้ ไม่ใช่ต้องแก้

มอเตอร์เกียร์ DC ไม่ออกตัวใต้ ~15% duty (≈ 51 mm/s ที่ล้อ Ø65)

```text
ที่ omega = 60 deg/s:  v_left = 37 mm/s  (11%)  ✗ ล้อหยุด แต่ firmware คิดว่าสั่งแล้ว
ที่ omega = 40 deg/s:  v_left = 58 mm/s  (17%)  ✓
```

firmware **ไม่ต้องชดเชย deadband** — มันถูกกันด้วย startup invariant ที่ฝั่ง Pi
(`omega_max` ถูกจำกัดไว้ที่ 40 ซึ่งเพดานจริงคือ 46)

สิ่งที่ firmware ต้องทำคือ **ปฏิเสธคำสั่งที่เป็นไปไม่ได้** แทนที่จะพยายามทำ:

```text
v == 0 && omega != 0   →  ตอบ error  (turn-in-place อยู่นอก MVP
                          ล้อทั้งสองข้างจะอยู่ใน deadband และต้องกลับทิศ)
```

### Golden vectors — สูตรนี้ถูก implement ซ้ำใน Python

```text
config/drive_mixing_vectors.csv     ◄── source of truth เดียว
        ├──> tests/unit/test_drive_mixing.py       Python / IsaacRover
        └──> firmware/esp32/test/test_mixing.cpp   ที่นี่
```

```bash
pio test -e native          # ไม่ต้องมีบอร์ด
```

ถ้าตัวใดตัวหนึ่ง fail แปลว่า implementation สองภาษาเพี้ยนจากกัน —
**แก้ implementation ห้ามแก้ตาราง**

ถ้า `track_width_mm` หรือ `wheel_v_max_mm_s` ใน CAD/config เปลี่ยน
ต้อง **regenerate ตารางทั้งชุด** ไม่ใช่แก้ทีละแถว

---

## E-stop

```text
แบต ──┬── E-stop (latching NC) ── driver ── มอเตอร์
      │         └──► sense ──► GPIO 39 (ต้องมี pull-up ภายนอก)
      └── buck 5V ── Pi + ESP32     ◄── ไม่ผ่าน E-stop
```

E-stop ตัดไฟมอเตอร์ **ทางกายภาพ** — sense line มีไว้ให้ firmware รายงานสถานะ
ไม่ใช่ให้ firmware เป็นคนหยุด

firmware ต้อง:

```text
[ ] อ่าน sense line ทุกรอบ loop ใส่ใน rover_state.estop
[ ] ดึง EN LOW ทันทีที่ sense active (กันมอเตอร์ออกตัวตอนปล่อยปุ่ม)
[ ] ปฏิเสธ reset ขณะ sense ยัง active → ตอบ error code emergency_stop
```

⚠️ `GPIO 39` เป็น input only และ **ไม่มี internal pull-up** —
`pinMode(39, INPUT_PULLUP)` compile ผ่านแต่ไม่ทำอะไรเลย ต้องใส่ 10 kΩ ภายนอก
ดู [../../hardware/electrical/wiring.md](../../hardware/electrical/wiring.md#️-gpio-34-35-36-39-ไม่มี-internal-pull-up)

---

## Boot Safety

```text
[ ] EN ต้องเป็น LOW ตั้งแต่ก่อน setup() เสร็จ
[ ] motors_enabled = false จนกว่าจะได้ drive ตัวแรก
[ ] fault = COMMAND_TIMEOUT เป็นสถานะเริ่มต้น (ไม่ใช่ OK)
```

ถ้า EN ลอยตอนบูต มอเตอร์อาจหมุนก่อน firmware พร้อม — บน rover นี่คือการออกตัวเอง

`uptime_ms` ใน telemetry มีไว้ให้ Pi ตรวจจับการรีบูตกลางทาง (ค่าลดลง)
ไม่ใช่ใช้คำนวณ latency

---

## Build & Test

```bash
./scripts/flash_esp32.sh       # build + upload
pio test -e native            # unit test บน host ไม่ต้องมีบอร์ด
```

`platformio.ini` ต้องมี 2 environment:

```text
[env:esp32dev]     บอร์ดจริง
[env:native]       host — สำหรับ test_mixing.cpp
```

`mixing.cpp` ต้องไม่ include อะไรที่ผูกกับ Arduino/ESP-IDF เพื่อให้ compile
ใน `native` ได้ — ถ้ามันต้องพึ่ง `analogWrite` มันไม่ใช่ pure function
และจะ test ข้ามภาษาไม่ได้

---

## HIL — ต้องล้อลอย

```text
ยกล้อลอย → ถอดสาย UART · กด E-stop · ส่ง seq ย้อนหลัง · ยัด buffer · วัด deadband
         → ผ่านครบแล้วจึงวางลงพื้น
```

รายการเต็ม: [../../hardware/mechanical/assembly.md](../../hardware/mechanical/assembly.md#bench-test-checklist--ล้อลอย-ขั้นที่-11)
