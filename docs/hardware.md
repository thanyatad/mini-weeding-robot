# Hardware & Firmware

---

## Scope

เอกสารนี้ครอบคลุม **firmware และการ integrate** เท่านั้น

ของจริงทางกายภาพ — รายการของ, ขนาด, การประกอบ, สายไฟ — อยู่ที่ [`hardware/`](../hardware/README.md)

| ต้องการ | ไปที่ |
|---|---|
| รายการของ / ข้อกำหนดก่อนซื้อ | [hardware/bom/poc-v2.md](../hardware/bom/poc-v2.md) |
| ขนาด rover | [hardware/mechanical/dimensions.md](../hardware/mechanical/dimensions.md) |
| ลำดับประกอบ + ค่าที่ต้องวัด | [hardware/mechanical/assembly.md](../hardware/mechanical/assembly.md) |
| Pin map + กับดัก ESP32 | [hardware/electrical/wiring.md](../hardware/electrical/wiring.md) |
| ไฟ / power budget | [hardware/electrical/power.md](../hardware/electrical/power.md) |
| นิยาม coordinate frame | [hardware/mechanical/coordinate-frames.md](../hardware/mechanical/coordinate-frames.md) |
| ขนาดต้นทาง (CAD) | [cad/parameters/README.md](../cad/parameters/README.md) |

เส้นแบ่ง: *ถ้าต้องใช้ไขควง = `hardware/` · ถ้าต้องใช้ compiler = ที่นี่*

---

## สองคอมพิวเตอร์ ไม่ใช่หนึ่ง

```text
Raspberry Pi (บน rover)              ESP32
├── Perception (2 กล้อง)              ├── Skid-steer mixing
├── RowFollower                      ├── Motor PWM
├── State machine                    ├── Command timeout  ◄── safety ชั้นที่ต้องเป็นอิสระ
├── Safety: row-loss watchdog        ├── Motor enable
└── Startup invariants               ├── E-stop sense
                                     └── Watchdog
```

**ESP32 ต้องปลอดภัยได้เองโดยไม่พึ่ง Pi** — ถ้า Pi แครชหรือสาย USB หลุด
มอเตอร์ต้องดับเองภายใน `command_timeout_ms` นี่คือเหตุผลที่ command timeout
อยู่ฝั่ง firmware ไม่ใช่ฝั่ง Pi

---

## Firmware Scope

```text
ESP32
│
├── Skid-steer mixing        (v, omega) → v_left, v_right → PWM
├── Motor PWM                4 มอเตอร์ ต่อขนานเป็น 2 ช่อง
├── Command timeout          ไม่ได้ drive เกิน 300 ms → ดับมอเตอร์
├── Motor enable             latch จาก estop / timeout
├── E-stop sense             อ่านไฟจริงหลัง E-stop ไม่ใช่ตัวแปร software
├── Communication            JSON บรรทัดละ message, latest-wins
└── Watchdog
```

**ไม่ควร** ให้ ESP32 ทำ:

- Weed detection · AI
- Row following (มันไม่เห็นภาพ)
- High-level workflow
- การตัดสินใจว่า `row_end` หรือ `row_lost`

**ตัดออกจาก firmware เดิมทั้งหมด**: stepper STEP/DIR · TMC2209 UART · endstop ×4 ·
servo PWM · soil contact

---

## Firmware Layout

```text
firmware/esp32/
│
├── platformio.ini
│
├── src/
│   ├── main.cpp
│   ├── mixing.cpp              # (v, omega) → v_left, v_right + saturation
│   ├── motors.cpp              # PWM + direction + enable
│   ├── safety.cpp              # command timeout + estop latch
│   └── communication.cpp       # JSON line parse, latest-wins
│
├── include/
│   ├── mixing.h
│   ├── motors.h
│   ├── safety.h
│   └── protocol.h
│
└── test/
    └── test_mixing.cpp         # native env — อ่าน config/drive_mixing_vectors.csv
```

---

## Skid-Steer Mixing — และกับดักที่ต้องรู้

```text
omega_rad = omega_deg * PI / 180.0
v_left    = v - omega_rad * track_width_mm / 2
v_right   = v + omega_rad * track_width_mm / 2
duty      = v_side / wheel_v_max_mm_s
```

### ต้อง scale ไม่ clip

ถ้าล้อข้างใดเกิน `wheel_v_max_mm_s` → **ลดทั้งสองข้างตามอัตราส่วนเดิม**
การ clip ข้างเดียวเปลี่ยน `omega` ที่ได้จริงโดยเงียบ

### ⚠️ Deadband — เหตุผลที่ `omega_max` เป็น 40 ไม่ใช่ 60

มอเตอร์เกียร์ DC **ไม่ออกตัวใต้ ~15% duty**

```text
ล้อ 65 mm · มอเตอร์ 100 RPM → wheel_v_max = 340 mm/s
deadband ≈ 15% ของ 340       → wheel_v_min ≈ 51 mm/s

ที่ omega = 60 deg/s:  v_left = 37 mm/s  (11%)  ✗ ล้อหยุดหมุนแต่ firmware คิดว่าสั่งแล้ว
ที่ omega = 40 deg/s:  v_left = 58 mm/s  (17%)  ✓ เหนือ deadband 7 mm/s

เพดานจริงของ omega_max คือ 46 deg/s — เลือก 40 เพื่อให้มี margin
ไม่ใช่เพราะ 40 เป็นค่าสูงสุดที่ทำได้
```

ล้อข้างในที่หยุดหมุนทำให้ rover เลี้ยวแรงกว่าที่สั่ง **โดยไม่มีสัญญาณบอก** —
ไม่มี encoder ไม่มีใครรู้ ต้องกันที่ startup invariant

`wheel_v_min_mm_s` **ต้องวัดจริงที่ V3** ก่อนวางล้อลงพื้น — ค่า 51 เป็นค่าประมาณ

### Turn-in-place อยู่นอก MVP

`v = 0` กับ `omega` ใด ๆ ทำให้ล้อทั้งสองข้างอยู่ใน deadband และต้องกลับทิศ
MVP ไม่ต้องใช้ (วิ่งร่องเดียวแล้วหยุด) — firmware ควร**ปฏิเสธ** `drive` ที่
`v == 0 && omega != 0` แล้วตอบ `error` แทนที่จะพยายามทำ

### สูตรนี้ถูก implement ซ้ำใน Python

`controller/rover/mixing.py` มีสูตรเดียวกัน แชร์โค้ดกันไม่ได้เพราะต่างภาษา
ทั้งสองต้องผ่าน golden vector ตารางเดียวกัน:

```bash
pytest tests/unit/test_drive_mixing.py         # Python
pio test -e native                             # C++
```

ทั้งคู่อ่าน [`config/drive_mixing_vectors.csv`](../config/drive_mixing_vectors.csv)

---

## Command Flow

```text
Controller (Pi)                      ESP32
│                                    │
├── drive seq=N (10 Hz) ────────────►│
│                                    ├── parse ทุก line ใน buffer
│                                    ├── ใช้ seq สูงสุด ทิ้งที่เหลือ
│                                    ├── mixing → saturation → PWM
│                                    └── reset timeout timer
│                                    │
│◄──── rover_state (20 Hz) ──────────┤ commanded / last_seq / estop / motors_enabled
```

### Latest-wins ต้องบังคับที่ฝั่ง ESP32

serial buffer สะสมได้ ถ้าประมวลผลเรียงตามคิว มันจะเลี้ยวตามคำสั่งเก่า
ที่ Pi ยกเลิกความคิดไปแล้ว

```text
ทุกรอบ loop:  อ่าน line ทั้งหมดที่มีใน buffer
              ใช้ drive ตัวที่ seq สูงสุด  ทิ้งที่เหลือ
              seq <= last_seq  →  ทิ้ง (out of order)
```

`stop` / `emergency_stop` ที่พบใน buffer ต้อง **ทำทันทีไม่ว่าลำดับไหน**
และ override `drive` ทุกตัวในรอบนั้น

---

## Command Timeout — safety ชั้นที่ต้องเป็นอิสระ

```c
if (millis() - last_drive_ms > COMMAND_TIMEOUT_MS) {
    motors_disable();
    fault = COMMAND_TIMEOUT;     // latch — ต้อง reset
}
```

ค่ามาจาก `config/safety.yaml` `command_timeout_ms: 300` — firmware ต้องไม่ hard-code

```text
v_max × command_timeout / 1000 <= runaway_budget      100 × 0.3 = 30 mm <= 60 mm
```

Timeout เป็น **latch** — เมื่อ link กลับมา firmware ต้องรายงาน `command_timeout`
แล้วรอ `reset` ไม่ใช่ออกตัวต่อเองเมื่อ `drive` ตัวถัดไปมาถึง

---

## E-stop

```text
แบต 3S ──┬── E-stop (latching NC) ── driver ── มอเตอร์
         │         └──► sense line ──► ESP32 GPIO
         └── buck 5V ── Pi + ESP32        ◄── ไม่ผ่าน E-stop
```

E-stop **ตัด motor rail ทางไฟ** ไม่ใช่บอก firmware ให้หยุด

ไฟ logic ไม่ถูกตัด → Pi และ ESP32 ยังมีไฟ → **ยังรายงานได้ว่า `estop: true`**
ถ้าตัดทั้งระบบ จะได้ rover ที่หยุดจริงแต่เงียบ แยกไม่ออกจากแบตหมดหรือแครช

firmware ต้อง**ปฏิเสธ** `reset` ขณะที่ sense line ยัง active —
ตอบ `error` code `emergency_stop` แทน `ack`

รายละเอียดการเดินสาย: [../hardware/electrical/wiring.md](../hardware/electrical/wiring.md)

---

## Machine Limits

Firmware ต้อง enforce limit ตาม `config/rover.yaml` ไม่ hard-code

```yaml
rover:
  track_width_mm: 430
  wheel_diameter_mm: 250

  drive:
    v_max_mm_s: 160
    omega_max_deg_s: 25
    wheel_v_max_mm_s: 327
    wheel_v_min_mm_s: 51

safety:
  command_timeout_ms: 300
```

ค่าเต็มและที่มาของแต่ละตัว: [config/README.md](../config/README.md) ·
[hardware/mechanical/dimensions.md](../hardware/mechanical/dimensions.md)

⚠️ `wheel_v_max_mm_s` คือความเร็วล้อสูงสุดของ **มอเตอร์** ส่วน `v_max_mm_s`
คือความเร็วเดินหน้าที่ใช้งาน — ต่างกันเพื่อให้มี headroom ให้ differential ตอนเลี้ยว
ถ้าตั้งเท่ากัน จะเลี้ยวไม่ได้เลยที่ความเร็วสูงสุด

---

## Build & Flash

```bash
./scripts/flash_esp32.sh
```

Build system: **PlatformIO** (ESP-IDF / Arduino framework)

Unit test บน host ไม่ต้องมีบอร์ด:

```bash
pio test -e native
```

---

## Hardware-in-the-Loop — ต้องล้อลอย

ESP32 จริง + motor driver จริง **แต่ยกล้อลอยบนโต๊ะ**

```text
Isaac Sim ──virtual camera──> Controller (Pi) ──serial──> Real ESP32 ──wheel cmd──> Isaac Sim
```

Safety ทั้งสามชั้นต้องถูกทดสอบ **ตอนที่การทดสอบล้มเหลวแล้วไม่มีอะไรวิ่งหนี**
ข้อนี้ไม่มีใน design gantry เพราะ gantry เคลื่อนที่เองไม่ได้

```text
[ ] ถอดสาย serial → มอเตอร์ดับภายใน 300 ms (วัดด้วย scope หรือ log timestamp)
[ ] กด E-stop → ล้อหยุด ขณะที่ Pi ยังส่ง drive อยู่
[ ] ยัด drive 50 ตัวเข้า buffer → ใช้ตัว seq สูงสุดตัวเดียว
[ ] ส่ง seq ย้อนหลัง → ถูกทิ้ง
[ ] reset ขณะ estop ยังกด → ถูกปฏิเสธ
[ ] วัด wheel_v_min_mm_s จริง (ค่อย ๆ เพิ่ม duty จน 0 จนล้อเริ่มหมุน)
[ ] re-validate invariant drive ตัวล่างด้วยค่าที่วัดได้
```

ผ่านครบแล้วจึงวางลงพื้น
