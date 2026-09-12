# Wiring

การเดินสายและ pin mapping ของ ESP32

⚠️ Pin ที่นี่ต้องตรงกับ `firmware/esp32/include/motors.h` — ถ้าไม่ตรง อาการคือ
มอเตอร์ไม่หมุนหรือหมุนผิดข้าง ซึ่งใช้เวลา debug นานเกินความจำเป็น

---

## ภาพรวม

```text
             Raspberry Pi                      ESP32
          ┌─────────────────┐           ┌─────────────────┐
camera ──►│ GPIO14 TX ──────┼──────────►│ GPIO16 RX2      │
front     │ GPIO15 RX ◄─────┼───────────┤ GPIO17 TX2      │
camera ──►│ GND ────────────┼───────────┤ GND             │
down      └─────────────────┘           └────┬───┬────┬───┘
                                             │   │    │
                                    PWM ◄────┘   │    └──► GPIO39 estop sense
                                                 └──► GPIO27 EN
                                ┌────────────────────────┐
                                │  H-bridge × 2 ช่อง      │
                                └───┬────────────────┬───┘
                                    ▼                ▼
                              motor_fl+rl      motor_fr+rr
                               (ซ้าย ขนาน)      (ขวา ขนาน)
```

Pi กับ ESP32 คุยกันด้วย **UART ไม่ใช่ USB** — เหตุผลอยู่ที่ [§UART ไม่ใช่ USB](#uart-ไม่ใช่-usb)

---

## ESP32 Pin Map

### Motor driver (BTS7960 / IBT-2 × 2)

| สัญญาณ | GPIO | หมายเหตุ |
|---|---|---|
| LEFT RPWM | 25 | LEDC channel, ~1 kHz |
| LEFT LPWM | 26 | |
| RIGHT RPWM | 32 | |
| RIGHT LPWM | 33 | |
| EN (ร่วมทั้งสอง driver) | 27 | R_EN + L_EN ผูกเข้าด้วยกัน active HIGH |

RPWM / LPWM คู่กันให้ทั้งทิศทางและความเร็ว: ขาหนึ่ง PWM อีกขา 0

**ห้าม PWM ทั้งสองขาพร้อมกัน** — จะเป็น shoot-through ทำให้ MOSFET ร้อนและพัง

`EN` เป็นขาที่ `safety.cpp` ใช้ดับมอเตอร์เมื่อ command timeout หรือ estop —
ต้องดึง LOW เป็น default ตอนบูต ไม่งั้นมอเตอร์อาจหมุนก่อน firmware พร้อม

### Serial ไป Pi (UART2)

| สัญญาณ | GPIO | ไปที่ |
|---|---|---|
| TX2 | 17 | Pi GPIO15 (RXD) |
| RX2 | 16 | Pi GPIO14 (TXD) |
| GND | GND | Pi GND (**บังคับ**) |

ทั้งสองบอร์ดเป็น **3.3 V logic** — ต่อตรงได้ ไม่ต้องมี level shifter

Baud: **115200** (ใช้ ~30% ของ link) ดู [../../docs/protocol.md](../../docs/protocol.md#transport)

### Safety

| สัญญาณ | GPIO | หมายเหตุ |
|---|---|---|
| E-stop sense | 39 | **input only — ต้องมี external pull-up** |

---

## UART ไม่ใช่ USB

ทางที่ง่ายกว่าคือเสียบ USB จาก Pi ไป ESP32 DevKit — ได้ทั้ง serial และไฟเลี้ยง
แต่**ทำลาย safety layer ที่ MVP พึ่งอยู่**

```text
USB:   ESP32 กินไฟจาก Pi  →  Pi ดับ = ESP32 ดับ = ไม่มีใครดับมอเตอร์
UART:  ESP32 มี buck ของตัวเอง  →  Pi ดับ = ESP32 ยังอยู่ = command timeout ทำงาน
```

command timeout เป็น safety ชั้นที่ต้อง **ไม่พึ่ง Pi** ถ้า ESP32 กินไฟจาก Pi
ชั้นนั้นก็พึ่ง Pi ทางไฟฟ้าอยู่ดี

USB ยอมรับได้สำหรับ flash firmware และ bench debug — **แต่ไม่ใช่สำหรับ link
ที่ใช้งานจริง** ดู [../../docs/architecture.md](../../docs/architecture.md#7-safety--3-ชั้น-3-เจ้าของ)

---

## ⚠️ GPIO 34, 35, 36, 39 ไม่มี internal pull-up

ขาเหล่านี้เป็น **input only** และ **ไม่มี pull-up/pull-down ภายใน**

ถ้าต่อ switch โดยไม่ใส่ตัวต้านทานภายนอก ขาจะลอย และอ่านค่าได้แบบสุ่ม —
อาการคือ E-stop trigger เองตอนมอเตอร์หมุน

**ต้องใส่ pull-up 10 kΩ ไป 3.3 V**

```text
3.3 V
  │
 10 kΩ
  │
  ├──────── GPIO 39
  │
E-stop sense (หลังหน้าสัมผัส NC)
  │
 GND
```

`pinMode(39, INPUT_PULLUP)` **ไม่มีผล** — compile ผ่าน ไม่มี error แต่ไม่ได้ทำอะไรเลย
เป็นกับดักที่พบบ่อยที่สุดของ ESP32

### GPIO ที่ห้ามใช้

```text
6–11        ต่อกับ flash — ใช้แล้วบูตไม่ขึ้น
0, 2, 15    strapping pin — มีแรงดันตอนบูตแล้วเข้า mode ผิด
12          strapping pin (MTDI) — HIGH ตอนบูต → flash voltage 1.8 V → บูตไม่ขึ้น
1, 3        UART0 — ใช้สำหรับ flash และ boot log
34–39       input only — ไม่ output ได้ และไม่มี pull-up
```

pin map ข้างบนหลีกเลี่ยงทั้งหมดแล้ว — 25, 26, 27, 32, 33 เป็น output ปกติ
และ 16, 17 เป็น UART2 ที่ไม่ชนกับ boot

---

## Switch Wiring — NC เสมอ

E-stop ต่อแบบ **normally closed**

```text
ปกติ:      วงจรปิด  → sense อ่านได้ LOW   → ปลอดภัย
ถูกกด:     วงจรเปิด  → sense อ่านได้ HIGH  → หยุด
สายขาด:    วงจรเปิด  → sense อ่านได้ HIGH  → หยุด    ◄── สำคัญ
```

สายขาดต้องให้ผลเหมือนถูกกด ไม่ใช่เหมือนปกติ

ถ้าต่อแบบ NO สายที่หลุดจะทำให้ระบบคิดว่า "ยังไม่มีใครกด" แล้ววิ่งต่อไป —
เป็นความผิดพลาดที่ไม่แสดงอาการจนกว่าจะเกิดความเสียหาย

MVP **ไม่มี endstop และไม่มี bumper** จึงมี switch แค่ตัวนี้ตัวเดียว —
ยิ่งต้องต่อให้ถูก

---

## E-stop — ตัด motor rail ไม่ใช่ตัด logic

```text
        แบต 3S 11.1 V
              │
         ┌────┴────┐
         │ สวิตช์หลัก│
         │ + ฟิวส์ 10A│
         └────┬────┘
              │
      ┌───────┴────────┬──────────────────┐
      │                │                  │
 ┌────┴─────┐    ┌─────┴─────┐      ┌─────┴─────┐
 │  E-stop  │    │ buck 5V   │      │ buck 5V   │
 │latching NC│   │  (Pi)     │      │ (logic)   │
 └────┬─────┘    └─────┬─────┘      └─────┬─────┘
      │                │                  │
      ├──► sense ──────┼──────────────► ESP32 GPIO39
      │                ▼                  ▼
 ┌────┴─────┐         Pi              ESP32
 │ H-bridge │
 │  × 2ch   │   + ตัวเก็บประจุ 1000 µF ที่ขาไฟเข้า
 └────┬─────┘
      ▼
  มอเตอร์ × 4
```

E-stop ต้อง **ตัดไฟมอเตอร์ทางกายภาพ** ส่วนสาย sense ที่ไป GPIO เป็นแค่
ให้ firmware รู้สถานะเพื่อรายงานขึ้น controller

E-stop ที่ทำงานผ่าน software อย่างเดียวไม่ใช่ E-stop — ถ้า firmware ค้าง
หรือ MCU แฮงค์ มอเตอร์จะยังหมุนต่อ

**Pi และ ESP32 ต้องไม่ถูกตัดไฟไปด้วย** ไม่งั้นจะรายงานสถานะไม่ได้
และ controller จะเห็นเป็น `communication_lost` แทน `emergency_stop` —
สองอย่างนี้ต้องการการจัดการต่างกัน

firmware ต้อง **ปฏิเสธ `reset`** ขณะที่ sense line ยัง active
ตอบ `error` code `emergency_stop` แทน `ack`

---

## Motor — ต่อขนานเป็นข้าง

skid-steer สั่งล้อหน้าและหลังข้างเดียวกันเหมือนกันเสมอ

```text
ช่อง A (LEFT)  ──┬── motor_fl
                 └── motor_rl

ช่อง B (RIGHT) ──┬── motor_fr
                 └── motor_rr
```

เหลือ driver 2 ช่องพอสำหรับ 4 มอเตอร์ แต่แต่ละช่องต้องทน **stall ของสองมอเตอร์**
(3–4 A) → ต้องเลือก ≥ 5 A/ช่อง

แลกกับ: คุมหน้า/หลังแยกไม่ได้ (ไม่ต้องใช้) และวัดกระแสต่อล้อไม่ได้ (ไม่ต้องใช้)

### ตรวจทิศทางก่อนปิดฝา

```text
drive(+100, 0)         → ล้อทั้ง 4 หมุนไปข้างหน้า
drive(+100, +40)       → ล้อขวาเร็วกว่าซ้าย  (เลี้ยวซ้าย = CCW)
```

ถ้าข้างใดกลับ ให้สลับสายคู่ของมอเตอร์นั้น หรือกลับใน firmware —
**เลือกแก้ใน firmware แล้วบันทึกไว้** จะได้ไม่สับสนตอนเปลี่ยนมอเตอร์

⚠️ ถ้า `omega > 0` ทำให้เลี้ยวขวาแทนซ้าย row follower จะกลายเป็น
**positive feedback** — rover จะออกนอกร่องเร็วขึ้นแทนที่จะกลับเข้าร่อง
นี่เป็นกับดักที่แพงที่สุดของ MVP

---

## Grounding

```text
แบต 3S ──┬── H-bridge VM (ผ่าน E-stop)
         ├── buck 5V (Pi)      ──── Pi 5V
         └── buck 5V (logic)   ──── ESP32 VIN

GND ─────── ต่อร่วมกันทั้งหมดที่จุดเดียว (star ground ที่ขั้วลบแบต)
```

GND ของ Pi, ESP32 และ driver **ต้องร่วมกัน** ไม่งั้นสัญญาณ UART และ PWM
จะไม่มีจุดอ้างอิงที่ถูกต้อง

แต่ห้องต่อพ่วงเป็นลูกโซ่ (daisy chain) — กระแสมอเตอร์ที่ไหลผ่าน GND ร่วม
จะสร้างแรงดันตกคร่อมที่ทำให้ ESP32 เห็น GND ไม่เท่ากับ Pi

---

## Cable Management

```text
[ ] สายกล้องขึ้นเสาต้องใช้ spiral wrap — เสาสั่นทำให้ภาพสั่น
[ ] สายมอเตอร์แยกจากสาย UART และสายกล้อง
[ ] สายทุกเส้นต้องไม่พันล้อตลอดการหมุน (ทดสอบด้วยมือ 360°)
[ ] เผื่อความยาวให้รถเคลื่อนที่ได้ ถ้ายังมีสายภายนอก (bench test)
```

สายมอเตอร์ที่เดินคู่กับสาย UART ยาว ๆ จะเหนี่ยวนำสัญญาณรบกวนเข้า serial
ทำให้ JSON parse พลาดเป็นบางเฟรม — อาการจะโผล่เฉพาะตอนมอเตอร์หมุนเร็ว
และจะดูเหมือนปัญหา latest-wins ทั้งที่เป็นปัญหาสายไฟ

**ภาพสั่นจากเสากล้องที่โยกทำให้ valley histogram แกว่ง → `omega` กระตุก**
ซึ่งจะดูเหมือน gain สูงเกิน — ยึดเสาให้แน่นก่อน tune gain

---

## Verification

ก่อนจ่ายไฟ ดู [../mechanical/assembly.md](../mechanical/assembly.md#pre-power-checklist)

หลังจ่ายไฟ (ล้อลอย):

```text
[ ] อ่าน E-stop sense — กดแล้วค่าเปลี่ยน และไฟมอเตอร์ดับจริง
[ ] Pi และ ESP32 ยังมีไฟขณะ E-stop ถูกกด
[ ] UART สองทาง: Pi ส่ง drive → ESP32 ตอบ rover_state
[ ] หมุนมอเตอร์ทีละข้างช้า ๆ ตรวจทิศทาง
[ ] ถอดสาย UART → มอเตอร์ดับใน 300 ms
[ ] วัด wheel_v_min ทั้ง 4 ล้อ
```

รายการเต็ม: [../mechanical/assembly.md](../mechanical/assembly.md#bench-test-checklist--ล้อลอย-ขั้นที่-11)
