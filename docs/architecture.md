# Architecture

## 1. Objective

Repository นี้ออกแบบให้พัฒนา Mini Smart Weeding Robot โดยเริ่มจาก Simulation
แล้วเปลี่ยนไปใช้ Hardware จริงภายหลังได้ โดยไม่ต้องรื้อ architecture หลัก

Form factor: **rover skid-steer 4 ล้อ วิ่งในร่องระหว่างแถวปลูก**

(Rover Base V0 — 650 × 520 mm, ~35 kg: ร่องกว้างขึ้นตามตัวรถ `row_spacing_mm`
จึงเป็น **750 mm** ไม่ใช่ 350 mm ของรอบพัฒนาก่อนหน้า — ดู
[docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md](superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md)
convention ของ frame และ layer ด้านล่างไม่เปลี่ยนเลยตาม form factor นี้)

First MVP ทำสองอย่าง: **เดินตามร่องด้วยกล้อง** และ **ตรวจจับวัชพืชในร่อง** —
ไม่กำจัดวัชพืช ดู [§12 MVP Scope](#12-mvp-scope)

- ใช้ NVIDIA Isaac Sim จำลองแปลงดิน, Rover, Camera, Sensor และ Physics
- ใช้ Python เป็น Robot Controller (รันบน Raspberry Pi บน rover)
- ใช้ Computer Vision ทั้งสำหรับ navigation และ weed detection
- รองรับ ESP32 + H-bridge + DC gear motor ใน Hardware จริง
- ใช้ logic ชุดเดียวกันระหว่าง Simulation และ Hardware
- รองรับ Software-in-the-Loop (SIL) และ Hardware-in-the-Loop (HIL)
- รองรับ Automated Simulation Test
- แยก module ชัดเจนเพื่อให้ระบบขยายได้ในอนาคต

---

## 2. Layer Boundary

```text
                 Interfaces

    Perception ────────────────┐
                               │
                               ▼
                          Controller
                               │
                               ▼
                        Rover Interface
                         │           │
                         ▼           ▼
                      Isaac        ESP32
```

หลักการที่ห้ามละเมิด:

```text
Simulation != Controller
Controller != Hardware
Hardware   != Perception
Perception != Simulation
```

เส้นแบ่งชุดนี้**ไม่ถูกแตะเลย**ตอนเลิก gantry มาใช้ rover — เปลี่ยนแค่เนื้อหาของ
interface ไม่ใช่รูปแบบของมัน นี่คือส่วนที่ architecture เดิมคุ้มค่า

---

## 3. Rover Abstraction

Controller ไม่เรียก Isaac Sim หรือ ESP32 โดยตรง — เรียกผ่าน interface กลาง

```python
from typing import Protocol


class Rover(Protocol):

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None:
        """สั่งความเร็ว ต้องเรียกซ้ำภายใน command_timeout_ms ไม่งั้นมอเตอร์ดับเอง"""
        ...

    def stop(self) -> None:
        """หยุดแบบควบคุม brake ค้าง กลับมาสั่ง drive() ต่อได้"""
        ...

    def get_drive_state(self) -> dict:
        ...

    def emergency_stop(self) -> None:
        """ตัด motor enable + latch ต้อง reset ด้วยมือ"""
        ...
```

Sign convention: `v > 0` เดินหน้า · `omega > 0` เลี้ยวซ้าย (CCW, right-hand rule, z ขึ้น)
ผูกกับ [coordinate-frames.md](../hardware/mechanical/coordinate-frames.md)

Implementation:

```text
Rover
│
├── FakeRover
├── IsaacRover
└── Esp32Rover
```

### `get_drive_state()` ออกแบบให้โกหกไม่ได้

```python
{
    "commanded": {"v_mm_s": 100.0, "omega_deg_s": 0.0},
    "estop": False,
    "link_age_ms": 12,
}
```

**ไม่มี field `measured` โดยเจตนา** — ไม่มี encoder จึงไม่มีใครรู้ว่าล้อหมุนจริงเท่าไร
ถ้าใส่ `v_mm_s` เดี่ยว ๆ คนอ่านจะเข้าใจว่าเป็นค่าที่วัดได้ การซ้อนใต้ `commanded`
ทำให้ call site อ่านออกว่าเป็น echo ของคำสั่ง ไม่ใช่ feedback

`link_age_ms` คือค่าเดียวใน struct นี้ที่วัดได้จริง — safety ใช้จับ communication lost

วันที่เพิ่ม encoder (M2): เติม `"measured": {...}` แบบ additive โดยไม่แตะ key เดิม

`get_drive_state()` ของทุก backend ต้องคืน **key ชุดเดียวกันเป๊ะ** และต้องมี test บังคับ

### สิ่งที่ไม่มีใน interface — และเหตุผล

| ไม่มี | เพราะ |
|---|---|
| `drive_distance(mm)` | ไม่มี encoder จะ implement ได้แค่ `velocity × time` ซึ่งอ้างความแม่นที่ไม่มีจริง ถ้าต้องการ ให้เพิ่ม encoder ไม่ใช่เพิ่ม method |
| `get_position()` | ไม่มี localization ไม่มีอะไรตอบได้ |
| `home()` | ไม่มี absolute frame ให้ home เข้าหา |
| `get_endstops()` | ไม่มี endstop `estop` ย้ายไปอยู่ใน `get_drive_state()` |
| `get_tool_state()` | ไม่มี tool ใน MVP (กลับมาที่ M3) |

interface ที่บอกได้ว่าตัวเองทำอะไร**ไม่ได้** มีค่ามากกว่า interface ที่มี method ครบสวย

### `stop()` ต่างจาก `drive(0, 0)`

ทั้งคู่รีเฟรช watchdog เหมือนกัน แต่ `stop()` สั่ง brake ค้าง ขณะที่ `drive(0, 0)`
ปล่อยให้ไหลตามแรงเฉื่อย — บนแปลงเอียงต่างกันจริง และ call site ที่เขียนว่า
`rover.stop()` อ่านรู้เจตนาทันที

### Correct usage

```python
est = row_estimator.estimate(cameras.front())
rover.drive(*follower.step(est))
rover.stop()
```

### Anti-pattern — ห้ามทำ

```python
if simulation:
    isaac.set_joint_velocity(...)
else:
    serial.write(...)
```

เพราะ simulation logic และ hardware logic จะกระจายไปทั่ว project

### Selection ทำที่ startup เท่านั้น

```python
if config.backend == "isaac":
    rover = IsaacRover()

elif config.backend == "esp32":
    rover = Esp32Rover()

else:
    rover = FakeRover()
```

---

## 4. Camera Abstraction

MVP มีกล้อง 2 ตัวที่ทำหน้าที่ต่างกัน

```python
class CameraSet(Protocol):

    def front(self) -> Frame:
        """สำหรับ row following"""
        ...

    def down(self) -> Frame:
        """สำหรับ weed detection"""
        ...
```

```text
CameraSet
│
├── FakeCameraSet
├── IsaacCameraSet
└── UsbCameraSet
```

เลือก **named accessor** ไม่ใช่ `capture(camera_id)` เพราะสองกล้องนี้ต่างกันที่
contract ไม่ใช่ต่างกันที่ index:

| | `front()` | `down()` |
|---|---|---|
| ใช้ทำ | row following | weed detection |
| Calibration | **ไม่ต้อง** — ใช้แค่มุม/offset ในภาพ | homography → mm |
| Rate | สูง (อยู่ใน control loop) | ต่ำ (นอก control loop) |

`capture(id)` จะซ่อนความต่างนี้ แล้วมีคนเผลอเอาภาพ `front` ไปทำ coordinate transform

---

## 5. Module Responsibility

| Module | รับผิดชอบ | ไม่รับผิดชอบ |
|---|---|---|
| `sim/isaac/` | Physics, แปลงดิน/soil heightfield, scene, sensor, rover model, collision, joint, environment, simulation I/O | Weed detection workflow, business logic, state machine, row following policy, safety policy หลัก |
| `controller/` | Workflow, state machine, row following, safety logic, startup invariants, fault management | Isaac Sim API, ESP32 transport, CV model |
| `perception/` | Camera input, ExG, row estimate, weed detection, weed log, calibration | Motion, rover control |
| `firmware/esp32/` | Skid-steer mixing, motor PWM, command timeout, motor enable, E-stop sense, communication, watchdog | Weed detection, AI, row following, high-level workflow |
| `bridge/` | Serial transport + protocol encoding ระหว่าง Controller และ ESP32 / Simulation | Business logic |

---

## 6. State Machine

```python
class RoverState(Enum):
    BOOT = 1
    READY = 2
    DRIVING_ROW = 3
    STOPPED = 4      # จบ run ปกติ รอคน ack
    ERROR = 5
    ESTOP = 6
```

```text
BOOT ──validate ผ่าน──> READY ──start──> DRIVING_ROW ──row_end──> STOPPED ──ack──> READY
  │                                            │
  │ validate ไม่ผ่าน                             │ row_lost / link_lost / camera_timeout
  ▼                                            ▼
ERROR <───────────────────────────────────── ERROR ──ack──> READY

ESTOP  ◄── estop == true จากทุก state ── ปลดปุ่ม + ack ──> READY
```

MVP วิ่ง **ร่องเดียวแล้วหยุด** — การเลี้ยวเข้าร่องถัดไปต้องเลี้ยวโดยมองไม่เห็น
ร่องเป้าหมาย ซึ่งไม่มี odometry มารองรับ จึงอยู่นอก MVP (→ M4)

### `row_end` กับ `row_lost` แยกกันด้วย `green_fraction`

ทั้งสองกรณีคือ "estimate invalid ติดกัน `row_loss_frames` เฟรม" ตัวแยกมีตัวเดียว:

| `green_fraction` ตอน invalid | ตีความ | ปลายทาง |
|---|---|---|
| ต่ำ — สีเขียวหมดจากเฟรม | สุดร่องแล้ว | `STOPPED(row_end_suspected)` |
| สูง — ยังเห็นเขียวแต่จับเส้นไม่ได้ | navigation พลาด | `ERROR(row_lost)` |

ชื่อ `row_end_suspected` ตั้งให้เตือนว่า **นี่คือ heuristic ไม่ใช่การวัด** —
ต้องมี scenario test คู่ตรงข้าม (`row_end.yaml` + `crop_gap_midrow.yaml`)
ยืนยันว่า threshold แยกได้จริง ผ่านข้างเดียวแปลว่าตั้งเอาใจข้างเดียว

### Safety state — แยกจาก normal workflow

```text
NORMAL
   │
   ├── Row Lost
   ├── Link Lost
   ├── Command Timeout
   ├── Camera Timeout
   ├── Config Invalid
   └── Emergency Stop
          │
          ▼
        ERROR
          │
          ▼
        ESTOP
```

---

## 7. Safety — 3 ชั้น 3 เจ้าของ

| กลไก | เจ้าของ | เงื่อนไข trip | การทำงาน |
|---|---|---|---|
| Command timeout | **ESP32 firmware** | ไม่ได้รับ velocity command เกิน `command_timeout_ms` | มอเตอร์ดับ |
| Row-loss watchdog | **controller** | estimate invalid ติดกัน `row_loss_frames` | `stop()` → STOPPED/ERROR |
| Latching E-stop | **hardware** | คนกด | ตัด motor rail ทางไฟ |

หลักการ: **แต่ละชั้นทำงานได้โดยไม่ต้องพึ่งชั้นบน**

```text
Pi แครช / USB หลุด    →  ESP32 timeout ดับมอเตอร์      (ไม่พึ่ง Pi)
firmware แฮงค์         →  E-stop ตัดไฟได้               (ไม่พึ่ง firmware)
```

ถ้าชั้นใดพึ่งชั้นบน มันไม่ใช่ safety layer มันคือ feature

`link_lost_ms (500) > command_timeout_ms (300)` โดยเจตนา — ESP32 ต้องดับมอเตอร์
**ก่อน**ที่ Pi จะประกาศ error เพื่อให้ลำดับเหตุการณ์ใน log อ่านได้ว่าอะไรเกิดก่อน
ถ้ากลับกัน Pi จะเข้า ERROR ขณะที่ล้อยังหมุนอยู่ 200 ms

MVP **ไม่มี bumper switch** — ตัวชดเชยคือ §8 Runaway Budget

---

## 8. Runaway Budget — invariant แทน tool reach

ไม่มีการป้องกันทางกายภาพ ตัวชดเชยคือการจำกัด **ระยะที่ rover วิ่งต่อได้หลัง
เสียการควบคุม** ให้เป็นค่าที่ประกาศไว้และ validate ตอน startup

```yaml
safety:
  runaway_budget_mm: 60
  row_loss_frames: 3
  command_timeout_ms: 300
```

```text
v_max_mm_s × command_timeout_ms / 1000     <= runaway_budget_mm     30 <= 60
v_max_mm_s × row_loss_frames / loop_hz     <= runaway_budget_mm     30 <= 60
```

`runaway_budget_mm` ต้องเล็กกว่าระยะจากกันชนหน้า rover ถึงขอบแปลงและถึงต้นพืชที่
ใกล้สุด — **ต้องวัดจากแปลงจริงแล้วบันทึก** ไม่ใช่เดา

รูปแบบเดียวกับ tool reach invariant ของ design gantry: ความปลอดภัยที่พึ่งตัวเลข
ใน config ต้องถูก validate ตอน startup และ fail ทันทีพร้อมบอกค่าที่ขัดกัน

รายการ invariant ทั้ง 9 ข้อ: [config/README.md](../config/README.md#startup-invariants)

---

## 9. Development Phases

### V0 — Fake Rover

```text
Controller → FakeRover
```

ยังไม่ใช้ Isaac Sim เป้าหมาย: `Rover` interface, `RowFollower`, state machine,
safety, protocol, startup invariants, golden mixing vectors

**และปิดลูปได้ทั้งวงโดยไม่ต้องมีกราฟิก:**

```text
FakeRover        integrate (v, omega) → pose ภายใน    (kinematic only)
FakeRowSensor    pose + สมการเส้นร่อง → RowEstimate
RowFollower      RowEstimate → (v, omega)
                        └──> วนกลับเข้า FakeRover
```

ตอบคำถาม *"gain ชุดนี้ลู่เข้าไหม และแกว่งเกินร่องไหม"* ในหลักมิลลิวินาที อยู่ใน CI ได้

⚠️ `FakeRover` มี pose ภายใน แต่ **ห้าม expose ผ่าน `get_drive_state()`** —
ถ้าหลุดออกมา code ที่เขียนตอน V0 จะพึ่งพา pose แล้วพังทั้งหมดตอนเปลี่ยนเป็น `esp32`

### V1 — Isaac Simulation

```text
Controller → IsaacRover → Isaac Sim
```

Scene มี: Bed (soil heightfield), Crop rows, Weeds, Rover 4 ล้อ, camera_front, camera_down

เป้าหมาย: **physics tuning ของล้อบน heightfield (งานหนักสุดของโปรเจกต์)**,
row estimator บนภาพ render, scenario ทั้งชุด

### V2 — Computer Vision

```text
camera_down → weed_detector → Detection[] → weed_log
```

เป้าหมาย: per-frame precision / recall เทียบ ground truth จาก Isaac
โดยเปิด randomization ครบ

### V3 — Hardware-in-the-Loop (ล้อลอย)

ESP32 จริง + motor driver จริง **แต่ยกล้อลอยบนโต๊ะ**

```text
ยกล้อลอย → ถอดสาย serial · กด E-stop · ส่ง seq ย้อนหลัง · ยัด buffer · วัด deadband
         → ผ่านครบแล้วจึงวางลงพื้น
```

Safety ทั้งสามชั้นต้องถูกทดสอบ **ตอนที่การทดสอบล้มเหลวแล้วไม่มีอะไรวิ่งหนี** —
ข้อนี้ไม่มีใน design gantry เพราะ gantry เคลื่อนที่เองไม่ได้

### V4 — Real Machine

```text
2 Cameras → Perception → Controller (Pi) → serial → ESP32 → H-bridge → DC motor × 4 → แปลงจริง
```

เป้าหมาย: tune gain ใหม่ (slip ต่างจาก sim), วัด runaway distance จริง,
วัด homography error จริง

---

## 10. SIL / HIL

### Software-in-the-Loop

```text
Isaac Sim → Python Controller → IsaacRover
```

ทดสอบ robot logic โดยไม่ต้องมี Hardware

### Hardware-in-the-Loop

```text
Isaac Sim ──virtual camera──> Controller ──serial──> Real ESP32 ──wheel cmd──> Isaac Sim
```

ทดสอบ firmware จริงก่อนวางล้อลงพื้น

---

## 11. Production Architecture

```text
camera_front   camera_down
     │              │
     └──────┬───────┘
            ▼
     Raspberry Pi (on rover)
            │
            ├── Perception  (ExG, row estimate, weed detect)
            ├── RowFollower
            └── Controller  (state machine, safety, startup checks)
                   │
                   ▼ serial
                 ESP32
                   │
          ┌────────┼────────┐
          ▼        ▼        ▼
     H-bridge   E-stop   Watchdog
          │      sense
          ▼
  DC gear motor × 4  (ซ้าย/ขวา ต่อขนานเป็นข้าง)
```

Wi-Fi **ไม่อยู่ใน control path** — log เขียนไฟล์บน Pi ดึงทีหลัง

---

## 12. MVP Scope

### ทำ

```text
✓ เดินตามร่องด้วยกล้องหน้า (row following)
✓ ตรวจจับวัชพืชในร่องด้วยกล้องล่าง แล้วบันทึก log
✓ safety 3 ชั้น + startup invariants
```

### ไม่ทำ

```text
✗ ไม่กำจัดวัชพืช — ไม่มี tool ไม่มี servo
✗ ไม่เลี้ยวเข้าร่องถัดไป — วิ่งร่องเดียวแล้วหยุด
✗ ไม่มีพิกัดวัชพืชในแปลง — ไม่มี odometry
✗ ไม่ dedupe การตรวจจับ — log นับ detection ไม่ใช่นับต้น
✗ ไม่เห็นวัชพืชในแถว — เห็นแค่ในร่อง
✗ ไม่มี obstacle avoidance — ไม่มี bumper ไม่มี ToF
✗ ไม่มี turn-in-place — v = 0 กับ omega ใด ๆ อยู่ใน deadband ทั้งสองล้อ
✗ ไม่มี suspension
✗ ไม่มี ROS2 · ไม่มี MQTT · ไม่มี Wi-Fi ใน control path
```

---

## 13. Priority Order

สิ่งที่ควรให้ความสำคัญตั้งแต่ต้น:

1. Rover Interface
2. CameraSet Interface
3. RowFollower (pure function — test ได้ก่อนมีภาพ)
4. State Machine
5. Startup Invariants
6. Communication Protocol (streaming vs discrete)
7. Safety / E-Stop
8. Scenario Testing

`Coordinate Transform` หลุดจากลำดับต้น ๆ ที่ design gantry เคยให้ไว้ — เพราะ
row following ไม่ต้องใช้มันเลย และ MVP ไม่มีอะไรกระทำต่อตำแหน่งวัชพืช
มันกลับมาสำคัญที่ M2/M3
