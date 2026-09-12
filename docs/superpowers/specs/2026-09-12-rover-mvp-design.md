# Design: Rover 4 ล้อ ติดกล้อง — First MVP

**Date:** 2026-09-12
**Status:** Approved
**Supersedes:** `2026-09-12-weeding-bed-design.md` (D1, D2, D3, D8 และทุกส่วนที่ผูกกับ gantry kinematics)

---

## 1. Context

Design ก่อนหน้าเลือก **gantry คร่อมแปลง** (FarmBot style) โดย D1 ปฏิเสธ rover อย่างชัดเจน
ด้วยเหตุผลว่าจะได้ใช้ `Machine` interface และ gantry kinematics เดิมทั้งหมด
ไม่ต้องเพิ่ม navigation / odometry / localization

Design นี้**พลิก D1** — first MVP เป็น **rover skid-steer 4 ล้อ ติดกล้อง 2 ตัว**
และ gantry ถูกยกเลิกถาวร ไม่ใช่เลื่อน

ขอบเขต MVP แคบลงอย่างมีเจตนา: **วิ่งตามร่องและมองเห็นวัชพืช — ไม่กำจัด**
การกำจัดวัชพืชเลื่อนไป M3 พร้อมงาน tool ทั้งชุดที่ spec เดิมออกแบบไว้แล้ว

เหตุผลที่ยอมทิ้ง gantry: form factor ที่คร่อมแปลงขยายไม่ได้เกินความกว้างของโครง
rover พิสูจน์ mobility + perception ซึ่งเป็นความเสี่ยงที่ gantry ไม่เคยแตะ
และเป็นสิ่งที่ตัดสินว่าระบบใช้ในแปลงจริงได้หรือไม่

---

## 2. Decisions

| # | Decision | Rationale |
|---|---|---|
| R1 | **Rover แทน gantry ถาวร** — พลิก D1 | mobility เป็นความเสี่ยงที่ยังไม่ถูกพิสูจน์ และ gantry ขยายเกินความกว้างโครงไม่ได้ |
| R2 | **MVP = วิ่ง + มองเห็น ไม่มี tool** | ตัด servo, soil contact, tool reach invariant ออกทั้งชุด ทำให้พิสูจน์ mobility + perception ได้โดยไม่ต้องแก้ปัญหา actuator พร้อมกัน |
| R3 | **Row following ด้วยกล้อง ไม่มี odometry ไม่มีแผนที่** | เป็น autonomy ที่น้อยที่สุดที่ยังเจอปัญหา CV จริง โดยไม่ต้องเปิด SLAM |
| R4 | **Skid-steer 4WD** | เลี้ยวด้วยความต่างความเร็วซ้าย/ขวา interface เหลือ `(v, omega)` ไม่ต้องมี steering servo หมุนอยู่กับที่ได้ ผ่านดินขรุขระได้ดี |
| R5 | **Compute: Raspberry Pi บน rover ← serial → ESP32** | ไม่มี Wi-Fi ใน control path Pi แครช ESP32 ก็ยังดับมอเตอร์เองได้ |
| R6 | **กล้อง 2 ตัว — front (nav) + down (detect)** | สัญญาที่ต่างกันจริง: front ไม่ต้อง calibrate / down ต้องทำ homography รวมเป็นตัวเดียวจะบังคับให้ประนีประนอมทั้งสองงาน |
| R7 | **ยังไม่ใช้ ROS2** | MVP มี 1 หุ่นยนต์ 2 กล้อง ไม่มี SLAM ไม่มี path planning ไม่มี MoveIt — เงื่อนไขใน README เดิมยังไม่ครบ |
| R8 | **Simulation-first คงเดิม V0 → V4** | architecture หลักไม่ถูกแตะ เปลี่ยนเครื่องแต่ pattern การเลือก backend เหมือนเดิม |
| R9 | **Safety 3 ชั้น เจ้าของแยกกัน** | row-loss watchdog (controller) + command timeout (ESP32) + latching E-stop (hardware) แต่ละชั้นทำงานได้โดยไม่พึ่งชั้นบน |
| R10 | **`Machine` → `Rover` เปลี่ยนชื่อจริง** | interface เดิมถูกแทนเกือบทั้งหมด ไม่ใช่ extend การคงชื่อไว้จะทำให้ทั้ง repo อ่านเหมือน gantry ที่ถูกดัดแปลง — ความผิดพลาดแบบเดียวกับที่ D8 เดิมเตือนไว้ |
| R11 | **ไม่ทำ `Platform` interface รองรับทั้งสอง form factor** | abstraction ที่มี implementation เดียวตลอดกาลคือต้นทุนเปล่า YAGNI |
| R12 | **ไม่มี bumper switch** | ชดเชยด้วย `runaway_budget_mm` ที่ validate ตอน startup + ขอบแปลงยกสูงกว่าล้อ + ความเร็วต่ำ |
| R13 | **เก็บ spec เดิม 2 ฉบับเป็นประวัติ ไม่ลบ** | งาน tool reach invariant / soil contact / error code ของ tool ยังถูกต้องทั้งชุด M3 หยิบกลับมาใช้ได้ |
| R14 | **`body_width` = ความกว้าง *รวมล้อ* ไม่ใช่ความกว้างแชสซี** | ส่วนที่ชนใบพืชคือล้อ ที่ `track_width` 140 ความกว้างรวมเป็น 166 mm ทำให้ invariant ข้อ 5 เหลือ margin 4 mm จึงลด `track_width` เป็น 120 → รวม 146 mm margin 24 mm |

---

## 3. What Does NOT Change

```text
Backend selection ที่ startup    ไม่เปลี่ยน  (fake / isaac / esp32)
Layer boundary                  ไม่เปลี่ยน  (perception → controller → machine → hardware)
Controller ไม่รู้ว่าคุม sim หรือของจริง      ไม่เปลี่ยน
Development phases V0 → V4       ไม่เปลี่ยนโครง (เปลี่ยนเนื้อหาและน้ำหนัก)
CAD = geometry, config = operational (D9)   ไม่เปลี่ยน
# derived: cad  + test_cad_config_sync.py   ไม่เปลี่ยนกลไก
URDF → base USD (generated) + override layer (D10)   ไม่เปลี่ยน
Unit convention  config/protocol = mm+deg, URDF/USD = m+rad   ไม่เปลี่ยน
```

การที่เปลี่ยน form factor ทั้งเครื่องได้โดยไม่แตะ pattern การเลือก backend และเส้นแบ่งชั้น
คือหลักฐานว่า architecture เดิมคุ้มค่า — สิ่งที่เปลี่ยนคือ *เนื้อหา* ของ interface ไม่ใช่ *รูปแบบ* ของมัน

---

## 4. Interfaces

### 4.1 `Rover` แทน `Machine`

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

Sign convention (ผูกกับ `coordinate-frames.md`): `v > 0` เดินหน้า · `omega > 0` เลี้ยวซ้าย (CCW, right-hand rule, z ขึ้น)

`stop()` ต่างจาก `drive(0, 0)`: ทั้งคู่รีเฟรช watchdog แต่ `stop()` สั่ง brake ค้าง
ขณะที่ `drive(0, 0)` ปล่อยไหลตามแรงเฉื่อย — บนแปลงเอียงต่างกันจริง

### 4.2 `get_drive_state()` — ออกแบบให้โกหกไม่ได้

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

### 4.3 สิ่งที่ **ไม่** มีใน interface และเหตุผล

| ไม่มี | เพราะ |
|---|---|
| `drive_distance(mm)` | ไม่มี encoder จะ implement ได้แค่ `velocity × time` ซึ่งเป็นการอ้างความแม่นที่ไม่มีจริง ถ้าต้องการ ให้เพิ่ม encoder ไม่ใช่เพิ่ม method |
| `get_position()` | ไม่มี localization ไม่มีอะไรตอบได้ |
| `home()` | ไม่มี absolute frame ให้ home เข้าหา |
| `get_endstops()` | ไม่มี endstop `estop` ย้ายไปอยู่ใน `get_drive_state()` |
| `get_tool_state()` | ไม่มี tool ใน MVP (กลับมาที่ M3) |

interface ที่บอกได้ว่าตัวเองทำอะไร**ไม่ได้** มีค่ามากกว่า interface ที่มี method ครบสวย

### 4.4 `CameraSet` แทน `Camera`

```python
class CameraSet(Protocol):

    def front(self) -> Frame:
        """สำหรับ row following"""
        ...

    def down(self) -> Frame:
        """สำหรับ weed detection"""
        ...
```

เลือก **named accessor** ไม่ใช่ `capture(camera_id)` เพราะสองกล้องนี้ต่างกันที่ contract ไม่ใช่ต่างกันที่ index:

| | `front()` | `down()` |
|---|---|---|
| ใช้ทำ | row following | weed detection |
| Calibration | **ไม่ต้อง** — ใช้แค่มุม/offset ในภาพ | homography → mm |
| Rate | สูง (อยู่ใน control loop) | ต่ำ (นอก control loop) |

`capture(id)` จะซ่อนความต่างนี้ แล้วมีคนเผลอเอาภาพ `front` ไปทำ coordinate transform

Implementation: `FakeRover` · `IsaacRover` · `Esp32Rover` · `FakeCameraSet` · `IsaacCameraSet` · `UsbCameraSet`

---

## 5. Controller

### 5.1 `RowEstimate` — สัญญาระหว่าง perception กับ controller

```python
@dataclass(frozen=True)
class RowEstimate:
    valid: bool
    lateral_err: float      # [-1, 1]  ร่องเบี่ยงจากกลางภาพ  (+ = ร่องอยู่ขวา)
    heading_err: float      # [-1, 1]  ความเอียงของเส้นร่องในภาพ  (+ = เอียงขวา)
    confidence: float       # [0, 1]
    green_fraction: float   # [0, 1]  สัดส่วน pixel ที่ ExG ผ่าน threshold
```

**ไม่มีหน่วย mm หรือ deg ทั้ง struct โดยเจตนา** — กล้องหน้าไม่ calibrate
ค่าที่วัดได้จากภาพที่ไม่ calibrate คือ pixel ratio ถ้าประกาศเป็น mm หรือ deg
จะเป็นการอ้างหน่วยที่ไม่มีใครยืนยันได้ gain ของ controller จึงมีหน่วย
`(deg/s) / unitless` และ **ต้อง tune** — ยอมรับตรง ๆ ดีกว่าซ่อนใต้หน่วยปลอม

### 5.2 `RowFollower` — pure function

```python
class RowFollower:
    def __init__(self, k_lat: float, k_head: float,
                 v_mm_s: float, omega_max_deg_s: float): ...

    def step(self, est: RowEstimate) -> tuple[float, float]:
        """→ (v_mm_s, omega_deg_s)"""
```

```python
omega = -(k_lat * est.lateral_err + k_head * est.heading_err)
omega = clamp(omega, -omega_max, +omega_max)
return (v_mm_s, omega)
```

`RowFollower` **ไม่รู้จัก camera และไม่รู้จัก rover** — รับ `RowEstimate` คืน tuple
ทำให้ test ได้ด้วยตัวเลขสังเคราะห์ ไม่ต้องมีภาพ ไม่ต้องเปิด Isaac

**ทำไม P ไม่ใช่ PID**: `heading_err` คืออัตราการเปลี่ยนของ `lateral_err` ในเชิงเรขาคณิตอยู่แล้ว
`k_head` จึงทำหน้าที่เป็นเทิร์ม damping โดยไม่ต้องหาอนุพันธ์จากสัญญาณภาพที่มี noise
ไม่เอา integral เพราะ offset ค้างในร่องไม่คุ้มกับความเสี่ยง windup

ความเร็ว `v` คงที่ใน MVP — การลดความเร็วเมื่อเบี่ยงมากเป็น knob ที่เพิ่มภายหลังได้

### 5.3 State machine

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

MVP วิ่ง **ร่องเดียวแล้วหยุด** — การเลี้ยวเข้าร่องถัดไปต้องเลี้ยวโดยมองไม่เห็นร่องเป้าหมาย
ซึ่งไม่มี odometry มารองรับ จึงอยู่นอก MVP (→ M4)

### 5.4 `row_end` กับ `row_lost` แยกกันด้วย `green_fraction`

ทั้งสองกรณีคือ "estimate invalid ติดกัน `row_loss_frames` เฟรม" ตัวแยกมีตัวเดียว:

| `green_fraction` ตอน invalid | ตีความ | ปลายทาง |
|---|---|---|
| ต่ำ — สีเขียวหมดจากเฟรม | สุดร่องแล้ว | `STOPPED(row_end_suspected)` |
| สูง — ยังเห็นเขียวแต่จับเส้นไม่ได้ | navigation พลาด | `ERROR(row_lost)` |

ชื่อ `row_end_suspected` ตั้งให้เตือนว่า **นี่คือ heuristic ไม่ใช่การวัด**
ต้องมี scenario test ทั้งสองทาง (`row_end.yaml` + `crop_gap_midrow.yaml`)
ยืนยันว่า threshold แยกได้จริง ถ้าแยกไม่ได้ เราจะรู้ใน sim ไม่ใช่ในแปลง

### 5.5 Control loop

```python
while state is DRIVING_ROW:
    est = row_estimator.estimate(cameras.front())    # 10 Hz
    if not est.valid:
        watchdog.tick(est)
        continue
    watchdog.reset()
    rover.drive(*follower.step(est))
    weed_log.maybe_record(cameras.down())            # 1-2 Hz นอก control path
```

`weed_log` อยู่ในลูปแต่**ไม่อยู่ใน control path** — detection ช้าหรือพลาดต้องไม่ทำให้ rover เลี้ยวผิด
front frame ย่อเหลือ ~320×240 ก่อนทำ ExG เพื่อให้ Pi ทัน 10 Hz

### 5.6 Safety — 3 ชั้น 3 เจ้าของ

| กลไก | เจ้าของ | เงื่อนไข trip | การทำงาน |
|---|---|---|---|
| Command timeout | **ESP32 firmware** | ไม่ได้รับ velocity command เกิน `command_timeout_ms` | มอเตอร์ดับ |
| Row-loss watchdog | **controller** | estimate invalid ติดกัน `row_loss_frames` | `stop()` → STOPPED/ERROR |
| Latching E-stop | **hardware** | คนกด | ตัด motor rail ทางไฟ |

หลักการ: **แต่ละชั้นทำงานได้โดยไม่ต้องพึ่งชั้นบน** — ESP32 timeout ไม่พึ่ง Pi
(Pi แครชหรือ USB หลุด มอเตอร์ก็ดับ) · E-stop ไม่พึ่ง firmware (firmware แฮงค์ก็ยังตัดไฟได้)
ถ้าชั้นใดพึ่งชั้นบน มันไม่ใช่ safety layer มันคือ feature

### 5.7 Module responsibility ที่เปลี่ยน

| Module | เพิ่ม | ตัดออก |
|---|---|---|
| `controller/` | `row_follower.py`, `startup_checks.py`, `safety/runaway.py` | waypoint motion planning, homing, tool sequencing |
| `firmware/esp32/` | skid-steer mixing, command timeout, motor enable, sense line E-stop | stepper/TMC2209, endstop, servo, soil contact |
| `perception/` | `exg.py`, `row_estimator.py`, `weed_detector.py`, `weed_log.py` | coordinate transform สำหรับ nav (ไม่ต้องมี), `row_detection.py` (ไม่สร้าง) |

---

## 6. Environment & Isaac Sim

### 6.1 เรขาคณิตแปลง

```text
Bed                       2000 × 1000 mm  (ยาว × กว้าง)
Crop rows                 3 แถว ตามยาว
row_spacing               350 mm      กึ่งกลางแถวถึงกึ่งกลางแถว
crop_foliage_half_width   30 mm       ใบยื่นออกจากกึ่งกลางแถวข้างละเท่านี้
clear furrow              290 mm      350 − 2 × 30  ◄── ค่าที่ rover ใช้จริง
crop_spacing              80 mm ตามแถว
Edge margin               150 mm แต่ละข้าง
Furrows                   2 ร่องระหว่างแถว — MVP วิ่งร่องเดียว
Soil                      heightfield rigid ±15 mm ไม่ยุบ  (คงจาก design เดิม)
BedFrame                  ขอบยกสูงกว่า wheel_diameter/2
```

**`row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้** — ใบพืชยื่นเข้ามาข้างละ ~30 mm
ความกว้างที่ใช้จริงคือ *clear furrow* = `row_spacing − 2 × crop_foliage_half_width`
invariant ทุกข้อที่เกี่ยวกับช่องว่างด้านข้างต้องใช้ค่านี้ ไม่ใช่ `row_spacing`

`row_spacing = 350 mm` มาจาก invariant ไม่ใช่จากการเลือก:

```text
clear_furrow > body_width + 2 × runaway_budget

ที่ 250 mm:  190 > 266   ไม่ผ่าน
ที่ 300 mm:  240 > 266   ไม่ผ่าน
ที่ 350 mm:  290 > 266   ผ่าน  margin 24 mm
```

เลือกขยายระยะแถวแทนการลด `runaway_budget` เพราะการลด budget ต้องไปบีบ
`command_timeout` หรือ `v_max` ซึ่งกระทบทั้ง control loop

`crop_foliage_half_width_mm = 30` เป็น **ค่าประมาณที่ต้องวัดที่ V1** จาก asset พืชที่ใช้จริง
ถ้าใบกว้างกว่านี้ clear furrow แคบลงและต้องขยาย `row_spacing` อีก

**BedFrame ขอบยกสูงกว่าล้อคือ compensating control ของ R12 (ไม่มี bumper)**
ต้องมีทั้งใน sim และของจริง ไม่ใช่ของประดับ

### 6.2 Scene model

```text
World
├── Bed
│   ├── SoilSurface              heightfield rigid  ±15 mm
│   ├── BedFrame                 ขอบยก > wheel_diameter/2
│   ├── CropRow_1 .. CropRow_3
│   │   └── Crop_001 ..
│   └── Weed_001 .. Weed_M       ในร่อง / ในแถว
│
├── Rover
│   ├── base_link
│   ├── wheel_fl · wheel_fr · wheel_rl · wheel_rr    continuous joint, velocity drive
│   ├── camera_front             tilt ~45° มองไปข้างหน้า
│   └── camera_down              top-down
│
└── Lighting
```

**ไม่มี suspension** — chassis แข็ง 4 ล้อ บนผิว ±15 mm จะมีจังหวะล้อลอยและเสียแรงขับ
นี่ไม่ใช่ข้อบกพร่องของ sim ที่ต้องแก้ ถ้าเกิดใน sim มันจะเกิดของจริงด้วย —
ให้ sim บอกก่อนซื้อล้อ

### 6.3 Skid-steer mixing — จุดที่โค้ดจะแตกเป็นสองชุด

```text
omega_rad   = omega_deg × pi / 180
v_left      = v - omega_rad × track_width_mm / 2
v_right     = v + omega_rad × track_width_mm / 2
wheel_rad_s = v_side / (wheel_diameter_mm / 2)
```

สูตรนี้ต้อง implement **สองที่ที่แชร์โค้ดกันไม่ได้**: Python ใน `IsaacRover`
และ C++ ใน ESP32 firmware ถ้าปล่อยไว้ ทั้งสองจะค่อย ๆ เพี้ยนจากกัน
แล้ว gain ที่ tune ใน sim จะใช้กับของจริงไม่ได้โดยไม่มีใครรู้ว่าทำไม

**วิธีจัดการ: golden test vector เป็น source of truth เดียว**

```text
config/drive_mixing_vectors.csv    v_mm_s, omega_deg_s, v_left_mm_s, v_right_mm_s
        │
        ├──> tests/unit/test_drive_mixing.py          (Python / IsaacRover)
        └──> firmware/esp32/test/test_mixing.cpp      (PlatformIO native env)
```

ทั้งสอง implementation ต้องผ่านตารางเดียวกัน ไม่ต้องเขียน code generator —
ตารางค่าเป็นสัญญาที่ทั้งสองภาษาอ่านได้

**Saturation ต้อง scale ไม่ใช่ clip**: ถ้าล้อข้างหนึ่งเกิน `wheel_v_max_mm_s`
ให้ลดทั้งสองข้างตามอัตราส่วนเดิม การ clip ข้างเดียวจะเปลี่ยน `omega` ที่ได้จริงโดยเงียบ

**ข้อจำกัดที่ต้องประกาศ**: skid-steer จริงมี slip ทำให้ `omega` ที่ได้ ≠ `omega` ที่สั่ง
สูตรนี้เป็น *nominal* ไม่ใช่ค่าจริง ระบบทนได้เพราะ row follower ปิดลูปด้วยภาพ
แต่หมายความว่า **gain ต้อง tune ใหม่บน hardware** — บันทึกแยกตาม backend

### 6.4 Environment generator

```python
generate_bed(
    bed_length_mm=2000,
    crop_rows=3,
    row_spacing_mm=350,
    crop_spacing_mm=80,
    weed_count=20,
    soil_variation_mm=15,
    row_curvature_mm=40,          # ใหม่
    crop_gap_probability=0.08,    # ใหม่
)
```

สอง parameter ใหม่สำคัญกว่าที่เหลือทั้งหมด:

| Parameter | ป้องกันอะไร |
|---|---|
| `row_curvature_mm` | ถ้าร่องตรงเป๊ะทุก run `RowFollower` จะผ่านด้วย gain = 0 ก็ได้ เราจะไม่รู้ว่ามัน track ได้จริงไหมจนลงแปลง |
| `crop_gap_probability` | ต้นหายกลางแถวคือกรณีที่ `green_fraction` ตกชั่วคราว — heuristic ใน §5.4 จะพังที่นี่ ต้องเจอใน sim |

คงจาก design เดิม: soil seed/ความขรุขระ, texture/ความชื้นดิน, ตำแหน่ง-ขนาด-หมุนวัชพืช,
jitter ตำแหน่งพืชในแถว, มุมเอียงแถวทั้งชุด, lighting, camera noise

### 6.5 Asset & path

```text
sim/isaac/robots/gantry/     ──>  sim/isaac/robots/rover/
  gantry_base.usd                   rover_base.usd   generated, gitignored
  gantry.usd                        rover.usd        physics tuning, commit

cad/fusion/gantry.f3d        ──>  cad/fusion/rover.f3d
cad/urdf/weeding_gantry.urdf ──>  cad/urdf/weeding_rover.urdf

คงชื่อ  sim/isaac/scenes/weeding_bed.usd  (เนื้อหาเป็นร่องยาว)
คงแผน  scenes/bed_frame.usd · scenes/soil_heightfield.usd · scenes/crops/ · scenes/weeds/
```

`rover.usd` override layer เก็บ: wheel drive gain/damping, friction ของล้อกับ heightfield,
contact offset, camera intrinsic — ค่าที่ URDF เก็บไม่ได้ เหตุผลเดียวกับ D10 เดิม

### 6.6 ความเสี่ยง physics ที่ gantry ไม่มี

Prismatic joint ไม่มี contact dynamics แต่ล้อมีทั้งหมด ต้อง budget เวลา tune:

1. **Friction** — ต้องพอให้ขับไปข้างหน้า แต่ต้องให้ slip ได้ตอนเลี้ยว skid-steer
   ถ้าตั้งสูงเกินจะเลี้ยวไม่ออกหรือ solver ระเบิด
2. **ล้อลอยบน heightfield** — แรงขับหายเป็นช่วง
3. **Timestep** — contact ที่ความเร็วต่ำบนผิวขรุขระอาจต้อง substep ถี่กว่าเดิม กระทบเวลารัน CI

**นี่เป็นงานที่ใช้เวลามากที่สุดของ V1** มากกว่า row follower — วางแผนเวลาตามนี้

---

## 7. Perception

```text
                  ┌── exg.py (shared) ──┐
                  ▼                     ▼
camera.front ─> row_estimator.py    weed_detector.py <─ camera.down
                  │                     │
                  ▼                     ▼
             RowEstimate            Detection[]  ──> weed_log.py
             (control path)         (observation path)
```

### 7.1 `exg.py` — ExG + adaptive threshold + absolute floor

```python
exg = 2*G - R - B
if exg.max() < exg_floor:
    return Mask.empty()        # ไม่มีเขียวจริง — ไม่เรียก Otsu
mask = exg > otsu(exg)
```

ใช้ **Otsu ต่อเฟรม** ไม่ใช่ค่าคงที่ เพราะสีดินเปลี่ยนตามความชื้นและแสง
ค่าคงที่จะต้อง tune ใหม่ทุกครั้งที่รดน้ำ

แต่ Otsu มีจุดตายที่กระทบ heuristic ใน §5.4 โดยตรง: **เมื่อไม่มีสีเขียวในเฟรมเลย
Otsu จะแบ่ง noise ออกเป็นสองกอง** แล้วคืน mask ที่ดูเหมือนมีพืช ทำให้ `row_end` ตรวจไม่เจอ
`exg_floor` ป้องกันข้อนี้ และเป็นค่าเดียวใน pipeline ที่ต้อง tune ด้วยมือ
ต้องมี unit test ป้อนภาพดินเปล่ายืนยันว่าได้ mask ว่าง

### 7.2 `row_estimator.py` — valley ของ column histogram ไม่ใช่ line fitting

สิ่งที่ rover ต้องตามคือ *ร่องดิน* ซึ่งคือหุบเขาระหว่างแถบเขียวสองแถบ

```python
valley_near = find_valley(mask[bottom_third])    # ใกล้ล้อ
valley_far  = find_valley(mask[middle_third])    # lookahead

lateral_err = (valley_near - u_center) / (width / 2)
heading_err = (valley_far - valley_near) / (width / 2)
confidence  = valley_prominence                  # ความลึกของหุบเทียบยอดสองข้าง
valid       = confidence > conf_min  (ทั้งสองแถบ)
```

| | valley histogram | line fitting |
|---|---|---|
| ต้นหายกลางแถว | ทนได้ — histogram ยังเห็นแถบ | เสี่ยง fit เพี้ยนเพราะ blob หาย |
| ร่องโค้ง | ทนได้ — วัดสองระยะแยกกัน ไม่บังคับเป็นเส้นตรง | ต้อง fit เส้นโค้ง ซับซ้อนขึ้น |
| ต้นทุน | numpy sum + argmin ทัน 10 Hz บน Pi สบาย | Hough แพงกว่ามาก |
| Debug | plot histogram เห็นทันทีว่าพลาดที่ไหน | ต้องดู blob + parameter หลายตัว |

`heading_err` จากผลต่างของ valley สองระยะ **ไม่ต้องรู้ geometry กล้องเลย** —
สอดคล้องกับ R6 ที่ว่ากล้องหน้าไม่ calibrate

`valley_prominence` เป็น confidence ที่มีความหมายทางกายภาพ: ถ้าแถบเขียวสองข้าง
แยกจากหุบไม่ชัด แปลว่าไม่เห็นร่อง ซึ่งควรทำให้ `valid = False` จริง ๆ

### 7.3 `weed_detector.py` — เรขาคณิตของ rover ทำงานแทน row fitting

design เดิมต้อง fit เส้นแถวเพื่อแยก crop จาก weed เพราะ gantry มองทั้งแปลงจากด้านบน
พืชผลกับวัชพืชอยู่ในเฟรมเดียวกัน

rover กลับด้านปัญหานี้: **กล้องล่างเล็งที่ร่อง และพืชผลไม่ได้ปลูกในร่อง**

```text
สีเขียวในร่อง = วัชพืช  โดยนิยาม
```

```python
mask = exg_mask(down_frame)
for blob in blobs(mask):
    if blob.touches_side_edge:                     continue   # ใบพืชผลที่ล้ำเข้ามา
    if blob.centroid_x_mm not in furrow_corridor:  continue
    if blob.area_mm2 < min_weed_area_mm2:          continue
    yield Detection(...)
```

`furrow_corridor` ตั้งจากเรขาคณิต:

```text
clear furrow   row_spacing − 2 × crop_foliage_half_width   350 − 60 = 290 mm
corridor       clear_furrow − 2 × max_lateral_error_mm     290 − 80 = 210 mm
```

`corridor` คือแถบที่**รับประกันว่าอยู่ในร่อง**แม้ rover เบี่ยงเต็มพิสัย — สีเขียวในแถบนี้
จึงเป็นวัชพืชได้โดยไม่ต้องรู้ว่าแถวอยู่ไหน

สองค่าที่ประกอบกันเป็น `corridor` เป็น **สมมติฐานที่ต้องวัดที่ V1 ทั้งคู่**:

| ค่า | ตั้งไว้ | ถ้าวัดจริงแล้วมากกว่า |
|---|---|---|
| `max_lateral_error_mm` | 40 | corridor แคบลง recall ตก |
| `crop_foliage_half_width_mm` | 30 | clear furrow แคบลง กระทบ invariant §9.5 ข้อ 5 ด้วย |

ต้องบันทึกค่าที่วัดได้ ไม่ใช่คงค่าเดา — และถ้าค่าที่วัดได้ทำให้ `corridor <= 0`
แปลว่าแปลงแคบเกินไปสำหรับ rover ขนาดนี้ ไม่ใช่ว่า detector ต้อง tune ใหม่

กฎ `touches_side_edge` จัดการใบพืชผลที่ยื่นเข้าร่องโดยไม่ต้องรู้ว่ามันคือพืชผล —
ใบที่ต่อเนื่องออกไปนอกเฟรมย่อมไม่ใช่ต้นเล็กที่อยู่กลางร่อง

**ข้อจำกัด: วัชพืชในแถว (in-row weed) MVP มองไม่เห็น** — เป็นกรณียากที่สุดของปัญหาจริง
และอยู่นอก MVP โดยเจตนา เพราะการกำจัดมันต้องมี tool ซึ่งไม่มีใน MVP
ไม่ควรทำให้ perception ยากกว่าที่ actuator ทำได้

### 7.4 Calibration — เหลือแค่กล้องล่าง

| | กล้องหน้า | กล้องล่าง |
|---|---|---|
| Calibration | **ไม่มี** | homography → soil reference plane |
| Accuracy target | – | **±20 mm** (ผ่อนจาก ±8 mm เดิม) |

ผ่อนได้เพราะ **MVP ไม่มีอะไรกระทำต่อตำแหน่ง** — ไม่มี tool ที่ต้องเล็ง ตัวเลข mm
ใช้แค่บันทึกขนาด/ตำแหน่งคร่าว ๆ งาน calibration ระดับ ±8 mm (รัศมีหัว tool) เก็บไว้ทำที่ M3

แต่ยัง**ต้องทำ** homography ตอนนี้ ไม่เลื่อนทั้งหมด — เพื่อรู้ตัวเลข error budget จริง
บนดิน ±15 mm **ก่อน**ออกแบบ tool ไม่ใช่หลังจากนั้น

### 7.5 `weed_log.py` — และข้อจำกัดที่ต้องไม่ให้ใครเข้าใจผิด

```jsonl
{"t":12.34,"frame":123,"lateral_err":0.12,"u":160,"v":200,
 "x_mm":-25,"y_mm":120,"area_mm2":480}
```

**ไม่มี field พิกัดในแปลง และไม่มี weed id** — ไม่มี odometry จึงไม่มีทางรู้ว่า
วัชพืชที่เห็นในเฟรม 123 กับ 124 เป็นต้นเดียวกันหรือคนละต้น

ผลที่สำคัญที่สุด: **log นี้นับ "การตรวจจับ" ไม่ใช่ "จำนวนวัชพืช"**
ตัวเลขรวมจาก log ไม่มีความหมาย **ห้ามใช้เป็น metric**

acceptance criteria จึงวัด **precision / recall ต่อเฟรม** เทียบ ground truth ที่ sim รู้อยู่แล้ว:

```text
per-frame precision >= 0.9
per-frame recall    >= 0.8   (วัดเฉพาะวัชพืชในร่อง)
```

recall ตั้งต่ำกว่า precision โดยเจตนา — MVP ไม่กำจัดอะไร การพลาดต้นหนึ่งไม่เสียหาย
แต่การรายงานดินเป็นวัชพืชจะทำให้ threshold ถูก tune ผิดทางตั้งแต่ต้น

---

## 8. Protocol

### 8.1 ต้องแยก command เป็นสองชนิด

Rule 4 เดิมของ `protocol/messages.md` เขียนว่า *"Command ที่ไม่ได้ `ack` ภายใน timeout
→ Controller เข้าสู่ `ERROR`"* — ถูกต้องสำหรับ `move`/`home`/`tool` ที่สั่งครั้งเดียวแล้วรอผล

แต่ velocity command ส่ง 10 ครั้งต่อวินาทีตลอดเวลา ถ้าใช้ pattern เดิม
**เฟรมที่ตกหนึ่งเฟรมจะกลายเป็น ERROR** และจะมี ack ไหลกลับ 10 Hz ที่ไม่มีใครใช้

| | Streaming | Discrete |
|---|---|---|
| ตัวอย่าง | `drive` | `stop` · `emergency_stop` · `reset` |
| `ack` | **ไม่มี** | มี (pattern เดิม) |
| ตกหล่นได้ | ได้ — ตัวถัดไปมาใน 100 ms | ไม่ได้ |
| Semantics | latest-wins | ต้องทำทุกตัว |
| ยืนยัน liveness | telemetry echo `last_seq` | `ack` |

**Rule 4 ถูกจำกัดขอบเขตให้ใช้กับ discrete command เท่านั้น** — เป็นการแก้กฎ protocol ไม่ใช่แค่เพิ่ม message

### 8.2 Messages

```json
// Pi → ESP32   streaming, 10 Hz, ไม่มี ack
{"type":"drive","seq":1234,"v_mm_s":100.0,"omega_deg_s":-12.5}

// Pi → ESP32   discrete
{"type":"stop","id":77}
{"type":"emergency_stop","id":78}
{"type":"reset","id":79}

// ESP32 → Pi   telemetry, 20 Hz
{"type":"rover_state",
 "commanded":{"v_mm_s":100.0,"omega_deg_s":-12.5},
 "last_seq":1234,
 "estop":false,
 "motors_enabled":true,
 "uptime_ms":48210}
```

### 8.3 `seq` + `last_seq` — วัด `link_age_ms` โดยไม่ต้องซิงก์นาฬิกา

Pi จำเวลาที่ส่ง `seq` แต่ละตัว · ESP32 echo `last_seq` ที่รับได้ · Pi คำนวณ

```python
link_age_ms = now() - sent_at[state["last_seq"]]
```

ไม่ต้องมี clock sync ระหว่าง Pi กับ ESP32 และไม่ต้องมี ack แยก —
telemetry ที่ต้องส่งอยู่แล้วทำหน้าที่นี้ไปพร้อมกัน

### 8.4 Latest-wins ต้องบังคับที่ฝั่ง ESP32

serial buffer สะสมได้ ถ้า ESP32 ประมวลผลเรียงตามคิว มันจะเลี้ยวตามคำสั่งเก่า
ที่ Pi ยกเลิกความคิดไปแล้ว

```text
ทุกรอบ loop:  อ่าน line ทั้งหมดที่มีใน buffer
              ใช้ drive ตัวที่ seq สูงสุด  ทิ้งที่เหลือ
              seq <= last_seq  →  ทิ้ง (out of order)
```

`stop` / `emergency_stop` ที่พบใน buffer ต้อง**ทำทันทีไม่ว่าลำดับไหน**
และ override `drive` ทุกตัวในรอบนั้น

### 8.5 Error codes

```text
เพิ่ม                          ตัดออก
link_lost                     endstop_error
command_timeout               motor_timeout      (→ link_lost / command_timeout)
row_lost                      position_mismatch
config_invalid                tool_no_contact
                              tool_overreach

คงเดิม  camera_timeout · communication_lost · emergency_stop
```

| Code | ผู้ตรวจพบ | การจัดการ |
|---|---|---|
| `command_timeout` | ESP32 | มอเตอร์ดับเอง แล้วรายงานกลับเมื่อ link กลับมา — controller ต้องถือว่า rover หยุดแล้วจริง **ไม่ resume เอง** |
| `link_lost` | Pi | `link_age_ms` เกิน `link_lost_ms` → ERROR |
| `row_lost` | Pi | ตาม §5.4 |
| `config_invalid` | Pi ตอน startup | ไม่ start |

`command_timeout` กับ `link_lost` เป็นเรื่องเดียวกันที่มองจากสองฝั่ง และ**ต้องมีทั้งคู่** —
ถ้ามีแค่ฝั่ง Pi แล้ว Pi แครช ไม่มีใครสั่งหยุด

### 8.6 Transport

```text
เดิม  MQTT / Serial        ใหม่  Serial เท่านั้น  (Pi ↔ ESP32 บนบอร์ดเดียวกัน)
```

**MQTT ออกจาก MVP** — Wi-Fi ไม่อยู่ใน control path อีกแล้ว log เขียนไฟล์บน Pi ดึงทีหลัง

Bandwidth: ขึ้น ~70 B × 10 Hz = 700 B/s · ลง ~140 B × 20 Hz = 2.8 KB/s
ที่ **115200 baud** (11.5 KB/s) ใช้ ~30% มี headroom พอเพิ่ม loop rate เป็น 50 Hz
โดยไม่ต้องเปลี่ยน transport

---

## 9. Config & Invariants

### 9.1 โครงไฟล์

| File | Scope | สถานะ |
|---|---|---|
| `rover.yaml` | geometry (derived: cad), drive limit, track width | **แทน** `machine.yaml` |
| `safety.yaml` | runaway budget, timeout, row-loss frames | **ใหม่** |
| `perception.yaml` | `exg_floor`, `conf_min`, `loop_hz`, `camera_timeout_ms`, `max_lateral_error_mm`, `min_weed_area_mm2` | **ใหม่** |
| `control.yaml` | row follower gain **แยกตาม backend** | **ใหม่** |
| `simulation.yaml` | physics, bed generator, randomization | แก้ |
| `camera.yaml` | front (ไม่ calibrate) + down (homography, error budget) | แก้ |
| `development.yaml` | backend selection, logging | แก้เล็ก |

### 9.2 `rover.yaml`

```yaml
rover:
  track_width_mm: 120          # derived: cad track_width
  wheelbase_mm: 120            # derived: cad wheelbase
  body_width_mm: 146           # derived: cad body_width  (รวมล้อ = track + wheel_width)
  wheel_diameter_mm: 65        # derived: cad wheel_diameter
  chassis_clearance_mm: 35     # derived: cad chassis_clearance

  drive:
    v_max_mm_s: 100
    omega_max_deg_s: 40
    wheel_v_max_mm_s: 340      # derived: cad wheel_diameter + มอเตอร์ 100 RPM
    wheel_v_min_mm_s: 51       # deadband — ต้องวัดจริงที่ V3

bed:
  soil_variation_mm: 15
  row_spacing_mm: 350
  crop_foliage_half_width_mm: 30    # ประมาณ — ต้องวัดจาก asset พืชที่ V1
```

ตัดออกจาก `machine.yaml` เดิม: `workspace` · `x_axis` · `y_axis` · `home_offset_mm` · `tool` · `servo`

กลไก `# derived: cad` และ `tests/unit/test_cad_config_sync.py` **คงเดิมทั้งชุด** (D9)
เปลี่ยนแค่รายการพารามิเตอร์

### 9.3 `safety.yaml`

```yaml
safety:
  runaway_budget_mm: 60
  row_loss_frames: 3
  command_timeout_ms: 300
  link_lost_ms: 500
  enable_estop: true
```

`link_lost_ms (500) > command_timeout_ms (300)` โดยเจตนา — ESP32 ต้องดับมอเตอร์
**ก่อน**ที่ Pi จะประกาศ error เพื่อให้ลำดับเหตุการณ์ใน log อ่านได้ว่าอะไรเกิดก่อน
ถ้ากลับกัน Pi จะเข้า ERROR ขณะที่ล้อยังหมุนอยู่ 200 ms

### 9.4 `control.yaml` — gain ต้องแยกตาม backend

```yaml
row_follower:
  v_mm_s: 100
  omega_max_deg_s: 40

  gains:
    fake:  {k_lat: 45.0, k_head: 25.0}
    isaac: {k_lat: 45.0, k_head: 25.0}
    esp32: {k_lat: null, k_head: null}    # ยังไม่ tune บนของจริง
```

`null` ไม่ใช่ค่าว่างที่ถูกลืม — เป็น **guard**: ถ้า `backend: esp32` แล้ว gain เป็น `null`
ให้ fail ตอน startup ด้วย `config_invalid`

เหตุผล: slip ของ skid-steer บนดินจริงไม่เท่ากับใน sim (§6.3) การปล่อยให้ค่า sim
ถูกใช้เป็น default บนของจริงคือ rover ที่แกว่งเข้าหาต้นพืชโดยไม่มีใครรู้ว่าทำไม
**การบังคับให้ประกาศว่า "ยังไม่ได้ tune" ถูกกว่าการค้นหาสาเหตุในแปลง**

`camera_front_tilt_deg` ต้องมีคำเตือนกำกับในไฟล์นี้ด้วย (ดู §11.5)

### 9.5 Startup invariants — 9 ข้อ ตรวจที่ `controller/startup_checks.py`

```text
safety      v_max × command_timeout/1000        <= runaway_budget        30 <= 60   ok
safety      v_max × row_loss_frames/loop_hz     <= runaway_budget        30 <= 60   ok
geometry    wheel_diameter        >= 4 × soil_variation                  65 >= 60   ok
geometry    chassis_clearance     >  soil_variation                      35 >  15   ok
geometry    clear_furrow          >  body_width + 2 × runaway_budget    290 > 266   ok
              โดย clear_furrow = row_spacing − 2 × crop_foliage_half_width
drive       v + omega_max_rad × track/2  <= wheel_v_max               141.9 <= 340  ok
drive       v − omega_max_rad × track/2  >= wheel_v_min                58.1 >=  51
config      gains[backend] ไม่เป็น null
consistency simulation.soil.variation_mm == bed.soil_variation_mm
```

ทุกข้อ fail = **ไม่ start** + บอกค่าที่ขัดกันเป็นตัวเลข ไม่ใช่ warning

ข้อ `consistency` ยกมาจาก `config/README.md` เดิมที่เขียนว่า *"ควร validate
ความสอดคล้องนี้ตอน startup"* — ของเดิมเป็นคำแนะนำที่ไม่มีใครบังคับ คราวนี้อยู่ในไฟล์เดียวกับที่เหลือ

ข้อ `drive` ตัวล่างผ่านแบบพอดีขอบ ค่า `wheel_v_min_mm_s` เป็นค่าประมาณจาก 15% duty
ถ้าวัดจริงที่ V3 ได้เกิน 51 **ต้องลด `omega_max` ลงอีก ห้ามลด `wheel_v_min` เพื่อให้ผ่าน**

`runaway_budget_mm` ต้องเล็กกว่าระยะจากกันชนหน้า rover ถึงขอบแปลงและถึงต้นพืชที่ใกล้สุด —
**ต้องวัดจากแปลงจริงแล้วบันทึก** ไม่ใช่เดา

---

## 10. Phases, Milestones, Acceptance

### 10.1 V0 — FakeRover ปิดลูปได้โดยไม่ต้องมีกราฟิก

```text
FakeRover        integrate (v, omega) → pose ภายใน    (kinematic only)
FakeRowSensor    pose + สมการเส้นร่อง → RowEstimate
RowFollower      RowEstimate → (v, omega)
                        │
                        └──> วนกลับเข้า FakeRover
```

ลูปควบคุมทั้งวง test ได้ **โดยไม่เปิด Isaac ไม่มีภาพ ไม่มี physics** รันในหลักมิลลิวินาที อยู่ใน CI ได้:

```python
def test_converges_from_offset():
    sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
    sim.run(seconds=3.0)
    assert abs(sim.history[-1].lateral_err) < 0.1
    assert sim.min_clearance_mm > 0        # ไม่เบียดต้นพืชระหว่างทาง
```

ตอบคำถาม *"gain ชุดนี้ลู่เข้าไหม และแกว่งเกินร่องไหม"* ได้ก่อนแตะ Isaac —
คำถามที่แพงที่สุดถ้าต้องรอ physics ทุกรอบ

**ข้อควรระวังที่ต้องเขียนกำกับ**: `FakeRover` มี pose ภายใน แต่ **ห้าม expose
ผ่าน `get_drive_state()`** ถ้าหลุดออกมา code ที่เขียนตอน V0 จะพึ่งพา pose
แล้วพังทั้งหมดตอนเปลี่ยนเป็น `esp32` ที่ไม่มี pose ให้ — `get_drive_state()`
ของทุก backend ต้องคืน key ชุดเดียวกันเป๊ะ และมี test บังคับข้อนี้

### 10.2 Phases

| | เดิม (gantry) | ใหม่ (rover) |
|---|---|---|
| **V0** | state machine, motion interface, safety, protocol | + **closed loop ด้วย FakeRover** · golden mixing vectors · startup invariants |
| **V1** | home, move, tool, endstop, soil contact, collision | rover USD · **physics tuning (งานหนักสุด)** · row_estimator บนภาพ render · scenario ทั้งชุด |
| **V2** | ExG + row fitting → transform → move → tool | weed detector กล้องล่าง + **evaluation harness เทียบ ground truth** |
| **V3** | firmware จริง + virtual bed | ESP32 + motor driver บนโต๊ะ **ล้อลอย** — mixing, timeout, E-stop, latest-wins, วัด deadband |
| **V4** | camera + ESP32 + TMC2209 + NEMA17 + แปลงจริง | rover จริงในแปลงจริง — **tune gain ใหม่** · วัด runaway budget · วัด homography error |

ลำดับ V0 → V4 ไม่เปลี่ยน แต่ **น้ำหนักย้าย**: gantry หนักที่ V2 (แยก crop/weed)
rover หนักที่ V1 (physics ของล้อ) ต้องวางแผนเวลาตามนี้ ไม่ใช่ตามรูปแบบเดิม

### 10.3 V3 ต้องทำบนโต๊ะล้อลอยก่อน

ข้อนี้ไม่มีใน design เดิมเพราะ gantry เคลื่อนที่เองไม่ได้ safety ทั้งสามชั้นต้องถูกทดสอบ
**ตอนที่การทดสอบล้มเหลวแล้วไม่มีอะไรวิ่งหนี**

```text
ยกล้อลอยบนโต๊ะ → ถอดสาย serial · กด E-stop · ส่ง seq ย้อนหลัง · ยัด buffer · วัด deadband
                → ผ่านครบแล้วจึงวางลงพื้น
```

### 10.4 MVP Acceptance Criteria

```text
V0  ไม่ต้องมี Isaac
[ ] FakeRover closed loop: เริ่มเบี่ยง 60 mm + เอียง 15° → |lateral_err| < 0.1 ใน 3 s
[ ] ระหว่างลู่เข้า ไม่มีจังหวะที่ระยะถึงต้นพืชติดลบ
[ ] golden mixing vectors ผ่านทั้ง Python และ firmware (native env)
[ ] startup invariant ทั้ง 9 ข้อ fail จริงเมื่อป้อนค่าที่ขัดกัน (test ละข้อ)
[ ] get_drive_state() คืน key ชุดเดียวกันทั้ง fake / isaac / esp32
[ ] ภาพดินเปล่า → exg mask ว่าง (ไม่ให้ Otsu แบ่ง noise)

V1  Isaac
[ ] rover วิ่งร่องตรง 2000 mm จบ ไม่เบียดต้นพืช
[ ] ร่องโค้ง 40 mm วิ่งจบ
[ ] เริ่มเอียง 15° เข้าร่องได้
[ ] ผิวดิน ±15 mm ยังตามร่องได้ (ล้อลอยเป็นช่วง)
[ ] ช่องว่างกลางแถว → ไม่หยุด
[ ] สุดร่อง → STOPPED(row_end_suspected)
[ ] ถอดแถวแต่ยังมีเขียว → ERROR(row_lost)
[ ] front loop >= 10 Hz
[ ] วัด max_lateral_error จริง บันทึกลง perception.yaml
[ ] วัด crop_foliage_half_width จริงจาก asset พืช บันทึกลง rover.yaml
[ ] re-validate invariant ข้อ 5 ด้วยค่าที่วัดได้ทั้งสองตัว

V2  Perception
[ ] weed detector per-frame precision >= 0.9
[ ] weed detector per-frame recall >= 0.8  (วัชพืชในร่องเท่านั้น)
[ ] วัดกับ randomization เปิดครบ ไม่ใช่ seed เดียว

V3  HIL — ล้อลอย
[ ] ถอดสาย serial → มอเตอร์ดับภายใน 300 ms (วัดด้วย scope หรือ log timestamp)
[ ] กด E-stop → ล้อหยุด ขณะที่ Pi ยังส่ง drive อยู่
[ ] ยัด drive 50 ตัวเข้า buffer → ESP32 ใช้ตัว seq สูงสุดตัวเดียว
[ ] ส่ง seq ย้อนหลัง → ถูกทิ้ง
[ ] วัด wheel_v_min_mm_s จริง แล้ว re-validate invariant drive ตัวล่าง

V4  แปลงจริง
[ ] 2 กล้องพร้อมกันบน Pi โดย front ยัง >= 10 Hz
[ ] tune gain esp32 แล้วบันทึกลง control.yaml (ไม่ใช้ค่า isaac)
[ ] วัด runaway distance จริง <= runaway_budget_mm
[ ] วัด homography error จริง บันทึกลง camera.yaml
[ ] วิ่งจบร่อง 2000 mm ในแปลงจริง 3 ครั้งติด
```

เกณฑ์ทุกข้อ **วัดได้** — ไม่มีข้อไหนเขียนว่า "ทำงานได้" โดยไม่บอกว่าวัดอย่างไร

### 10.5 Scenario tests

| Scenario | ทดสอบ |
|---|---|
| `row_straight.yaml` | baseline — ตามร่องตรงได้ |
| `row_curved.yaml` | ตาม `row_curvature_mm` ได้โดยไม่เบียดต้นพืช |
| `row_tilted_start.yaml` | เริ่มเอียง 15° ต้องเข้าร่องได้ |
| `crop_gap_midrow.yaml` | ช่องว่างกลางแถว → **ต้องไม่หยุด** |
| `row_end.yaml` | สุดร่อง → `STOPPED(row_end_suspected)` |
| `row_lost.yaml` | ถอดแถวกลางทางแต่ยังมีเขียว → `ERROR(row_lost)` |
| `soil_rough.yaml` | ±15 mm ล้อลอย ต้องยังตามร่องได้ |
| `link_lost.yaml` | Pi ↔ ESP32 หลุด → มอเตอร์ดับใน `command_timeout_ms` |
| `estop_during_drive.yaml` | กด E-stop ระหว่างวิ่ง |
| `weed_in_furrow.yaml` | detect วัชพืชในร่องได้ |

`crop_gap_midrow.yaml` กับ `row_end.yaml` เป็นคู่ตรงข้ามที่ต้องผ่านทั้งคู่ —
ผ่านข้างเดียวแปลว่า threshold ตั้งเอาใจข้างเดียว

**ตัดออก**: `weed_near_x_min` · `weed_near_x_max` · `endstop_failure` · `motor_timeout`
· `soil_high_spot` · `soil_low_spot` · `weed_inside_crop_row` · `crop_row_misaligned`
· `normal_bed_10_weeds` · `100_random_weeds` · `camera_delay`

(`soil_high_spot` / `soil_low_spot` / `weed_inside_crop_row` กลับมาที่ M3 พร้อม tool)

### 10.6 รั้วขอบเขต — สิ่งที่ MVP ไม่ทำ

```text
ไม่กำจัดวัชพืช — ไม่มี tool ไม่มี servo
ไม่เลี้ยวเข้าร่องถัดไป — วิ่งร่องเดียวแล้วหยุด
ไม่มีพิกัดวัชพืชในแปลง — ไม่มี odometry
ไม่ dedupe การตรวจจับ — log นับ detection ไม่ใช่นับต้น
ไม่เห็นวัชพืชในแถว — เห็นแค่ในร่อง
ไม่มี obstacle avoidance — ไม่มี bumper ไม่มี ToF
ไม่มี turn-in-place — v = 0 กับ omega ใด ๆ อยู่ใน deadband ทั้งสองล้อ
ไม่มี suspension
ไม่มี ROS2 · ไม่มี MQTT · ไม่มี Wi-Fi ใน control path
```

### 10.7 Milestone ถัดไป — ลำดับที่ไม่ควรสลับ

| M | เนื้อหา | ทำไมต้องลำดับนี้ |
|---|---|---|
| **M2** | wheel encoder → ระยะตามแถว, dedupe, weed map | ต้องมาก่อน tool: tool ต้องเล็ง และการเล็งต้องรู้ตำแหน่ง |
| **M3** | tool + soil contact + in-row detection | ยกงานจาก `2026-09-12-weeding-bed-design.md` กลับมาใช้ได้เกือบทั้งหมด — tool reach invariant, soil contact, `tool_no_contact`/`tool_overreach` ยังถูกต้องทั้งชุด |
| **M4** | เลี้ยวเข้าร่องถัดไป → cover ทั้งแปลง | ต้องมี odometry จาก M2 ถึงจะเลี้ยวแบบไม่เห็นเป้าหมายได้ |
| **M5** | AI detector แทน ExG | dataset จาก M1–M3 มี background ดินจริงแล้ว |

---

## 11. Hardware, CAD, Coordinate Frames

### 11.1 `omega_max` ต้องเป็น 40 ไม่ใช่ 60 — deadband ของมอเตอร์ DC

```text
ล้อ 65 mm → เส้นรอบวง 204 mm
v = 100 mm/s → 0.49 rev/s → 29 RPM ที่ล้อ
มอเตอร์ 12 V ~100 RPM → ความเร็วล้อสูงสุด 340 mm/s

ที่ omega_max = 60 deg/s = 1.047 rad/s:
  differential = 1.047 × 60 = 62.8 mm/s
  v_right = 162.8 mm/s   (48% duty)   ok
  v_left  =  37.2 mm/s   (11% duty)   ไม่ผ่าน — อยู่ใน deadband

ที่ omega_max = 40 deg/s = 0.698 rad/s:
  differential = 0.698 × 60 = 41.9 mm/s
  v_right = 141.9 mm/s   (42%)   ok
  v_left  =  58.1 mm/s   (17%)   ok เหนือ deadband 7.1 mm/s

เพดานจริงของ omega_max คือ 46 deg/s (ที่ 46.8 ค่า v_left จะแตะ 51 พอดี)
เลือก 40 เพื่อให้มี margin ไม่ใช่เพราะ 40 เป็นค่าสูงสุดที่ทำได้
```

มอเตอร์เกียร์ DC ไม่ออกตัวใต้ ~15% duty (≈ 51 mm/s ที่ล้อนี้)
**ล้อข้างในจะหยุดหมุนขณะที่ ESP32 คิดว่ากำลังสั่งให้หมุน** — rover จะเลี้ยวแรงกว่าที่สั่ง
โดยไม่มีสัญญาณบอก นี่คือเหตุผลของ invariant `drive` สองข้อใน §9.5

### 11.2 BOM — POC v2 (rover)

```text
ตัดออก                              เพิ่ม
TMC2209 × 2                        Raspberry Pi 4 (4 GB) + microSD
NEMA17 × 2                         DC gear motor 12 V ~100 RPM × 4
MG996R servo                       Dual H-bridge >= 5 A/ch × 1  (BTS7960 / IBT-2)
Limit switch × 4                   ล้อยาง Ø65 mm × 4
Microswitch (soil contact)         แชสซี + แผ่นยึด
                                   กล้อง × 2  (Pi Camera / USB)
คงเดิม                              แบตเตอรี่ 3S LiPo 2200 mAh
ESP32 DevKit (WROOM-32)            DC-DC buck 5 V/5 A × 2
E-stop latching NC                 ตัวเก็บประจุ 1000 µF (motor rail)
```

**เลือกมอเตอร์ที่ ~100 RPM ไม่ใช่เร็วกว่า** — มอเตอร์ 300 RPM จะทำให้ความเร็ว
เป้าหมาย 100 mm/s ตกไปอยู่ที่ 10% duty ซึ่งอยู่ใน deadband ตลอดเวลา
ความเร็วมอเตอร์ควรอยู่ที่ **2–3 เท่า** ของความเร็วใช้งาน ไม่ใช่มากที่สุดที่หาได้

ข้อกำหนดกล้องจาก BOM เดิม (resolution >= 1280×720, fixed focus, manual exposure) ยังใช้
แต่ข้อกำหนด FOV >= 60° ที่ 520 mm **ไม่ใช้แล้ว** — กล้องอยู่บน rover ไม่ได้ครอบทั้งแปลง
กล้องล่างต้องครอบความกว้าง corridor + ขอบ ≈ 240 mm ที่ความสูงติดตั้งจริง (คำนวณจาก CAD)

### 11.3 Wiring — ต่อล้อขนานเป็นข้าง

skid-steer สั่งล้อหน้าและหลังข้างเดียวกัน**เหมือนกันเสมอ** จึงต่อขนานได้:

```text
ช่อง A  ──┬── motor_fl        ช่อง B  ──┬── motor_fr
          └── motor_rl                  └── motor_rr
```

เหลือ driver 2 ช่องพอสำหรับ 4 มอเตอร์ — แลกกับที่คุมหน้า/หลังแยกไม่ได้ (ไม่ต้องใช้)
และวัดกระแสต่อล้อไม่ได้ (ไม่ต้องใช้) แต่ต้องเลือก driver ที่ทน
**กระแสสองมอเตอร์ต่อช่อง** ไม่ใช่หนึ่ง

### 11.4 E-stop ต้องตัด motor rail ไม่ใช่ตัด logic

```text
แบต 3S ──┬── E-stop (latching NC) ── driver ── มอเตอร์
         │
         └── buck 5V ── Pi + ESP32        ◄── ไม่ผ่าน E-stop
```

ตัดเฉพาะไฟมอเตอร์ ทำให้ Pi และ ESP32 ยังมีไฟ → **ยังรายงานได้ว่า `estop: true`**
ถ้าตัดทั้งระบบ จะได้ rover ที่หยุดจริงแต่เงียบ แยกไม่ออกจาก rover ที่แบตหมดหรือแครช

ESP32 อ่านสถานะจาก sense line หลัง E-stop ผ่านตัวแบ่งแรงดัน

**Power — ความเสี่ยงที่คร่าชีวิตโปรเจกต์หุ่นยนต์มากกว่า bug**

Inrush ตอนมอเตอร์ออกตัวดึงแรงดันตก → Pi รีบูตกลางทาง มาตรการที่ต้องมีตั้งแต่แรก:

1. **buck แยกสำหรับ Pi** ไม่แชร์กับ driver
2. **ตัวเก็บประจุ 1000 µF** ที่ขาไฟเข้า driver
3. ESP32 กินไฟจาก buck ไม่ใช่จาก motor rail
4. ground แบบ star ที่แบต ไม่ต่อพ่วงกันเป็นลูกโซ่

### 11.5 CAD

```text
cad/fusion/rover.f3d           แชสซี + ยึดมอเตอร์ 4 จุด + ยึดกล้อง 2 จุด + ถาดอิเล็กทรอนิกส์
cad/urdf/weeding_rover.urdf    base_link + 4 continuous wheel joint + 2 camera link
cad/parameters/parameters.csv  track_width · wheelbase · wheel_diameter
                               chassis_clearance · body_width
                               camera_front_height_mm · camera_front_tilt_deg
                               camera_down_height_mm
```

**`camera_front_tilt_deg` เป็นพารามิเตอร์ที่ผูกกับ gain ของ controller** —
มุมเอียงกำหนดว่า `valley_far` อยู่ห่างไปข้างหน้าเท่าไร ซึ่งคือ lookahead distance
ถ้าขยับขายึดกล้อง gain ที่ tune ไว้ใช้ไม่ได้ ต้องเขียนคำเตือนนี้ทั้งใน
`cad/parameters/README.md` และ `config/control.yaml`

เป็น coupling ระหว่าง CAD กับ control ที่ gantry ไม่มี (กล้อง gantry มองตรงลง
มุมไม่มีผลต่อ control)

### 11.6 Coordinate frames — โซ่ที่ขาดโดยเจตนา

```text
เดิม   world → bed → machine → tool
                       └─► camera          โซ่ต่อเนื่อง machine อยู่นิ่ง

ใหม่   world → bed  ┄┄ ไม่มี ┄┄  rover ──┬─► camera_front
                                          └─► camera_down
```

**`bed → rover` ไม่มีใครรู้ค่า runtime** เพราะไม่มี localization
ทุกอย่างใต้ `rover` ทำงานใน rover frame เท่านั้น

ต้องเขียนใน `coordinate-frames.md` ให้ชัดว่านี่คือ **ช่องว่างที่ประกาศไว้
ไม่ใช่ของที่ลืมเขียน** — ใครเขียน code ที่ต้องการพิกัดใน bed frame ต้องหยุด
แล้วไปทำ M2 (encoder) ไม่ใช่เดาค่ามาเติม

ตัดออก: `home_offset_mm` (ไม่มี homing ให้ offset) · `machine` frame
คงเดิม: ตาราง unit convention (config/protocol = mm + deg, URDF/USD = m + rad)
และจุดแปลงหน่วย 2 ที่ที่ต้องมี unit test

---

## 12. Files Touched

```text
เขียนใหม่ทั้งไฟล์
  README.md                              rover, MVP scope, รั้วขอบเขต, acceptance ใหม่
  docs/architecture.md                   Rover/CameraSet interface, state machine, safety 3 ชั้น
  docs/simulation.md                     rover USD, heightfield, physics tuning, generator
  docs/protocol.md                       streaming vs discrete, seq/last_seq
  protocol/messages.md                   message ใหม่ทั้งชุด, error codes
  config/README.md                       ไฟล์ config 7 ไฟล์, invariant 9 ข้อ
  controller/README.md                   row_follower, startup_checks, safety
  perception/README.md                   exg, row_estimator (valley), weed_detector, weed_log
  hardware/electrical/wiring.md          ล้อขนาน, E-stop ตัด motor rail, sense line
  hardware/electrical/power.md           buck แยก, bulk cap, star ground
  hardware/mechanical/dimensions.md      rover geometry แทนราง
  hardware/mechanical/assembly.md        ประกอบ rover
  hardware/mechanical/coordinate-frames.md   โซ่ขาดที่ bed → rover
  docs/hardware.md                       firmware scope ใหม่ (mixing, timeout, ไม่มี stepper/servo)
  docs/calibration.md                    กล้องล่างเท่านั้น, error budget ±20 mm
  cad/README.md · cad/parameters/README.md · cad/urdf/README.md
  sim/isaac/README.md                    rover/, scene, sensor
  tests/README.md                        scenario ชุดใหม่
  scripts/README.md                      import_urdf, run_fake_loop
  bridge/README.md                       serial เท่านั้น, ตัด MQTT

ย้าย / เปลี่ยนชื่อ
  sim/isaac/robots/gantry/  →  sim/isaac/robots/rover/
  hardware/bom/poc-v1.md    →  hardware/bom/poc-v2.md

ใหม่
  config/drive_mixing_vectors.csv

เก็บเป็นประวัติ — ใส่หมายเหตุ superseded ที่หัวไฟล์ ไม่แก้เนื้อหา
  docs/superpowers/specs/2026-09-12-weeding-bed-design.md

ลบ
  mini-smart-weeding-table-repository-architecture.md    (design โต๊ะ + ถาด)
```

เอกสาร design โต๊ะ + ถาด **ถูกลบออกจาก working tree** ไม่ได้เก็บไว้เป็นไฟล์ —
มันถูก superseded สองรอบแล้ว (โต๊ะ → แปลงดิน → rover) และไม่มีส่วนไหน
ที่ milestone ข้างหน้าจะหยิบกลับมาใช้ เนื้อหายังอ่านได้จาก git history

**เก็บ 2 ไฟล์ประวัติไว้ไม่ลบ** — ส่วน tool reach invariant, soil contact sensor,
`tool_no_contact`/`tool_overreach` ใน spec แปลงดินยังถูกต้องทั้งชุดและ M3 จะหยิบกลับมาใช้
header ของไฟล์ต้องบอกว่าส่วนไหนยังใช้ได้ ส่วนไหนตายแล้ว —
เพื่อให้คนอ่านปีหน้ารู้ว่าอะไรเลื่อน อะไรทิ้ง

---

## 13. Known Limitations

1. **ไม่มีพิกัดวัชพืช** — ไม่มี odometry `weed_log` นับ detection ไม่ใช่นับต้น
   ตัวเลขรวมจาก log ไม่มีความหมาย แก้ที่ M2 ด้วย encoder
2. **วัชพืชในแถวมองไม่เห็น** — กล้องล่างเล็งที่ร่อง กรณีที่ยากที่สุดของปัญหาจริงอยู่นอก MVP
3. **ไม่มีการป้องกันทางกายภาพ** — ไม่มี bumper ไม่มี ToF พึ่ง `runaway_budget_mm`
   + ขอบแปลงยก + ความเร็วต่ำ ถ้าขอบแปลงไม่ยก MVP นี้ไม่ปลอดภัยพอจะรันโดยไม่มีคนดู
4. **`row_end` vs `row_lost` เป็น heuristic** — แยกด้วย `green_fraction` ตัวเดียว
   ต้องมี scenario test คู่ตรงข้ามยืนยัน และอาจต้อง tune threshold ต่อแปลง
5. **gain ไม่โอนจาก sim ไป hardware** — slip ของ skid-steer ต่างกัน ต้อง tune ใหม่ที่ V4
   และ gain ผูกกับ `camera_front_tilt_deg` ด้วย
6. **`wheel_v_min_mm_s` ยังเป็นค่าประมาณ** — invariant `drive` ตัวล่างมี margin 7.1 mm/s
   ถ้าวัดจริงที่ V3 แล้วเกิน 58 mm/s ต้องลด `omega_max` ลงอีก (เพดานคือ 46 deg/s)
   **ห้ามลด `wheel_v_min` เพื่อให้ผ่าน** — ค่านั้นเป็นคุณสมบัติของมอเตอร์
7. **ดินไม่ยุบ** (ยกมาจาก design เดิม) — ล้อบนดินจริงจะจมและมีแรงต้านที่ sim ไม่เห็น
   พิสูจน์ที่ V4
8. **ไม่มี suspension** — ล้อลอยเป็นช่วงบนผิว ±15 mm แรงขับหายเป็นจังหวะ
   ถ้า V1 แสดงว่าตามร่องไม่ได้ ต้องกลับมาทบทวนล้อ/ช่วงล่าง ไม่ใช่ทบทวน gain
9. **`crop_foliage_half_width_mm` ยังเป็นค่าประมาณ** — เป็นตัวกำหนดทั้ง clear furrow
   (invariant ข้อ 5) และ corridor ของ weed detector พร้อมกัน ค่านี้ผิดจะทำให้
   ทั้งความปลอดภัยและ recall ผิดไปด้วยกัน ต้องวัดจาก asset พืชที่ V1 ก่อนเชื่อตัวเลขใด ๆ
   ที่สืบทอดจากมัน และใบพืชโตขึ้นตามเวลา — แปลงจริงที่ V4 อาจต้องระยะแถวมากกว่า sim
