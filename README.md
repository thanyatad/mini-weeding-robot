# Mini Smart Weeding Robot

POC หุ่นยนต์เกษตรแบบ **rover 4 ล้อ วิ่งระหว่างแถวปลูก** —
NVIDIA Isaac Sim + Python + Computer Vision + Raspberry Pi + ESP32

เป้าหมาย: เริ่มพัฒนาจาก Simulation ก่อน แล้วเปลี่ยนไปใช้ Hardware จริงภายหลัง
โดยไม่ต้องรื้อ architecture หลัก

---

## First MVP — วิ่ง + มองเห็น

MVP แรกทำสองอย่างเท่านั้น: **เดินตามร่องด้วยกล้อง** และ **ตรวจจับวัชพืชในร่อง**

```text
✓ เดินตามร่องระหว่างแถวปลูกด้วยกล้องหน้า
✓ ตรวจจับวัชพืชในร่องด้วยกล้องล่าง แล้วบันทึก log
✗ ไม่กำจัดวัชพืช — ไม่มี tool ไม่มี servo
```

การกำจัดวัชพืชอยู่ที่ **M3** พร้อมงาน tool ทั้งชุด — ดู [Roadmap](#roadmap)

เหตุผล: mobility + perception เป็นความเสี่ยงที่ยังไม่ถูกพิสูจน์
แก้ทีเดียวพร้อม actuator จะแยกไม่ออกว่าอะไรพัง

Design เต็ม: **[docs/superpowers/specs/2026-09-12-rover-mvp-design.md](docs/superpowers/specs/2026-09-12-rover-mvp-design.md)**

---

## Core Principle

> Controller ต้องไม่รู้ว่ากำลังควบคุม Simulation หรือ Hardware จริง

```text
Simulation != Controller
Controller != Hardware
Hardware   != Perception
Perception != Simulation
```

ทุก layer เชื่อมกันผ่าน **Interface** เท่านั้น

หลักการนี้รอดจากการเปลี่ยน form factor ทั้งเครื่อง — ตอนเลิก gantry มาใช้ rover
`Rover`/`CameraSet` interface ถูกเขียนใหม่ แต่ **pattern การเลือก backend
และเส้นแบ่งชั้นไม่ต้องแตะเลย**

---

## High-Level Architecture

```text
     ┌──────────────────────┐      ┌──────────────────────┐
     │   camera_front       │      │    camera_down       │
     │   row following      │      │   weed detection     │
     └──────────┬───────────┘      └──────────┬───────────┘
                ▼                             ▼
     ┌──────────────────────┐      ┌──────────────────────┐
     │    RowEstimate       │      │     Detection[]      │
     └──────────┬───────────┘      └──────────┬───────────┘
                ▼                             ▼
     ┌──────────────────────┐             weed_log
     │      Controller      │        (observation path)
     │  RowFollower         │
     │  State Machine       │
     │  Safety              │
     └──────────┬───────────┘
                ▼
     ┌──────────────────────┐
     │   Rover Interface    │   drive(v, omega) · stop() · emergency_stop()
     └──────────┬───────────┘
    ┌───────────┴───────────┐
    ▼                       ▼
┌─────────────┐     ┌─────────────┐
│ IsaacRover  │     │ Esp32Rover  │
└──────┬──────┘     └──────┬──────┘
       ▼                   ▼
 Isaac Sim            ESP32 (serial)
                           │
                  ┌────────┼────────┐
                  ▼        ▼        ▼
             H-bridge  E-stop   Watchdog
                  │     sense
                  ▼
         DC gear motor × 4  (skid-steer)
```

---

## The Machine

Rover skid-steer 4 ล้อ วิ่งในร่องระหว่างแถวปลูก

```text
Bed                       2000 × 1000 mm
Crop rows                 3 แถว ตามยาว
row_spacing               350 mm      กึ่งกลางแถวถึงกึ่งกลางแถว
clear furrow              290 mm      ◄── ค่าที่ rover ใช้จริง
Soil                      heightfield ผิวไม่เรียบ ±15 mm ไม่ยุบตัว
Rover                     skid-steer 4WD · กว้างรวมล้อ 146 mm · ล้อ Ø65 mm
Cameras                   front (nav, ไม่ calibrate) + down (detect, homography)
Compute                   Raspberry Pi ← serial → ESP32
```

**`row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้** — ใบพืชยื่นเข้ามาข้างละ ~30 mm
ค่าที่ใช้จริงคือ `clear_furrow = row_spacing − 2 × crop_foliage_half_width`
invariant ทุกข้อที่เกี่ยวกับช่องว่างด้านข้างใช้ค่านี้

---

## ไม่มี Odometry — และผลที่ตามมา

MVP ไม่มี encoder ไม่มี localization ไม่มีแผนที่ ผลที่ต้องรู้ก่อนเขียน code:

```text
ไม่มี get_position()         ไม่มีอะไรตอบได้
ไม่มี drive_distance(mm)     จะ implement ได้แค่ velocity × time ซึ่งอ้างความแม่นที่ไม่มีจริง
ไม่มีพิกัดวัชพืชในแปลง        weed_log นับ "การตรวจจับ" ไม่ใช่ "จำนวนต้น"
โซ่ frame ขาดที่ bed → rover  ทุกอย่างใต้ rover ทำงานใน rover frame เท่านั้น
```

ใครเขียน code ที่ต้องการพิกัดใน `bed` frame ต้องหยุดแล้วไปทำ **M2 (encoder)**
ไม่ใช่เดาค่ามาเติม — ดู [hardware/mechanical/coordinate-frames.md](hardware/mechanical/coordinate-frames.md)

---

## Safety — 3 ชั้น 3 เจ้าของ

| กลไก | เจ้าของ | เงื่อนไข trip |
|---|---|---|
| Command timeout | **ESP32 firmware** | ไม่ได้รับ velocity command เกิน `command_timeout_ms` |
| Row-loss watchdog | **controller** | estimate invalid ติดกัน `row_loss_frames` |
| Latching E-stop | **hardware** | คนกด — ตัด motor rail ทางไฟ |

**แต่ละชั้นทำงานได้โดยไม่ต้องพึ่งชั้นบน** — Pi แครช มอเตอร์ก็ดับ ·
firmware แฮงค์ ก็ยังตัดไฟได้ ถ้าชั้นใดพึ่งชั้นบน มันไม่ใช่ safety layer มันคือ feature

MVP **ไม่มี bumper switch** ตัวชดเชยคือ `runaway_budget_mm` ที่ validate ตอน startup
+ ขอบแปลงยกสูงกว่าล้อ + ความเร็วต่ำ

---

## Startup Invariants

ความปลอดภัยที่พึ่งตัวเลขใน config ต้องถูก validate ตอน startup ไม่ใช่หวังว่าจะตั้งถูก

```text
safety      v_max × command_timeout/1000        <= runaway_budget        30 <= 60
safety      v_max × row_loss_frames/loop_hz     <= runaway_budget        30 <= 60
geometry    wheel_diameter        >= 4 × soil_variation                  65 >= 60
geometry    chassis_clearance     >  soil_variation                      35 >  15
geometry    clear_furrow          >  body_width + 2 × runaway_budget    290 > 266
drive       v + omega_max_rad × track/2  <= wheel_v_max               141.9 <= 340
drive       v − omega_max_rad × track/2  >= wheel_v_min                58.1 >=  51
config      gains[backend] ไม่เป็น null
consistency simulation.soil.variation_mm == bed.soil_variation_mm
```

ทุกข้อ fail = **ไม่ start** + บอกค่าที่ขัดกันเป็นตัวเลข ไม่ใช่ warning

ตัวเลขสี่ชุดในโปรเจกต์นี้ถูกแก้เพราะ invariant จับได้ตอนร่างเอกสาร ไม่ใช่ตอน debug:
`row_spacing` (250 → 350) · `omega_max` (60 → 40 เพราะ deadband มอเตอร์) ·
นิยามข้อ 5 (`row_spacing` → `clear_furrow` เพราะใบพืชกินร่อง) ·
`track_width` (140 → 120 เพราะ `body_width` ต้องหมายถึงความกว้าง**รวมล้อ** ไม่ใช่แค่แชสซี)

ดู [config/README.md](config/README.md#startup-invariants) · [controller/README.md](controller/README.md#startup-validation)

---

## Repository Layout

| Path | Responsibility | Docs |
|---|---|---|
| `cad/` | Fusion source, parameters, exports, URDF — **ต้นทางของขนาดทุกตัว** | [cad/README.md](cad/README.md) |
| `hardware/` | เครื่องจริง — BOM, ขนาด, ประกอบ, สายไฟ, coordinate frames | [hardware/README.md](hardware/README.md) |
| `sim/isaac/` | Physics, แปลงดิน, sensor, rover model, simulation I/O | [docs/simulation.md](docs/simulation.md) |
| `controller/` | Robot brain — row follower, state machine, safety, startup checks | [controller/README.md](controller/README.md) |
| `perception/` | ExG, row estimator, weed detector, weed log, calibration | [perception/README.md](perception/README.md) |
| `firmware/esp32/` | Skid-steer mixing, command timeout, motor enable, E-stop sense | [docs/hardware.md](docs/hardware.md) |
| `bridge/` | Serial transport ระหว่าง Controller ↔ ESP32 / Simulation | [bridge/README.md](bridge/README.md) |
| `protocol/` | Message contract (JSON) shared by all layers | [docs/protocol.md](docs/protocol.md) |
| `config/` | YAML parameters — never hard-code in source | [config/README.md](config/README.md) |
| `tests/` | unit / integration / simulation / scenarios | [tests/README.md](tests/README.md) |
| `scripts/` | Run & flash entry points | [scripts/README.md](scripts/README.md) |
| `tools/` | Calibration, dataset, visualization utilities | [tools/README.md](tools/README.md) |

Full architecture rationale: **[docs/architecture.md](docs/architecture.md)**

---

## Asset Pipeline

Geometry ทั้งหมดเกิดที่ CAD แล้วไหลไป simulation — ไม่มีใครวาดของซ้ำสองที่

```text
Fusion 360  cad/fusion/rover.f3d           ◄── source of truth
   │
   ├── parameters ──> cad/parameters/parameters.csv ──┬─> config/rover.yaml
   │                                                   └─> hardware/mechanical/dimensions.md
   │
   └── exports/meshes ──> cad/urdf/weeding_rover.urdf
                                 │  Isaac URDF importer
                                 ▼
              sim/isaac/robots/rover/rover_base.usd    generated, gitignored
                                 │  + USD layer
                                 ▼
              sim/isaac/robots/rover/rover.usd         physics tuning, commit
```

**CAD เป็นเจ้าของ geometry · `config/` เป็นเจ้าของ operational limits**
ค่าที่ derived จาก CAD มีตารางกำกับใน [cad/parameters/README.md](cad/parameters/README.md)
และมี `tests/unit/test_cad_config_sync.py` จับ drift

เส้นแบ่ง: *วัดด้วยเวอร์เนียร์ได้ = CAD · เปลี่ยนได้โดยไม่ต้องถอดสกรู = config*

⚠️ ข้อยกเว้นที่ rover มีแต่ gantry ไม่มี: **`camera_front_tilt_deg` ผูกกับ gain ของ controller**
มุมก้มกำหนด lookahead distance ขยับขายึดกล้องแล้ว gain ที่ tune ไว้ใช้ไม่ได้

---

## Backend Selection

Implementation ถูกเลือก **ตอน startup เท่านั้น** — ไม่มี `if simulation:` กระจายใน codebase

```yaml
# config/development.yaml
backend: isaac   # isaac | esp32 | fake
```

| Backend | Use case |
|---|---|
| `fake` | Unit test, closed-loop row following โดยไม่เปิด Isaac |
| `isaac` | Software-in-the-Loop, simulation test, digital twin |
| `esp32` | Hardware-in-the-Loop และ Real machine |

---

## Development Phases

```text
V0  Fake Rover        → Rover interface, RowFollower, state machine, safety, protocol
                        + closed loop ด้วย FakeRover (ไม่ต้องมีกราฟิก)
V1  Isaac Simulation  → rover USD, physics tuning (งานหนักสุด), row estimator, scenarios
V2  Computer Vision   → weed detector กล้องล่าง + evaluation harness เทียบ ground truth
V3  ESP32 HIL         → firmware จริงบนโต๊ะ ล้อลอย — mixing, timeout, E-stop, deadband
V4  Real Hardware     → rover จริงในแปลงจริง — tune gain ใหม่, วัด runaway budget
```

**น้ำหนักงานย้ายจาก design เดิม**: gantry หนักที่ V2 (แยก crop/weed)
rover หนักที่ **V1** (physics ของล้อบน heightfield) — วางแผนเวลาตามนี้

### V0 ปิดลูปได้โดยไม่ต้องมีกราฟิก

```text
FakeRover        integrate (v, omega) → pose ภายใน    (kinematic only)
FakeRowSensor    pose + สมการเส้นร่อง → RowEstimate
RowFollower      RowEstimate → (v, omega)
                        └──> วนกลับเข้า FakeRover
```

ตอบคำถาม *"gain ชุดนี้ลู่เข้าไหม และแกว่งเกินร่องไหม"* ได้ในหลักมิลลิวินาที อยู่ใน CI ได้
ก่อนแตะ Isaac เลย

⚠️ `FakeRover` มี pose ภายใน แต่ **ห้าม expose ผ่าน `get_drive_state()`** —
ถ้าหลุดออกมา code ที่เขียนตอน V0 จะพึ่งพา pose แล้วพังทั้งหมดตอนเปลี่ยนเป็น `esp32`

### MVP Acceptance Criteria

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
[ ] วัด max_lateral_error + crop_foliage_half_width จริง แล้ว re-validate invariant ข้อ 5

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

---

## รั้วขอบเขต — สิ่งที่ MVP ไม่ทำ

เขียนไว้ชัดเพื่อไม่ให้ไหลเข้ามาทีละข้อ

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

## Roadmap

| M | เนื้อหา | ทำไมต้องลำดับนี้ |
|---|---|---|
| **M1** | **rover MVP — วิ่ง + มองเห็น** (ที่กำลังทำ) | พิสูจน์ mobility + perception ก่อนเพิ่ม actuator |
| **M2** | wheel encoder → ระยะตามแถว, dedupe, weed map | ต้องมาก่อน tool: tool ต้องเล็ง และการเล็งต้องรู้ตำแหน่ง |
| **M3** | tool + soil contact + in-row detection | ยกงาน tool จาก spec แปลงดินกลับมาใช้ได้เกือบทั้งหมด |
| **M4** | เลี้ยวเข้าร่องถัดไป → cover ทั้งแปลง | ต้องมี odometry จาก M2 ถึงจะเลี้ยวแบบไม่เห็นเป้าหมายได้ |
| **M5** | AI detector แทน ExG | dataset จาก M1–M3 มี background ดินจริงแล้ว |

---

## Technology Stack

| Component | Technology |
|---|---|
| Simulation | NVIDIA Isaac Sim |
| Controller | Python (บน Raspberry Pi) |
| Computer Vision | OpenCV / PyTorch |
| Firmware | ESP-IDF / Arduino |
| Firmware Build | PlatformIO |
| Communication | Serial (JSON lines) |
| Config | YAML |
| Protocol | JSON |
| Testing | pytest |
| Scenario Testing | YAML |
| Robot Model | USD |
| Version Control | Git |

---

## Why Not ROS2 (yet)

MVP นี้มี **1 หุ่นยนต์ 2 กล้อง 4 มอเตอร์ 1 E-stop** — ไม่มี SLAM ไม่มี path planning
ไม่มี MoveIt row following เป็น reactive controller ตัวเดียวที่รับ `RowEstimate`
คืน `(v, omega)` ไม่มี node graph ให้จัดการ

Python + Isaac Sim API + Rover abstraction ง่ายกว่าและเพียงพอ

ควรเพิ่ม ROS2 เมื่อมี: **Nav2 / SLAM จริง**, robot หลายตัว, sensor หลายชุด,
distributed process, hardware driver หลายชนิด, MoveIt / ROS Control

M4 (เลี้ยวเข้าร่องถัดไป + cover ทั้งแปลง) เป็นจุดที่ควรกลับมาทบทวนข้อนี้อีกครั้ง

---

## Design History

| Document | Status |
|---|---|
| [docs/superpowers/specs/2026-09-12-rover-mvp-design.md](docs/superpowers/specs/2026-09-12-rover-mvp-design.md) | **Current** — rover 4 ล้อ, row following, MVP วิ่ง + มองเห็น |
| [docs/superpowers/specs/2026-09-12-weeding-bed-design.md](docs/superpowers/specs/2026-09-12-weeding-bed-design.md) | Superseded — gantry คร่อมแปลง · **ส่วน tool ยังใช้ได้ที่ M3** |
| `mini-smart-weeding-table-repository-architecture.md` | ลบแล้ว — design โต๊ะ + ถาด อยู่ใน git history เท่านั้น |
