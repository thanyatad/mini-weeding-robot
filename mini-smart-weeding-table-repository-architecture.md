> **⚠️ SUPERSEDED — 2026-09-12**
>
> เอกสารนี้เก็บไว้เป็นบันทึกการตัดสินใจเดิม **เนื้อหาไม่ถูกแก้**
>
> Design ปัจจุบันจำลองบน **แปลงปลูกขนาดเล็ก** ไม่ใช่โต๊ะ + ถาด
> ดู [docs/superpowers/specs/2026-09-12-weeding-bed-design.md](docs/superpowers/specs/2026-09-12-weeding-bed-design.md)
>
> Section ที่ถูกแทนที่: 7 (Scene) · 8 (Gantry) · 9 (Sensor Mapping) ·
> 25 (Simulation Config) · 27 (Environment Generator) · 33 (Scenario Tests) ·
> 48 (Second Milestone)
>
> **อัปเดต 2026-09-12 (ครั้งที่ 2):** design ปัจจุบันเป็น **rover skid-steer 4 ล้อ**
> ไม่ใช่ gantry แล้ว — `Machine` interface ในเอกสารนี้ถูกแทนด้วย `Rover`
> ทั้งชุด (ไม่มี `home` / `move_xy` / `get_position` / `get_endstops` / `tool_*`)
> ดู [docs/superpowers/specs/2026-09-12-rover-mvp-design.md](docs/superpowers/specs/2026-09-12-rover-mvp-design.md)
>
> Architecture หลัก — machine abstraction, layer boundary, state machine,
> protocol, development phases — **ยังใช้ได้ทั้งหมด**

---

# Mini Smart Weeding Table
## Repository Architecture Design

> สำหรับ POC ระบบกำจัดวัชพืชขนาดเล็กด้วย NVIDIA Isaac Sim + Python + Computer Vision + ESP32

---

## 1. Objective

ออกแบบ repository สำหรับพัฒนา **Mini Smart Weeding Table** โดยให้สามารถเริ่มจาก Simulation ก่อน แล้วค่อยเปลี่ยนไปใช้ Hardware จริงภายหลังได้โดยไม่ต้องรื้อ architecture หลัก

เป้าหมายสำคัญคือ:

- ใช้ NVIDIA Isaac Sim จำลองโต๊ะ, Gantry, Camera, Sensor และ Physics
- ใช้ Python เป็น Robot Controller
- รองรับ Computer Vision สำหรับตรวจจับวัชพืช
- รองรับ ESP32 + TMC2209 + NEMA17 + Servo ใน Hardware จริง
- ใช้ logic ชุดเดียวกันระหว่าง Simulation และ Hardware
- รองรับ Software-in-the-Loop (SIL)
- รองรับ Hardware-in-the-Loop (HIL)
- รองรับ Automated Simulation Test
- แยก module ชัดเจนเพื่อให้ระบบขยายได้ในอนาคต

---

# 2. High-Level Architecture

```text
                 ┌──────────────────────┐
                 │      Perception      │
                 │                      │
                 │ Camera / AI / CV     │
                 └──────────┬───────────┘
                            │
                            ▼
                 ┌──────────────────────┐
                 │      Controller      │
                 │                      │
                 │ State Machine        │
                 │ Motion Planner       │
                 │ Safety               │
                 └──────────┬───────────┘
                            │
                            ▼
                 ┌──────────────────────┐
                 │  Machine Interface   │
                 └──────────┬───────────┘
                            │
             ┌──────────────┴──────────────┐
             │                             │
             ▼                             ▼
    ┌──────────────────┐          ┌──────────────────┐
    │   IsaacMachine   │          │   Esp32Machine   │
    └────────┬─────────┘          └────────┬─────────┘
             │                             │
             ▼                             ▼
      NVIDIA Isaac Sim                  ESP32
                                         │
                               ┌─────────┼─────────┐
                               ▼         ▼         ▼
                            TMC2209    Servo    Endstop
                               │
                               ▼
                            NEMA17
```

แนวคิดหลักคือ:

> Controller ต้องไม่รู้ว่ากำลังควบคุม Simulation หรือ Hardware จริง

---

# 3. Recommended Repository Structure

```text
mini-weeding-robot/
│
├── README.md
│
├── docs/
│   ├── architecture.md
│   ├── simulation.md
│   ├── hardware.md
│   ├── protocol.md
│   └── calibration.md
│
├── sim/
│   └── isaac/
│       ├── README.md
│       ├── app.py
│       ├── world.py
│       │
│       ├── scenes/
│       │   ├── weeding_table.usd
│       │   ├── tray.usd
│       │   └── plants/
│       │
│       ├── robots/
│       │   └── gantry/
│       │       ├── gantry.usd
│       │       ├── joints.py
│       │       └── config.yaml
│       │
│       ├── sensors/
│       │   ├── camera.py
│       │   ├── endstop.py
│       │   └── sensor_manager.py
│       │
│       ├── environments/
│       │   ├── random_plants.py
│       │   ├── random_weeds.py
│       │   └── domain_randomization.py
│       │
│       └── adapters/
│           ├── motor_adapter.py
│           └── io_adapter.py
│
├── controller/
│   ├── main.py
│   │
│   ├── machine/
│   │   ├── base.py
│   │   ├── isaac_machine.py
│   │   ├── esp32_machine.py
│   │   └── fake_machine.py
│   │
│   ├── motion/
│   │   ├── planner.py
│   │   ├── gantry.py
│   │   ├── homing.py
│   │   └── limits.py
│   │
│   ├── workflow/
│   │   ├── state_machine.py
│   │   └── weeding_cycle.py
│   │
│   └── safety/
│       ├── emergency_stop.py
│       ├── watchdog.py
│       └── fault_manager.py
│
├── perception/
│   ├── detector.py
│   ├── segmentation.py
│   ├── calibration.py
│   ├── coordinate_transform.py
│   ├── camera.py
│   └── models/
│
├── firmware/
│   └── esp32/
│       ├── platformio.ini
│       │
│       ├── src/
│       │   ├── main.cpp
│       │   ├── motion.cpp
│       │   ├── io.cpp
│       │   ├── servo.cpp
│       │   └── communication.cpp
│       │
│       └── include/
│           ├── motion.h
│           ├── io.h
│           └── protocol.h
│
├── bridge/
│   ├── main.py
│   ├── mqtt.py
│   ├── serial.py
│   ├── protocol.py
│   └── simulator.py
│
├── protocol/
│   ├── messages.md
│   ├── schema.json
│   └── examples/
│
├── config/
│   ├── machine.yaml
│   ├── simulation.yaml
│   ├── camera.yaml
│   └── development.yaml
│
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── simulation/
│   └── scenarios/
│
├── scripts/
│   ├── run_sim.sh
│   ├── run_controller.sh
│   ├── run_all.sh
│   └── flash_esp32.sh
│
└── tools/
    ├── calibration/
    ├── dataset/
    └── visualization/
```

---

# 4. Core Design Principle

## 4.1 Machine Abstraction

Controller ไม่ควรเรียก Isaac Sim หรือ ESP32 โดยตรง

ให้สร้าง interface กลาง:

```python
from typing import Protocol

class Machine(Protocol):

    def home(self) -> None:
        ...

    def move_xy(
        self,
        x_mm: float,
        y_mm: float,
        speed_mm_s: float | None = None,
    ) -> None:
        ...

    def tool_up(self) -> None:
        ...

    def tool_down(self) -> None:
        ...

    def get_position(self) -> tuple[float, float]:
        ...

    def get_endstops(self) -> dict:
        ...

    def emergency_stop(self) -> None:
        ...
```

จากนั้นมี implementation หลัก:

```text
Machine
│
├── FakeMachine
├── IsaacMachine
└── Esp32Machine
```

---

# 5. Why Machine Abstraction Matters

Controller ควรเขียนแบบนี้:

```python
machine.move_xy(320, 180)
machine.tool_down()
machine.tool_up()
```

ไม่ควรเขียน:

```python
if simulation:
    isaac.move_joint(...)
else:
    mqtt.publish(...)
```

เพราะจะทำให้ simulation logic และ hardware logic กระจายไปทั่ว project

ควรเลือก implementation ตอน startup เท่านั้น:

```python
if config.backend == "isaac":
    machine = IsaacMachine()

elif config.backend == "esp32":
    machine = Esp32Machine()

else:
    machine = FakeMachine()
```

---

# 6. Isaac Sim Module

โฟลเดอร์:

```text
sim/isaac/
```

ควรรับผิดชอบเฉพาะ:

- Physics
- Scene
- Sensor
- Robot Model
- Collision
- Joint
- Environment
- Simulation I/O

ไม่ควรรับผิดชอบ:

- Weed detection workflow
- Business logic
- State machine
- Motion planning ระดับ application
- Safety policy หลัก

---

# 7. Isaac Sim Scene

ตัวอย่าง Scene:

```text
World
│
├── Table
│
├── Tray
│   ├── Plant_001
│   ├── Plant_002
│   ├── Weed_001
│   └── Weed_002
│
├── Gantry
│   ├── X Axis
│   ├── Y Axis
│   └── Tool Z/Servo
│
└── Camera
```

---

# 8. Gantry Model

Gantry ใช้ Joint หลัก:

```text
Base
 │
 └── X Prismatic Joint
      │
      └── Y Prismatic Joint
           │
           └── Tool Joint
```

ตัวอย่างช่วง movement:

```text
X = 0–600 mm
Y = 0–400 mm
```

ตรงกับถาดประมาณ:

```text
40 × 60 cm
```

---

# 9. Sensor Mapping

Hardware จริงสามารถ map กับ Isaac Sim ได้ดังนี้:

| Hardware | Simulation |
|---|---|
| Webcam | RGB Camera |
| X Min Endstop | Joint Position / Contact |
| X Max Endstop | Joint Position / Contact |
| Y Min Endstop | Joint Position / Contact |
| Y Max Endstop | Joint Position / Contact |
| NEMA17 X | Prismatic Joint |
| NEMA17 Y | Prismatic Joint |
| MG996R | Revolute Joint |
| Emergency Switch | Virtual Digital Input |
| Tool Contact | Contact Sensor |

---

# 10. Controller Layer

โฟลเดอร์:

```text
controller/
```

เป็น Robot Brain ของระบบ

รับผิดชอบ:

- Workflow
- State machine
- Motion planning
- Safety logic
- Retry
- Timeout
- Fault management

---

# 11. State Machine

แนะนำให้ใช้ State Machine ตั้งแต่ POC

```text
BOOT
 │
 ▼
HOMING
 │
 ▼
IDLE
 │
 ▼
CAPTURE_IMAGE
 │
 ▼
DETECT_WEED
 │
 ▼
PLAN
 │
 ▼
MOVE_XY
 │
 ▼
TOOL_DOWN
 │
 ▼
TOOL_UP
 │
 ▼
VERIFY
 │
 ▼
NEXT_WEED
```

ตัวอย่าง State:

```python
from enum import Enum

class RobotState(Enum):
    BOOT = 1
    HOMING = 2
    IDLE = 3
    SCANNING = 4
    PLANNING = 5
    MOVING = 6
    WEEDING = 7
    ERROR = 8
    ESTOP = 9
```

---

# 12. Safety State

ต้องมี state ที่แยกจาก normal workflow

```text
NORMAL
   │
   ├── Endstop Error
   ├── Motor Timeout
   ├── Camera Timeout
   ├── Communication Lost
   ├── Position Mismatch
   └── Emergency Stop
          │
          ▼
        ERROR
          │
          ▼
        ESTOP
```

---

# 13. Perception Layer

โฟลเดอร์:

```text
perception/
```

รับผิดชอบ:

- Camera input
- Weed detection
- Segmentation
- Coordinate conversion
- Calibration

Pipeline:

```text
Camera
  │
  ▼
Image
  │
  ▼
Weed Detector
  │
  ▼
Pixel Coordinate
  │
  ▼
Camera Calibration
  │
  ▼
Machine Coordinate
  │
  ▼
Motion Planner
```

---

# 14. Coordinate Transformation

ตัวอย่าง Detection:

```text
weed pixel

u = 713
v = 422
```

แปลงเป็นตำแหน่งเครื่อง:

```text
X = 312.4 mm
Y = 184.2 mm
```

Module:

```text
perception/coordinate_transform.py
```

เป็นหนึ่งใน component ที่สำคัญที่สุดของระบบ

เพราะ Computer Vision ไม่ได้จบแค่:

```text
"เจอวัชพืช"
```

แต่ต้องตอบได้ว่า:

```text
"วัชพืชอยู่ตรงไหนบนโต๊ะจริง"
```

---

# 15. Camera Interface

ควรทำ Camera abstraction เช่นเดียวกับ Machine:

```python
class Camera:

    def capture(self):
        ...
```

Implementation:

```text
Camera
│
├── IsaacCamera
└── UsbCamera
```

ทำให้ perception pipeline ใช้ code ชุดเดียว

---

# 16. Firmware Architecture

ESP32 firmware ควรทำหน้าที่เป็น low-level controller

```text
ESP32
│
├── Motion
├── Endstop
├── Servo
├── Emergency Stop
├── Communication
└── Watchdog
```

ไม่ควรให้ ESP32 ทำ:

- Weed detection
- AI
- Global motion planning
- High-level workflow

---

# 17. ESP32 Firmware Layout

```text
firmware/esp32/
│
├── src/
│   ├── main.cpp
│   ├── motion.cpp
│   ├── io.cpp
│   ├── servo.cpp
│   └── communication.cpp
│
└── include/
    ├── motion.h
    ├── io.h
    └── protocol.h
```

---

# 18. ESP32 Motion Flow

```text
Controller
   │
   ▼
MOVE command
   │
   ▼
ESP32
   │
   ├── calculate step
   ├── output STEP/DIR
   ├── monitor endstop
   └── monitor timeout
          │
          ▼
       TMC2209
          │
          ▼
        NEMA17
```

---

# 19. Communication Bridge

โฟลเดอร์:

```text
bridge/
```

ใช้เชื่อม Controller กับ ESP32 หรือ Simulation

Protocol ที่ POC สามารถเริ่มได้:

```text
MQTT
```

หรือ:

```text
Serial
```

ภายหลังเปลี่ยนเป็น:

```text
UDP
CAN
MessagePack
Protobuf
```

โดยไม่เปลี่ยน Controller

---

# 20. Communication Protocol

ควรกำหนด contract กลาง

ตัวอย่าง Move Command:

```json
{
  "type": "move",
  "id": 123,
  "x_mm": 320.5,
  "y_mm": 180.0,
  "speed_mm_s": 50
}
```

ESP32 Response:

```json
{
  "type": "move_result",
  "id": 123,
  "status": "completed",
  "x_mm": 320.4,
  "y_mm": 180.1
}
```

---

# 21. Machine State Message

```json
{
  "type": "machine_state",

  "position": {
    "x_mm": 320.4,
    "y_mm": 180.1
  },

  "endstop": {
    "x_min": false,
    "x_max": false,
    "y_min": false,
    "y_max": false
  },

  "tool": {
    "state": "up"
  },

  "estop": false
}
```

---

# 22. Message Correlation

ทุก command ควรมี:

```text
id
```

เช่น:

```text
Command ID = 1001
```

เพื่อจับคู่:

```text
COMMAND
   ↓
ACK
   ↓
RESULT
```

ตัวอย่าง:

```text
MOVE id=1001

     ↓

ACK id=1001

     ↓

MOVE_RESULT id=1001
```

---

# 23. Configuration

อย่า hard-code parameter ใน source code

ใช้:

```text
config/
```

---

# 24. Machine Config

`config/machine.yaml`

```yaml
machine:

  workspace:
    width_mm: 600
    height_mm: 400

  x_axis:
    min_mm: 0
    max_mm: 600
    max_speed_mm_s: 100
    acceleration_mm_s2: 200

  y_axis:
    min_mm: 0
    max_mm: 400
    max_speed_mm_s: 100
    acceleration_mm_s2: 200

tool:

  servo:
    up_angle: 30
    down_angle: 85
```

---

# 25. Simulation Config

`config/simulation.yaml`

```yaml
simulation:

  physics_hz: 60
  render_hz: 30

  camera:
    width: 1280
    height: 720

  environment:
    plant_count: 20
    weed_count: 10

  randomization:
    enabled: true
```

---

# 26. Development Config

```yaml
backend: isaac

logging:
  level: INFO

controller:
  timeout_seconds: 5

safety:
  enable_estop: true
```

Hardware:

```yaml
backend: esp32
```

Simulation:

```yaml
backend: isaac
```

Unit test:

```yaml
backend: fake
```

---

# 27. Simulation Environment Generator

ควรมีระบบสุ่ม scene:

```text
sim/isaac/environments/
```

เช่น:

```python
generate_scene(
    plant_count=20,
    weed_count=10,
)
```

สามารถ random:

- ตำแหน่งต้นไม้
- ตำแหน่งวัชพืช
- ขนาดวัชพืช
- Rotation
- Lighting
- Camera noise
- Color variation

---

# 28. Domain Randomization

เป้าหมายคือให้ vision system ไม่จำ scene เดิม

ตัวอย่าง:

```text
Run #1

🌱       🌿
     🌱
             🌿


Run #2

       🌱
🌿
            🌱
       🌿
```

ใช้ทดสอบ robustness ของ AI

---

# 29. Test Architecture

```text
tests/
│
├── unit/
├── integration/
├── simulation/
└── scenarios/
```

---

# 30. Unit Tests

ตัวอย่าง:

```text
coordinate transform
motion planner
state transition
protocol encoding
workspace limit
```

ไม่จำเป็นต้องเปิด Isaac Sim

---

# 31. Integration Tests

ทดสอบ:

```text
Controller
+
FakeMachine
```

หรือ:

```text
Controller
+
Bridge
+
ESP32 Emulator
```

---

# 32. Simulation Tests

ใช้:

```text
Controller
+
IsaacMachine
+
Isaac Sim
```

ทดสอบ Physics และ Robot behavior

---

# 33. Scenario Tests

```text
tests/scenarios/
```

ตัวอย่าง:

```text
normal_10_weeds.yaml

weed_near_x_min.yaml

weed_near_x_max.yaml

endstop_failure.yaml

camera_delay.yaml

motor_timeout.yaml

estop_during_motion.yaml

100_random_weeds.yaml
```

---

# 34. Example Scenario

```yaml
name: estop-during-motion

machine:

  start:
    x: 0
    y: 0

target:

  x: 500
  y: 300

events:

  - at: 1.5

    set:
      estop: true

expect:

  state: ESTOP
  motor_running: false
```

---

# 35. Automated Simulation Testing

เป้าหมายระยะถัดไป:

```text
git push
   │
   ▼
CI Pipeline
   │
   ▼
Start Isaac Sim Headless
   │
   ▼
Run Scenarios
   │
   ▼
100 Simulation Runs
   │
   ▼
PASS / FAIL
```

---

# 36. Recommended Development Phases

## V0 — Fake Machine

ยังไม่ใช้ Isaac Sim

```text
Controller
   │
   ▼
FakeMachine
```

เป้าหมาย:

- State machine
- Motion interface
- Safety design
- Protocol design

---

# 37. V1 — Isaac Simulation

```text
Controller
   │
   ▼
IsaacMachine
   │
   ▼
Isaac Sim
```

Scene มี:

- Tray
- X/Y Gantry
- Tool
- Camera
- Endstop

เป้าหมาย:

```text
Home
Move
Tool Up/Down
Endstop
Collision
```

---

# 38. V2 — Computer Vision

เพิ่ม:

```text
Isaac Camera
    │
    ▼
Perception
    │
    ▼
Weed Detection
    │
    ▼
Coordinate Transform
    │
    ▼
Controller
```

Full workflow:

```text
Capture
  ↓
Detect
  ↓
Calculate X/Y
  ↓
Move
  ↓
Tool Down
  ↓
Tool Up
```

---

# 39. V3 — Hardware-in-the-Loop

ใช้ ESP32 จริง แต่ environment ยังเป็น virtual

```text
           Isaac Sim

        Virtual Machine
               │
               │ sensor state
               ▼
             ESP32
               │
               │ motor command
               ▼
           Isaac Sim
```

ใช้ทดสอบ:

- ESP32 firmware
- communication
- watchdog
- timeout
- endstop logic

---

# 40. V4 — Real Machine

```text
Real Camera
    │
    ▼
Perception
    │
    ▼
Controller
    │
    ▼
ESP32
    │
    ▼
TMC2209
    │
    ▼
NEMA17
    │
    ▼
Real Gantry
```

---

# 41. Software-in-the-Loop

SIL Architecture:

```text
Isaac Sim
   │
   ▼
Python Controller
   │
   ▼
Virtual Machine Interface
```

สามารถทดสอบ robot logic โดยไม่ต้องมี Hardware

---

# 42. Hardware-in-the-Loop

HIL Architecture:

```text
Isaac Sim
   │
   │ Virtual Sensor
   ▼
Real ESP32
   │
   │ Motor Command
   ▼
Isaac Sim
```

ช่วยทดสอบ firmware จริงก่อนต่อ motor จริง

---

# 43. Production Architecture

เมื่อ POC สำเร็จ:

```text
Camera
   │
   ▼
Edge Computer
   │
   ├── Perception
   ├── Motion Planner
   └── Controller
          │
          ▼
        ESP32
          │
   ┌──────┼─────────┐
   ▼      ▼         ▼
 Motor   Servo    Endstop
```

---

# 44. Why Not Use ROS2 Initially

สำหรับ POC นี้มี:

- 2 Linear Axis
- 1 Servo
- 1 Camera
- 4 Endstop

ดังนั้นใน V1 ยังไม่จำเป็นต้องใช้ ROS2

เริ่มด้วย:

```text
Python
+
Isaac Sim API
+
Machine Abstraction
```

จะง่ายกว่า

---

# 45. When ROS2 Becomes Useful

ควรเพิ่ม ROS2 เมื่อระบบมี:

- Robot หลายตัว
- Sensor หลายชุด
- Navigation
- Distributed process
- Hardware driver หลายชนิด
- External robotics ecosystem
- SLAM
- MoveIt
- ROS Control

Architecture ภายหลัง:

```text
Perception
    │
    ▼
ROS2
    │
    ├── Controller Node
    ├── Camera Node
    ├── ESP32 Bridge
    └── Isaac Sim
```

---

# 46. Recommended Technology Stack

| Component | Technology |
|---|---|
| Simulation | NVIDIA Isaac Sim |
| Controller | Python |
| Computer Vision | OpenCV / PyTorch |
| Firmware | ESP-IDF / Arduino |
| Firmware Build | PlatformIO |
| Communication | MQTT / Serial |
| Config | YAML |
| Protocol | JSON |
| Testing | pytest |
| Scenario Testing | YAML |
| Robot Model | USD |
| Version Control | Git |

---

# 47. Recommended First Milestone

อย่าเริ่มด้วย AI ก่อน

Milestone แรกควรทำ:

```text
Isaac Sim
   │
   ├── Table
   ├── Tray
   ├── X Joint
   ├── Y Joint
   ├── Tool
   └── Camera

        ↓

Python Controller

        ↓

Home

        ↓

Move X/Y

        ↓

Tool Down

        ↓

Tool Up
```

Acceptance Criteria:

```text
[ ] Gantry move X ได้
[ ] Gantry move Y ได้
[ ] Joint limit ทำงาน
[ ] Endstop ทำงาน
[ ] Home ได้
[ ] Tool up/down ได้
[ ] Camera capture ได้
```

---

# 48. Second Milestone

เพิ่ม plant และ weed

```text
Camera
   ↓
Image
   ↓
Simple Color Detection
   ↓
Weed X/Y
   ↓
Move
   ↓
Mark
```

ตอนแรกยังไม่ต้องใช้ Deep Learning

ใช้:

```text
OpenCV
Color Threshold
Shape Detection
```

เพื่อพิสูจน์ architecture ก่อน

---

# 49. Third Milestone

หลังจาก mechanical + coordinate pipeline work แล้วค่อยเพิ่ม:

```text
AI Weed Detection
```

เช่น:

```text
YOLO
Segmentation Model
Custom Weed Detector
```

---

# 50. Recommended Final Repository Boundary

หลักการสำคัญที่สุดคือ:

```text
Simulation != Controller
Controller != Hardware
Hardware != Perception
Perception != Simulation
```

เชื่อมกันผ่าน Interface

```text
                 Interfaces

    Perception ────────────────┐
                               │
                               ▼
                          Controller
                               │
                               ▼
                       Machine Interface
                         │           │
                         ▼           ▼
                      Isaac        ESP32
```

Architecture นี้จะทำให้ระบบสามารถพัฒนาจาก:

```text
Laptop
+
RTX 3060
+
Isaac Sim
```

ไปสู่:

```text
Real Camera
+
ESP32
+
NEMA17
+
TMC2209
+
Physical Weeding Robot
```

โดยใช้ Controller และ Perception architecture ชุดเดิม

---

# 51. Summary

สำหรับ Mini Smart Weeding Table แนะนำให้ใช้ **Monorepo + Clean Hardware Abstraction**

Core modules:

```text
sim/
controller/
perception/
firmware/
bridge/
protocol/
config/
tests/
```

ลำดับการพัฒนา:

```text
Fake Machine
    ↓
Isaac Sim
    ↓
Computer Vision
    ↓
ESP32 HIL
    ↓
Real Hardware
```

สิ่งที่ควรให้ความสำคัญตั้งแต่ต้น:

1. Machine Interface
2. Camera Interface
3. State Machine
4. Coordinate Transform
5. Communication Protocol
6. Safety / E-Stop
7. Scenario Testing

ถ้าแยก architecture ตามนี้ จะสามารถใช้ Isaac Sim เป็นทั้ง:

- Development Environment
- Digital Twin
- Robot Simulator
- Sensor Simulator
- Automated Test Environment
- Hardware-in-the-Loop Environment

ได้ใน repository เดียว
