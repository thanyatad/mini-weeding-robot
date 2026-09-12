# controller

**Robot Brain** ของระบบ

---

## Responsibility

- Workflow
- State machine
- Row following
- Safety logic
- Startup invariant validation
- Timeout
- Fault management

Controller ต้องไม่รู้ว่ากำลังควบคุม Simulation หรือ Hardware จริง —
คุยผ่าน `Rover` และ `CameraSet` interface เท่านั้น

---

## Layout

```text
controller/
├── main.py                     # startup: load config → validate → select backend → run
├── startup_checks.py           # invariant 9 ข้อ — fail = ไม่ start
│
├── rover/
│   ├── base.py                 # Rover Protocol (the contract)
│   ├── isaac_rover.py          # → sim/isaac/adapters/
│   ├── esp32_rover.py          # → bridge/
│   ├── fake_rover.py           # kinematic-only, integrate (v, omega) → pose ภายใน
│   └── mixing.py               # skid-steer mixing + saturation (scale ไม่ clip)
│
├── motion/
│   ├── row_follower.py         # RowEstimate → (v, omega)   pure function
│   └── limits.py               # clamp v / omega ตาม config
│
├── workflow/
│   ├── state_machine.py        # RoverState transitions
│   └── row_run.py              # ลูป: estimate → follow → drive → log
│
└── safety/
    ├── runaway.py              # row-loss watchdog
    ├── link_monitor.py         # link_age_ms → link_lost
    ├── emergency_stop.py
    └── fault_manager.py
```

---

## Rover Interface

```python
from typing import Protocol


class Rover(Protocol):

    def drive(self, v_mm_s: float, omega_deg_s: float) -> None: ...

    def stop(self) -> None: ...

    # {"commanded": {"v_mm_s": float, "omega_deg_s": float},
    #  "estop": bool, "link_age_ms": int}
    def get_drive_state(self) -> dict: ...

    def emergency_stop(self) -> None: ...
```

`get_drive_state()` **ไม่มี field `measured` โดยเจตนา** — ไม่มี encoder จึงไม่มีใคร
รู้ว่าล้อหมุนจริงเท่าไร การซ้อนใต้ `commanded` ทำให้ call site อ่านออกว่าเป็น echo
ของคำสั่ง ไม่ใช่ feedback

ทุก backend ต้องคืน **key ชุดเดียวกันเป๊ะ** — มี test บังคับข้อนี้

Backend เลือกที่ startup เท่านั้น:

```python
if config.backend == "isaac":
    rover = IsaacRover()

elif config.backend == "esp32":
    rover = Esp32Rover()

else:
    rover = FakeRover()
```

---

## `row_follower.py` — pure function

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
เท่านั้น ทำให้ test ได้ด้วยตัวเลขสังเคราะห์ ไม่ต้องมีภาพ ไม่ต้องเปิด Isaac:

```python
def test_row_right_of_center_turns_right():
    v, omega = follower.step(RowEstimate(valid=True, lateral_err=+0.5, ...))
    assert omega < 0
```

### ทำไม P ไม่ใช่ PID

`heading_err` คืออัตราการเปลี่ยนของ `lateral_err` ในเชิงเรขาคณิตอยู่แล้ว
`k_head` จึงทำหน้าที่เป็นเทิร์ม **damping** โดยไม่ต้องหาอนุพันธ์จากสัญญาณภาพที่มี noise

ไม่เอา integral เพราะ offset ค้างในร่องไม่คุ้มกับความเสี่ยง windup

ความเร็ว `v` คงที่ใน MVP — การลดความเร็วเมื่อเบี่ยงมากเป็น knob ที่เพิ่มภายหลังได้

### Gain ไม่โอนข้าม backend

`config/control.yaml` เก็บ gain แยกตาม backend และ `esp32` เป็น `null`
จนกว่าจะ tune บนของจริง — slip ของ skid-steer บนดินจริงไม่เท่ากับใน sim

---

## `mixing.py` — saturation ต้อง scale ไม่ clip

```python
v_left  = v - omega_rad * track_width_mm / 2
v_right = v + omega_rad * track_width_mm / 2
```

ถ้าล้อข้างใดเกิน `wheel_v_max_mm_s` → **ลดทั้งสองข้างตามอัตราส่วนเดิม**

การ clip ข้างเดียวจะเปลี่ยน `omega` ที่ได้จริงโดยเงียบ ซึ่งเป็นความผิดพลาด
ประเภทเดียวกับ deadband ที่ทำให้ต้องลด `omega_max` จาก 60 เป็น 40

สูตรนี้ถูก implement ซ้ำใน C++ ที่ firmware ทั้งสองต้องผ่าน
[`config/drive_mixing_vectors.csv`](../config/drive_mixing_vectors.csv) ตารางเดียวกัน

---

## State Machine

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
ร่องเป้าหมาย ซึ่งไม่มี odometry มารองรับ (→ M4)

### `row_end` กับ `row_lost` แยกด้วย `green_fraction`

| `green_fraction` ตอน invalid | ตีความ | ปลายทาง |
|---|---|---|
| ต่ำ — สีเขียวหมดจากเฟรม | สุดร่องแล้ว | `STOPPED(row_end_suspected)` |
| สูง — ยังเห็นเขียวแต่จับเส้นไม่ได้ | navigation พลาด | `ERROR(row_lost)` |

ชื่อ `row_end_suspected` เตือนว่านี่คือ **heuristic ไม่ใช่การวัด** — scenario test
`row_end.yaml` กับ `crop_gap_midrow.yaml` เป็นคู่ตรงข้ามที่ต้องผ่านทั้งคู่
ผ่านข้างเดียวแปลว่า threshold ตั้งเอาใจข้างเดียว

---

## Control Loop

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

`weed_log` อยู่ในลูปแต่ **ไม่อยู่ใน control path** — detection ช้าหรือพลาด
ต้องไม่ทำให้ rover เลี้ยวผิด rate แยกกันชัดเจน (front 10 Hz, down 1-2 Hz)

---

## Safety — 3 ชั้น 3 เจ้าของ

| กลไก | เจ้าของ | อยู่ที่ไหน |
|---|---|---|
| Command timeout | **ESP32 firmware** | `firmware/esp32/` |
| Row-loss watchdog | **controller** | `safety/runaway.py` |
| Latching E-stop | **hardware** | ตัด motor rail ทางไฟ |

**แต่ละชั้นทำงานได้โดยไม่ต้องพึ่งชั้นบน** — ถ้าชั้นใดพึ่งชั้นบน
มันไม่ใช่ safety layer มันคือ feature

Controller ครองแค่ชั้นกลาง และต้องเขียนโดยสมมติว่าอีกสองชั้นมีอยู่จริง —
ไม่ใช่พยายามทำงานแทนทั้งสามชั้น

### `command_timeout` ที่รายงานกลับมา

เมื่อ ESP32 รายงาน `command_timeout` แปลว่ามอเตอร์ถูกดับไปแล้ว
Controller ต้องถือว่า **rover หยุดแล้วจริง** เข้า `ERROR` และ **ห้าม resume เอง**
การส่ง `drive` ต่อทันทีหลังเห็น error นี้คือการทำให้ rover ออกตัวโดยไม่มีคนสั่ง

---

## Startup Validation

`main.py` เรียก `startup_checks.py` ก่อน start งานจริง — **9 ข้อ fail ข้อใดก็ไม่ start**

```text
safety      v_max × command_timeout/1000        <= runaway_budget
safety      v_max × row_loss_frames/loop_hz     <= runaway_budget
geometry    wheel_diameter        >= 4 × soil_variation
geometry    chassis_clearance     >  soil_variation
geometry    clear_furrow          >  body_width + 2 × runaway_budget
drive       v + omega_max_rad × track/2  <= wheel_v_max
drive       v − omega_max_rad × track/2  >= wheel_v_min
config      gains[backend] ไม่เป็น null
consistency simulation.soil.variation_mm == bed.soil_variation_mm
```

Error message ต้องบอก **ค่าที่ขัดกันเป็นตัวเลข** ไม่ใช่แค่ชื่อ invariant:

```text
config_invalid: clear_furrow (290 mm) must exceed
                body_width (146) + 2 × runaway_budget (60) = 266 mm
                → เพิ่ม row_spacing_mm หรือลด runaway_budget_mm
```

ค่าเต็มและที่มา: [../config/README.md](../config/README.md#startup-invariants)

---

## `FakeRover` ปิดลูปได้โดยไม่ต้องมีกราฟิก

```text
FakeRover        integrate (v, omega) → pose ภายใน    (kinematic only)
FakeRowSensor    pose + สมการเส้นร่อง → RowEstimate
RowFollower      RowEstimate → (v, omega)
                        └──> วนกลับเข้า FakeRover
```

```python
def test_converges_from_offset():
    sim = FakeLoop(row=straight_row(), start_offset_mm=60, start_heading_deg=15)
    sim.run(seconds=3.0)
    assert abs(sim.history[-1].lateral_err) < 0.1
    assert sim.min_clearance_mm > 0        # ไม่เบียดต้นพืชระหว่างทาง
```

ตอบคำถาม *"gain ชุดนี้ลู่เข้าไหม และแกว่งเกินร่องไหม"* ในหลักมิลลิวินาที อยู่ใน CI ได้
ก่อนแตะ Isaac เลย

### ⚠️ `FakeRover.pose` ต้องไม่หลุดออก interface

`FakeRover` มี pose ภายใน แต่ **ห้าม expose ผ่าน `get_drive_state()`**

ถ้าหลุดออกมา code ที่เขียนตอน V0 จะพึ่งพา pose แล้วพังทั้งหมดตอนเปลี่ยนเป็น
`esp32` ที่ไม่มี pose ให้ — และจะพังตอนที่ debug ยากที่สุด คือตอนอยู่ในแปลง

`FakeRowSensor` เข้าถึง pose ได้ (มันคือ test harness) แต่ `RowFollower`,
`state_machine` และทุกอย่างใน `workflow/` ห้าม

---

## Run

```bash
./scripts/run_controller.sh          # ตาม backend ใน config/development.yaml
./scripts/run_fake_loop.sh           # closed-loop V0 ไม่ต้องมี Isaac
```
