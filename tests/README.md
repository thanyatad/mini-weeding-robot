# tests

Framework: **pytest** · Scenario format: **YAML**

---

## Layout

```text
tests/
├── unit/            # pure logic — ไม่ต้องเปิด Isaac Sim
├── integration/     # Controller + FakeRover / Bridge + ESP32 Emulator
├── simulation/      # Controller + IsaacRover + Isaac Sim
└── scenarios/       # YAML scenario definitions
```

---

## Unit Tests

`backend: fake` — ไม่จำเป็นต้องเปิด Isaac Sim

ครอบคลุม:

```text
row follower  (RowEstimate → v, omega)
closed-loop convergence ด้วย FakeRover     ◄── ดู §Closed Loop
skid-steer mixing + saturation
startup invariants (9 ข้อ ทีละข้อ)
ExG + Otsu + absolute floor
valley finder
weed detector corridor filter
state transition
protocol encoding (streaming vs discrete)
seq tracker / link_age_ms
coordinate transform round-trip (กล้องล่าง)
CAD ↔ config sync
```

### `get_drive_state()` ต้องคืน key ชุดเดียวกันทุก backend

```python
def test_drive_state_shape_matches_across_backends():
    keys = {frozenset(r.get_drive_state()) for r in (FakeRover(), IsaacRover(), Esp32Rover())}
    assert len(keys) == 1
```

ถ้า `FakeRover` เผลอ expose `pose` ออกมา test นี้จับได้ทันที — ป้องกัน code
ที่เขียนตอน V0 พึ่งพา pose แล้วพังตอนเปลี่ยนเป็น `esp32`

---

## Closed Loop โดยไม่ต้องมีกราฟิก

ส่วนที่คุ้มที่สุดของ test suite นี้ และ gantry ทำไม่ได้

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

รันในหลักมิลลิวินาที อยู่ใน CI ได้ ตอบคำถาม *"gain ชุดนี้ลู่เข้าไหม
และแกว่งเกินร่องไหม"* **ก่อนแตะ Isaac เลย**

`min_clearance_mm > 0` สำคัญเท่า `lateral_err` — gain ที่ลู่เข้าเร็วแต่แกว่ง
เกินร่องระหว่างทางคือ gain ที่พาไปชนต้นพืช test ที่ดูแค่ค่าสุดท้ายจะปล่อยผ่าน

---

## Golden Mixing Vectors — test ข้ามภาษา

สูตร skid-steer mixing ถูก implement 2 ที่ที่แชร์โค้ดกันไม่ได้

```text
config/drive_mixing_vectors.csv     ◄── source of truth เดียว
        │
        ├──> tests/unit/test_drive_mixing.py          Python / IsaacRover
        └──> firmware/esp32/test/test_mixing.cpp      C++ / PlatformIO native
```

```bash
pytest tests/unit/test_drive_mixing.py
pio test -e native -d firmware/esp32
```

ทั้งสองต้องผ่านตารางเดียวกัน **ห้ามแก้ค่าในตารางเพื่อให้ test ผ่าน** — แก้ implementation

ตารางมี row ที่ทดสอบ saturation (`500, 25`) ซึ่งต้อง scale ทั้งสองข้าง
ไม่ใช่ clip ข้างเดียว — ที่ track 430 กับ `wheel_v_max` 327 แถวนี้ให้
`v_left`/`v_right` ก่อน scale เป็น 406.189/593.811 mm/s ซึ่งเกิน 327 ทั้งคู่
scale ด้วยอัตราส่วนเดิมจนล้อที่เร็วกว่าแตะเพดานพอดี ให้ `223.680 / 327.000`
(ดู [`config/drive_mixing_vectors.csv`](../config/drive_mixing_vectors.csv))
จึงแยกได้ชัดว่า implementation scale หรือ clip

---

## Startup Invariants — test ละข้อ

invariant 9 ข้อ ต้องมี test ที่ **ป้อนค่าที่ขัดกันแล้วยืนยันว่า fail จริง**
ไม่ใช่แค่ test ว่าค่าปัจจุบันผ่าน

```python
@pytest.mark.parametrize("override, expect_in_message", [
    ({"safety.runaway_budget_mm": 20}, "runaway_budget"),
    ({"bed.row_spacing_mm": 250}, "clear_furrow"),
    ({"rover.drive.omega_max_deg_s": 60}, "wheel_v_min"),
    ({"rover.wheel_diameter_mm": 40}, "soil_variation"),
    ...
])
def test_invariant_rejects_bad_config(override, expect_in_message):
    with pytest.raises(ConfigInvalid) as e:
        startup_checks.run(config_with(override))
    assert expect_in_message in str(e.value)
```

Test ต้องยืนยันว่า **error message มีค่าที่ขัดกันเป็นตัวเลข** ไม่ใช่แค่ชื่อ invariant —
เพราะจุดประสงค์ของ invariant คือบอกคนแก้ว่าต้องแก้อะไร

สามค่าใน `parametrize` ข้างบน (`250`, `60`, `40`) คือค่าที่ **เคยถูกตั้งไว้จริง
ตอนร่าง spec** แล้ว invariant จับได้ — เก็บเป็น regression test

---

## Integration Tests

```text
Controller + FakeRover
```

หรือ:

```text
Controller + Bridge + ESP32 Emulator
```

`bridge/simulator.py` ต้องเลียนพฤติกรรมที่ firmware จริงต้องมี ไม่ใช่แค่ตอบ ack:

```text
[ ] command timeout → motors_enabled: false
[ ] latest-wins (ยัด drive หลายตัว ใช้ seq สูงสุด)
[ ] ทิ้ง seq ย้อนหลัง
[ ] stop / emergency_stop override drive ในรอบเดียวกัน
[ ] ปฏิเสธ reset ขณะ estop sense ยัง active
[ ] uptime_ms ลดลงเมื่อจำลองรีบูต
```

Emulator ที่ตอบ ack ทุกอย่างแบบสุภาพจะทำให้ integration test ผ่านหมด
แล้วไปพังที่ V3 ซึ่ง debug แพงกว่ามาก

---

## Simulation Tests

```text
Controller + IsaacRover + Isaac Sim
```

ทดสอบ physics และพฤติกรรมของ rover จริง — ล้อบน heightfield, slip, ล้อลอย

---

## Scenario Tests

`tests/scenarios/`

```text
row_straight.yaml
row_curved.yaml
row_tilted_start.yaml
crop_gap_midrow.yaml
row_end.yaml
row_lost.yaml
soil_rough.yaml
link_lost.yaml
estop_during_drive.yaml
weed_in_furrow.yaml
```

| Scenario | ทดสอบ |
|---|---|
| `row_straight.yaml` | baseline — ตามร่องตรง 2000 mm ได้ |
| `row_curved.yaml` | ตาม `row_curvature_mm` ได้โดยไม่เบียดต้นพืช |
| `row_tilted_start.yaml` | เริ่มเอียง 15° ต้องเข้าร่องได้ |
| `crop_gap_midrow.yaml` | ช่องว่างกลางแถว → **ต้องไม่หยุด** |
| `row_end.yaml` | สุดร่อง → `STOPPED(row_end_suspected)` |
| `row_lost.yaml` | ถอดแถวกลางทางแต่ยังมีเขียว → `ERROR(row_lost)` |
| `soil_rough.yaml` | ±15 mm ล้อลอย ต้องยังตามร่องได้ |
| `link_lost.yaml` | Pi ↔ ESP32 หลุด → มอเตอร์ดับใน `command_timeout_ms` |
| `estop_during_drive.yaml` | กด E-stop ระหว่างวิ่ง |
| `weed_in_furrow.yaml` | detect วัชพืชในร่องได้ตามเกณฑ์ precision/recall |

### `crop_gap_midrow` กับ `row_end` เป็นคู่ตรงข้าม

ทั้งสองคือ "`green_fraction` ตก" แต่ต้องได้ผลต่างกัน:

```text
ช่องว่างกลางแถว   green ตกชั่วคราว   → ต้องวิ่งต่อ
สุดร่อง           green หมดถาวร     → ต้องหยุด
```

**ต้องผ่านทั้งคู่** — ผ่านข้างเดียวแปลว่า threshold ตั้งเอาใจข้างเดียว
และจะพังในแปลงจริงที่มีทั้งสองอย่าง

### Example — row curved

```yaml
name: row-curved

bed:
  bed_length_mm: 2000
  row_spacing_mm: 750
  row_curvature_mm: 40
  crop_gap_probability: 0

rover:
  start:
    lateral_offset_mm: 0
    heading_deg: 0

expect:
  final_state: STOPPED
  stop_reason: row_end_suspected
  distance_travelled_mm: { min: 1900 }
  min_clearance_mm: { min: 10 }
  max_lateral_error_mm: { max: 40 }     # ยืนยันสมมติฐานของ corridor
```

### Example — crop gap mid-row

```yaml
name: crop-gap-midrow

bed:
  bed_length_mm: 2000
  crop_gap_probability: 0.08
  gap_features:
    - at_mm: 1000
      length_mm: 240          # ต้นหาย 3 ต้นติดกัน

expect:
  final_state: STOPPED
  stop_reason: row_end_suspected     # หยุดที่ปลายร่อง ไม่ใช่ที่ช่องว่าง
  stopped_before_mm: { min: 1900 }   # ◄── ไม่หยุดที่ 1000
  error_code: null
```

### Example — estop during drive

```yaml
name: estop-during-drive

events:
  - at: 1.5
    set:
      estop: true

expect:
  final_state: ESTOP
  motors_enabled: false
  commanded_after_estop: { v_mm_s: 0 }
```

---

## Scenario ที่ตัดออกจาก design gantry

```text
weed_near_x_min · weed_near_x_max     ไม่มี workspace X/Y
endstop_failure                       ไม่มี endstop
motor_timeout                         → link_lost
camera_delay                          → camera_timeout (ยังมี แต่รวมใน link/timeout suite)
normal_bed_10_weeds · 100_random_weeds  → weed_in_furrow (วัดด้วย precision/recall ต่อเฟรม)
soil_high_spot · soil_low_spot        ต้องมี tool  → กลับมาที่ M3
weed_inside_crop_row                  ต้องมี in-row detection  → กลับมาที่ M3
crop_row_misaligned                   → row_curved (ครอบคลุมกว่า)
```

`soil_high_spot` / `soil_low_spot` / `weed_inside_crop_row` **ไม่ถูกลบทิ้ง** —
ยกไปที่ M3 พร้อม tool นิยามเดิมใน
[spec แปลงดิน §9](../docs/superpowers/specs/2026-09-12-weeding-bed-design.md) ยังถูกต้อง

---

## เกณฑ์ต้องวัดได้

ทุก `expect` ในไฟล์ scenario ต้องเป็นค่าที่วัดได้ ไม่ใช่ "ทำงานได้"

```text
✓ min_clearance_mm: { min: 10 }
✓ per-frame precision: { min: 0.9 }
✗ row_following: works
```

และ weed detection วัดด้วย **per-frame precision/recall** ไม่ใช่จำนวนรวม —
ไม่มี odometry จึง dedupe ไม่ได้ `weed_log` นับ detection ไม่ใช่นับต้น
ตัวเลขรวมไม่มีความหมาย

---

## Automated Simulation Testing

```text
git push
   │
   ▼
CI Pipeline
   │
   ├── pytest tests/unit/           ◄── รวม closed loop, ไม่ต้องมี GPU
   ├── pio test -e native           ◄── mixing vectors ฝั่ง C++
   ├── pytest tests/integration/
   │
   ▼
Start Isaac Sim Headless
   │
   ▼
Run Scenarios × randomization seeds
   │
   ▼
PASS / FAIL
```

`tests/unit/` ต้องรันได้เร็วและไม่ต้องมี GPU — closed-loop convergence test
อยู่ในชั้นนี้ได้เพราะไม่มี physics และไม่มีภาพ นั่นคือเหตุผลที่ออกแบบ
`FakeRover` + `FakeRowSensor` แบบนั้น
