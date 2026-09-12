# sim/isaac

NVIDIA Isaac Sim module — จำลอง **แปลงปลูกร่องยาว** ที่มี rover 4 ล้อวิ่งในร่องระหว่างแถว

รายละเอียดเต็ม: [../../docs/simulation.md](../../docs/simulation.md)

---

## Layout

```text
sim/isaac/
├── app.py                      # Isaac Sim application entry point
├── world.py                    # World / stage setup
│
├── scenes/
│   ├── weeding_bed.usd         # scene หลัก (ร่องยาว 2000 × 1000 mm)
│   ├── bed_frame.usd           # ขอบแปลง — ยกสูงกว่า wheel_diameter/2
│   ├── soil_heightfield.usd    # ผิวดิน (generate จาก seed)
│   ├── crops/                  # โมเดลพืชผล
│   └── weeds/                  # โมเดลวัชพืช
│
├── robots/
│   └── rover/
│       ├── rover_base.usd      # GENERATED จาก cad/urdf — gitignored, อย่าแก้
│       ├── rover.usd           # override layer: drive gain, friction, damping, sensor
│       ├── wheels.py           # Joint handles: fl, fr, rl, rr
│       └── config.yaml         # Joint drive gains, friction
│
├── sensors/
│   ├── camera_front.py         # → CameraSet.front()   tilt ~50°
│   ├── camera_down.py          # → CameraSet.down()    top-down
│   ├── estop.py                # virtual digital input
│   └── sensor_manager.py
│
├── environments/
│   ├── soil_heightfield.py     # generate ผิวดินจาก seed
│   ├── crop_rows.py            # วางพืชผลเป็นแถว + jitter + row angle + curvature + gap
│   ├── random_weeds.py         # สุ่มวัชพืชในร่อง / ในแถว
│   └── domain_randomization.py
│
└── adapters/
    ├── wheel_adapter.py        # (v, omega) → mixing → wheel joint velocity
    └── io_adapter.py           # virtual digital I/O (estop) + command timeout
```

---

## Boundary

✅ รับผิดชอบ: physics, แปลงดิน/soil heightfield, scene, sensor, rover model,
collision, joint, environment, simulation I/O

❌ **ไม่** รับผิดชอบ: weed detection workflow, row following policy, business logic,
state machine, safety policy หลัก

`adapters/` เป็นจุดเดียวที่ `controller/rover/isaac_rover.py` คุยด้วย

---

## Bed Geometry

```text
Bed                       2000 × 1000 mm
Crop rows                 3 แถว ตามยาว
row_spacing               750 mm      กึ่งกลางแถวถึงกึ่งกลางแถว
crop_foliage_half_width    30 mm
clear furrow              690 mm      ◄── ค่าที่ rover ใช้จริง
Furrows                   2 ร่อง — MVP วิ่งร่องเดียว
```

⚠️ ความกว้างแปลง 1000 mm ด้านบนไม่มีที่มาที่ตรวจสอบได้ในนี้ (ไม่มี config key
หรือ test อ้างอิงมัน) — ดูรายละเอียดที่
[docs/simulation.md#bed-geometry](../../docs/simulation.md#bed-geometry)

**`row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้** — `clear_furrow` คือค่าที่
startup invariant ใช้ ไม่ใช่ `row_spacing`

**`bed_frame.usd` ขอบยกสูงกว่าล้อ ไม่ใช่ของประดับ** — MVP ไม่มี bumper switch
ขอบยกเป็น compensating control ทางกายภาพ ต้องมีทั้งใน sim และของจริง

---

## Soil

ดินเป็น **rigid heightfield mesh ผิวไม่เรียบ ±15 mm — ไม่ยุบตัว**

ได้ collision ของล้อกับผิวไม่เรียบ และ CV background ที่สมจริง โดย physics
ยัง stable และรัน headless CI ได้

ไม่ได้: ดินยุบ, ล้อจม, แรงต้าน, ร่องล้อที่ทิ้งไว้

`soil.variation_mm` ใน `config/simulation.yaml` ต้องตรงกับ `bed.soil_variation_mm`
ใน `config/rover.yaml` — เป็น **startup invariant ข้อที่ 9** ไม่ใช่แค่คำแนะนำ

---

## Rover Model

Skid-steer 4WD — 4 ล้อขับ ไม่มีพวงมาลัย

```text
base_link
 ├── wheel_fl · wheel_fr · wheel_rl · wheel_rr    continuous joint, velocity drive
 ├── camera_front   fixed, tilt ~50°
 └── camera_down    fixed, top-down
```

ล้อหน้าและหลังข้างเดียวกันได้คำสั่ง**เหมือนกันเสมอ** (`wheel_adapter.py` ส่งค่าเดียว
ไป 2 joint) ตรงกับของจริงที่ต่อมอเตอร์ขนานเป็นข้าง

**ไม่มี suspension** — chassis แข็ง 4 ล้อ บนผิว ±15 mm จะมีจังหวะล้อลอยและ
เสียแรงขับ นี่ไม่ใช่ข้อบกพร่องของ sim ที่ต้องแก้ ถ้าเกิดใน sim มันจะเกิดของจริงด้วย
**ให้ sim บอกก่อนซื้อล้อ**

---

## `wheel_adapter.py` — mixing ต้องตรงกับ firmware

```text
omega_rad   = omega_deg × pi / 180
v_left      = v - omega_rad × track_width_mm / 2
v_right     = v + omega_rad × track_width_mm / 2
wheel_rad_s = v_side / (wheel_diameter_mm / 2)
```

Saturation ต้อง **scale ทั้งสองข้างตามอัตราส่วนเดิม** ไม่ใช่ clip ข้างเดียว

สูตรนี้ถูก implement ซ้ำใน C++ ที่ firmware — ทั้งคู่ต้องผ่าน
[`config/drive_mixing_vectors.csv`](../../config/drive_mixing_vectors.csv) ตารางเดียวกัน
ผ่าน `tests/unit/test_drive_mixing.py` และ `pio test -e native`

### `io_adapter.py` ต้องจำลอง command timeout ด้วย

`IsaacRover` ต้องดับล้อเมื่อไม่ได้ `drive` ใหม่เกิน `command_timeout_ms`
เหมือน firmware จริง ไม่งั้น scenario `link_lost.yaml` จะผ่านใน sim แต่พฤติกรรม
ไม่ตรงกับของจริง — และ bug ประเภทนี้จะโผล่ที่ V3 ซึ่ง debug แพงกว่ามาก

---

## Sensor Mapping

| Hardware | Simulation |
|---|---|
| Camera front (tilt 50°) | RGB Camera ผูกกับ `base_link` — 640×480 |
| Camera down (top-down) | RGB Camera ผูกกับ `base_link` — 1280×720 |
| DC gear motor × 4 | Continuous joint × 4 (velocity drive) |
| E-stop sense line | Virtual digital input |
| Command timeout (firmware) | `io_adapter.py` ด้วย timestamp ของคำสั่งล่าสุด |

**ไม่มี** endstop · soil contact · servo — ออกจาก MVP ทั้งหมด (กลับมาที่ M3)

`camera_front` ความละเอียดต่ำกว่า `camera_down` โดยเจตนา — front อยู่ใน control loop
ต้องได้ 10 Hz บน Pi และ valley histogram ไม่ต้องการ resolution สูง

---

## Environment Generator

```python
generate_bed(
    bed_length_mm=2000,
    crop_rows=3,
    row_spacing_mm=750,
    crop_spacing_mm=80,
    weed_count=20,
    soil_variation_mm=15,
    row_curvature_mm=40,          # ใหม่
    crop_gap_probability=0.08,    # ใหม่
)
```

### สอง parameter ใหม่สำคัญกว่าที่เหลือทั้งหมด

| Parameter | ถ้าเป็น 0 แล้วเกิดอะไร |
|---|---|
| `row_curvature_mm` | ร่องตรงเป๊ะ → `RowFollower` ผ่านได้ด้วย `gain = 0` เราจะไม่รู้เลยว่ามัน track ได้จริงไหมจนลงแปลง |
| `crop_gap_probability` | ไม่มีต้นหาย → heuristic แยก `row_end`/`row_lost` จะไม่ถูกทดสอบที่จุดที่มันพังจริง |

**ต้องไม่เป็น 0 ในการทดสอบปกติ**

---

## ความเสี่ยง Physics ที่ gantry ไม่มี

Prismatic joint ไม่มี contact dynamics แต่ล้อมีทั้งหมด — **นี่คืองานที่ใช้เวลา
มากที่สุดของ V1 มากกว่า row follower**

1. **Friction** — ต้องพอให้ขับไปข้างหน้าได้ แต่ต้องให้ slip ได้ตอนเลี้ยว skid-steer
   ถ้าตั้งสูงเกินจะเลี้ยวไม่ออกหรือ solver ระเบิด
2. **ล้อลอยบน heightfield** — แรงขับหายเป็นช่วง
3. **Timestep** — contact ที่ความเร็วต่ำบนผิวขรุขระอาจต้อง substep ถี่กว่าเดิม
   กระทบเวลารัน CI

ค่าที่ tune ได้ทั้งหมดอยู่ใน `robots/rover/rover.usd` (override layer) ไม่ใช่ใน URDF

### `omega` ใน sim ก็เป็น nominal เหมือนของจริง

slip ทำให้ `omega` ที่ได้ ≠ `omega` ที่สั่ง — ระบบทนได้เพราะ row follower
ปิดลูปด้วยภาพ แต่ **slip ใน sim ไม่เท่ากับของจริง** จึงโอน gain ข้ามไปไม่ได้
`config/control.yaml` เก็บ gain แยกตาม backend

---

## Run

```bash
./scripts/run_sim.sh
```

---

## Rover Model มาจาก CAD

`rover_base.usd` **generate จาก `cad/urdf/weeding_rover.urdf`** ไม่ได้วาดที่นี่

```bash
./scripts/import_urdf.sh      # เขียนทับ rover_base.usd เสมอ
```

Physics tuning ที่ URDF เก็บไม่ได้ (drive gain, damping, contact offset, friction,
sensor attachment, camera intrinsic) อยู่ใน `rover.usd` ซึ่งเป็น **layer ทับ**
จึงไม่หายตอน regenerate

⚠️ อย่าแก้ `rover_base.usd` โดยตรง — มันถูก gitignored และถูกเขียนทับทุกครั้ง

⚠️ ถ้า `camera_front_tilt_deg` เปลี่ยน **gain ของ row follower ใช้ไม่ได้แล้ว** —
มุมก้มกำหนด lookahead distance ต้อง tune ใหม่ ไม่ใช่แค่ regenerate

ดู [../../cad/README.md](../../cad/README.md#pipeline)
