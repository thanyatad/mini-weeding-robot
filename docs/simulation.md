# Simulation (NVIDIA Isaac Sim)

Path: `sim/isaac/`

จำลอง **แปลงปลูกร่องยาว** ที่มี rover 4 ล้อวิ่งอยู่ในร่องระหว่างแถว

---

## Scope

`sim/isaac/` รับผิดชอบเฉพาะ:

- Physics
- Scene (แปลงดิน, พืชผล, วัชพืช)
- Soil heightfield
- Sensor (2 กล้อง)
- Rover model
- Collision
- Joint (4 wheel joint)
- Environment
- Simulation I/O

**ไม่** รับผิดชอบ:

- Weed detection workflow
- Row following policy
- Business logic
- State machine
- Safety policy หลัก

---

## Scene Graph

```text
World
│
├── Bed                          แปลงร่องยาว
│   ├── SoilSurface              heightfield mesh, rigid
│   ├── BedFrame                 ขอบแปลง ยกสูงกว่า wheel_diameter/2
│   ├── CropRow_1 .. CropRow_3   พืชผลเรียงแถวตามยาว
│   │   └── Crop_001 ..
│   └── Weed_001 .. Weed_M       สุ่มในร่อง / ในแถว
│
├── Rover                        วิ่งในร่อง
│   ├── base_link
│   ├── wheel_fl · wheel_fr · wheel_rl · wheel_rr   continuous joint, velocity drive
│   ├── camera_front             tilt ~45° มองไปข้างหน้า
│   └── camera_down              top-down
│
└── Lighting
```

### Assets

```text
scenes/weeding_bed.usd           scene หลัก
scenes/bed_frame.usd             ขอบแปลง
scenes/soil_heightfield.usd      ผิวดิน (generate ได้จาก seed)
scenes/crops/                    โมเดลพืชผล
scenes/weeds/                    โมเดลวัชพืช
robots/rover/rover_base.usd      GENERATED จาก URDF — gitignored
robots/rover/rover.usd           override layer — commit
```

---

## Bed Geometry

```text
Bed                       2000 × 1000 mm  (ยาว × กว้าง)
Crop rows                 3 แถว ตามยาว
row_spacing               350 mm      กึ่งกลางแถวถึงกึ่งกลางแถว
crop_foliage_half_width   30 mm       ใบยื่นออกจากกึ่งกลางแถวข้างละเท่านี้
clear furrow              290 mm      350 − 2 × 30  ◄── ค่าที่ rover ใช้จริง
crop_spacing              80 mm ตามแถว
Edge margin               150 mm แต่ละข้าง
Furrows                   2 ร่องระหว่างแถว — MVP วิ่งร่องเดียว
```

**`row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้** — invariant ทุกข้อที่เกี่ยวกับ
ช่องว่างด้านข้างใช้ `clear_furrow` ไม่ใช่ `row_spacing`

`row_spacing = 350 mm` มาจาก invariant ไม่ใช่จากการเลือก:

```text
clear_furrow > body_width + 2 × runaway_budget

ที่ 250 mm:  190 > 266   ไม่ผ่าน
ที่ 300 mm:  240 > 266   ไม่ผ่าน
ที่ 350 mm:  290 > 266   ผ่าน  margin 24 mm
```

**BedFrame ขอบยกสูงกว่าล้อ ไม่ใช่ของประดับ** — MVP ไม่มี bumper switch
ขอบยกเป็น compensating control ทางกายภาพ ต้องมีทั้งใน sim และของจริง

---

## Soil Model

ดินเป็น **rigid heightfield mesh — ไม่ยุบตัว**

```yaml
soil:
  variation_mm: 15        # ความไม่เรียบ ±
  resolution_mm: 10       # ความละเอียด heightfield
```

| ได้ | ไม่ได้ |
|---|---|
| Collision ล้อกับผิวไม่เรียบ | ดินยุบ / ล้อจม |
| CV background เป็นดินจริง | แรงต้านของดิน |
| Physics stable, รัน headless CI ได้ | ร่องล้อที่ทิ้งไว้ |

Deformable soil แพงเกินและ tune ยากสำหรับ POC

---

## Rover Model

Skid-steer 4WD — 4 ล้อขับ ไม่มีพวงมาลัย เลี้ยวด้วยความต่างความเร็วซ้าย/ขวา

```text
base_link
 ├── wheel_fl   continuous joint  (velocity drive)
 ├── wheel_fr   continuous joint
 ├── wheel_rl   continuous joint
 ├── wheel_rr   continuous joint
 ├── camera_front   fixed, tilt ~45°
 └── camera_down    fixed, top-down
```

```text
track_width          120 mm
wheelbase            120 mm
body_width           146 mm   ◄── รวมล้อ (track 120 + wheel_width 26)
wheel_diameter        65 mm
chassis_clearance     35 mm
```

**ไม่มี suspension** — chassis แข็ง 4 ล้อ บนผิว ±15 mm จะมีจังหวะล้อลอยและเสียแรงขับ
นี่ไม่ใช่ข้อบกพร่องของ sim ที่ต้องแก้ ถ้าเกิดใน sim มันจะเกิดของจริงด้วย —
ให้ sim บอกก่อนซื้อล้อ

### Geometry invariants ที่ผูกกับ sim

```text
wheel_diameter    >= 4 × soil_variation     65 >= 60    ล้อเล็กเกินจะสะดุดทุกก้อนดิน
chassis_clearance >  soil_variation         35 >  15    ท้องไม่ครูดยอดดิน
```

---

## Skid-Steer Mixing

```text
omega_rad   = omega_deg × pi / 180
v_left      = v - omega_rad × track_width_mm / 2
v_right     = v + omega_rad × track_width_mm / 2
wheel_rad_s = v_side / (wheel_diameter_mm / 2)
```

ล้อหน้าและหลังข้างเดียวกันได้คำสั่ง**เหมือนกันเสมอ**

### สูตรนี้ถูก implement สองที่ที่แชร์โค้ดกันไม่ได้

Python ใน `IsaacRover` และ C++ ใน ESP32 firmware ถ้าปล่อยไว้ ทั้งสองจะค่อย ๆ
เพี้ยนจากกัน แล้ว gain ที่ tune ใน sim จะใช้กับของจริงไม่ได้โดยไม่มีใครรู้ว่าทำไม

```text
config/drive_mixing_vectors.csv    ◄── source of truth เดียว
        │
        ├──> tests/unit/test_drive_mixing.py          (Python / IsaacRover)
        └──> firmware/esp32/test/test_mixing.cpp      (PlatformIO native env)
```

ไม่มี code generator — ตารางค่าเป็นสัญญาที่ทั้งสองภาษาอ่านได้

### Saturation ต้อง scale ไม่ใช่ clip

ถ้าล้อข้างหนึ่งเกิน `wheel_v_max_mm_s` ให้ลด **ทั้งสองข้างตามอัตราส่วนเดิม**
การ clip ข้างเดียวจะเปลี่ยน `omega` ที่ได้จริงโดยเงียบ

### `omega` เป็น nominal ไม่ใช่ค่าจริง

skid-steer จริงมี slip ทำให้ `omega` ที่ได้ ≠ `omega` ที่สั่ง ระบบทนได้เพราะ
row follower ปิดลูปด้วยภาพ — มันแก้ความคลาดของ `omega` โดยอัตโนมัติ

แต่หมายความว่า **gain ต้อง tune ใหม่บน hardware** เพราะ slip บนดินจริงกับใน sim
ไม่เท่ากัน — `config/control.yaml` เก็บ gain แยกตาม backend

---

## Sensor Mapping

| Hardware | Simulation |
|---|---|
| Camera front (tilt 45°) | RGB Camera ผูกกับ `base_link` |
| Camera down (top-down) | RGB Camera ผูกกับ `base_link` |
| DC gear motor × 4 | Continuous joint × 4 (velocity drive) |
| E-stop sense line | Virtual Digital Input |
| Command timeout (firmware) | จำลองใน `IsaacRover` ด้วย timestamp ของคำสั่งล่าสุด |

**ไม่มี** endstop · soil contact sensor · servo — ออกจาก MVP ทั้งหมด

---

## Environment Generator

Path: `sim/isaac/environments/`

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

Random ได้:

- Soil heightfield seed, ความขรุขระ
- Texture / สีดิน, ความชื้น (เข้ม/อ่อน)
- ตำแหน่ง / ขนาด / rotation ของวัชพืช
- Jitter ตำแหน่งพืชผลในแถว
- มุมเอียงของแถวทั้งชุด
- **ความโค้งของร่อง** (`row_curvature_mm`)
- **ช่องว่างในแถว** (`crop_gap_probability`)
- Lighting, เงา
- Camera noise

### สอง parameter ใหม่สำคัญกว่าที่เหลือทั้งหมด

| Parameter | ป้องกันอะไร |
|---|---|
| `row_curvature_mm` | ถ้าร่องตรงเป๊ะทุก run `RowFollower` จะผ่านด้วย `gain = 0` ก็ได้ เราจะไม่รู้เลยว่ามัน track ได้จริงไหมจนลงแปลง |
| `crop_gap_probability` | ต้นหายกลางแถวคือกรณีที่ `green_fraction` ตกชั่วคราว — heuristic แยก `row_end`/`row_lost` จะพังที่นี่ ต้องเจอใน sim |

---

## Domain Randomization

เป้าหมายคือให้ระบบไม่จำ scene เดิม

```text
Run #1                          Run #2

║ 🌱  🌱  🌱  🌱 ║              ║ 🌱  🌱   ·  🌱 ║   ← ต้นหาย (crop gap)
║               ║              ║      ╲        ║
║   [rover]     ║              ║   [rover]     ║   ← ร่องโค้ง
║               ║              ║        ╲      ║
║ 🌱  🌱  🌱  🌱 ║              ║ 🌱  🌱  🌱  🌱 ║
```

`row_curvature_mm` และ `crop_gap_probability` ต้องไม่เป็น 0 ในการทดสอบปกติ —
ถ้าร่องตรงและแถวเต็มทุก run ทั้ง `row_estimator.py` และ heuristic `row_end`
จะ overfit และใช้กับแปลงจริงไม่ได้

---

## ความเสี่ยง Physics ที่ gantry ไม่มี

Prismatic joint ไม่มี contact dynamics แต่ล้อมีทั้งหมด **ต้อง budget เวลา tune
มากกว่างานอื่นทั้งหมดใน V1**

1. **Friction** — ต้องพอให้ขับไปข้างหน้าได้ แต่ต้องให้ slip ได้ตอนเลี้ยว skid-steer
   ถ้าตั้งสูงเกินจะเลี้ยวไม่ออกหรือ solver ระเบิด
2. **ล้อลอยบน heightfield** — แรงขับหายเป็นช่วง
3. **Timestep** — contact ที่ความเร็วต่ำบนผิวขรุขระอาจต้อง substep ถี่กว่าเดิม
   กระทบเวลารัน CI

ค่าที่ tune ได้ทั้งหมดอยู่ใน `robots/rover/rover.usd` (override layer)
ไม่ใช่ใน URDF — ดู [../cad/README.md](../cad/README.md#ทำไมต้องแยก-2-usd-layer)

---

## Config

See `config/simulation.yaml`

```yaml
simulation:

  physics_hz: 60
  render_hz: 30

  camera:
    front: { width: 640, height: 480 }
    down:  { width: 1280, height: 720 }

  soil:
    variation_mm: 15
    resolution_mm: 10

  environment:
    bed_length_mm: 2000
    crop_rows: 3
    crop_spacing_mm: 80
    row_spacing_mm: 350
    weed_count: 20

  randomization:
    enabled: true
    row_jitter_mm: 15
    row_angle_deg: 3
    row_curvature_mm: 40
    crop_gap_probability: 0.08
```

`soil.variation_mm` ต้องตรงกับ `bed.soil_variation_mm` ใน `rover.yaml` —
ถ้าไม่ตรง simulation จะสร้างดินที่ขัดกับ geometry invariant โดยที่ invariant check ผ่าน
ข้อนี้เป็น **invariant ข้อที่ 9** ที่ตรวจตอน startup ไม่ใช่แค่คำแนะนำ

`camera.front` ความละเอียดต่ำกว่า `camera.down` โดยเจตนา — front อยู่ใน control loop
ต้องได้ 10 Hz บน Pi และ valley histogram ไม่ต้องการ resolution สูง
