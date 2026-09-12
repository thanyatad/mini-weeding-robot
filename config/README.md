# config

> อย่า hard-code parameter ใน source code

---

## Files

| File | Scope |
|---|---|
| `rover.yaml` | Rover geometry (derived: cad), drive limit, bed geometry |
| `safety.yaml` | Runaway budget, timeout, row-loss frames, E-stop |
| `perception.yaml` | ExG floor, confidence, loop rate, corridor, weed area |
| `control.yaml` | Row follower gain — **แยกตาม backend** |
| `simulation.yaml` | Physics/render rate, camera, soil, bed generator, randomization |
| `camera.yaml` | Front (ไม่ calibrate) + down (intrinsic, homography, **error budget**) |
| `development.yaml` | Backend selection, logging |
| `drive_mixing_vectors.csv` | Golden vector ของ skid-steer mixing (ใช้ร่วมกับ firmware) |

---

## rover.yaml

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

### `omega_max_deg_s` เป็น 40 ไม่ใช่ 60 — เพราะ deadband ของมอเตอร์ DC

```text
ล้อ 65 mm → เส้นรอบวง 204 mm
มอเตอร์ 12 V ~100 RPM → ความเร็วล้อสูงสุด 340 mm/s
มอเตอร์เกียร์ DC ไม่ออกตัวใต้ ~15% duty ≈ 51 mm/s

ที่ omega_max = 60:  differential = 62.8 mm/s → v_left = 37.2 mm/s  (11%)  ✗ deadband
ที่ omega_max = 40:  differential = 41.9 mm/s → v_left = 58.1 mm/s  (17%)  ✓ เหนือ deadband

เพดานจริงของ `omega_max` คือ **46 deg/s** (ที่ 46.8 ค่า `v_left` จะแตะ 51 พอดี) —
เลือก 40 เพื่อให้มี margin ไม่ใช่เพราะ 40 เป็นค่าสูงสุดที่ทำได้
```

ถ้าล้อข้างในอยู่ใน deadband มันจะ**หยุดหมุนขณะที่ ESP32 คิดว่ากำลังสั่งให้หมุน** —
rover จะเลี้ยวแรงกว่าที่สั่งโดยไม่มีสัญญาณบอก

### `row_spacing_mm` ไม่ใช่ความกว้างที่ rover วิ่งได้

```text
clear_furrow = row_spacing − 2 × crop_foliage_half_width = 350 − 60 = 290 mm
```

invariant ทุกข้อที่เกี่ยวกับช่องว่างด้านข้างใช้ `clear_furrow` **ไม่ใช่** `row_spacing`

---

## safety.yaml

```yaml
safety:
  runaway_budget_mm: 60
  row_loss_frames: 3
  command_timeout_ms: 300
  link_lost_ms: 500
  enable_estop: true
```

`runaway_budget_mm` คือ **ระยะที่ยอมให้ rover วิ่งต่อหลังเสียการควบคุม**
MVP ไม่มี bumper switch ค่านี้จึงเป็นตัวชดเชยหลัก
ต้องเล็กกว่าระยะจากกันชนหน้า rover ถึงขอบแปลงและถึงต้นพืชที่ใกล้สุด —
**ต้องวัดจากแปลงจริงแล้วบันทึก** ไม่ใช่เดา

`link_lost_ms (500) > command_timeout_ms (300)` **โดยเจตนา** — ESP32 ต้องดับมอเตอร์
ก่อนที่ Pi จะประกาศ error เพื่อให้ลำดับเหตุการณ์ใน log อ่านได้ว่าอะไรเกิดก่อน
ถ้ากลับกัน Pi จะเข้า `ERROR` ขณะที่ล้อยังหมุนอยู่ 200 ms

---

## perception.yaml

```yaml
perception:
  loop_hz: 10                  # front camera / control loop
  camera_timeout_ms: 300

  exg:
    exg_floor: 12              # ต้อง tune ด้วยมือ — ดูหมายเหตุ

  row_estimator:
    conf_min: 0.25             # valley prominence ขั้นต่ำ
    front_downscale: [320, 240]

  weed_detector:
    max_lateral_error_mm: 40   # สมมติฐาน — ต้องวัดที่ V1
    min_weed_area_mm2: 60
    down_rate_hz: 2
```

### `exg_floor` เป็นค่าเดียวใน pipeline ที่ต้อง tune ด้วยมือ

ExG threshold ใช้ **Otsu ต่อเฟรม** ไม่ใช่ค่าคงที่ เพราะสีดินเปลี่ยนตามความชื้น

แต่ Otsu มีจุดตาย: **เมื่อไม่มีสีเขียวในเฟรมเลย มันจะแบ่ง noise ออกเป็นสองกอง**
แล้วคืน mask ที่ดูเหมือนมีพืช ทำให้ `row_end` ตรวจไม่เจอ

`exg_floor` เป็นพื้นที่กันข้อนี้ — ต้องมี unit test ป้อนภาพดินเปล่า
ยืนยันว่าได้ mask ว่าง

### `max_lateral_error_mm` กำหนดความกว้าง corridor

```text
corridor = clear_furrow − 2 × max_lateral_error_mm = 290 − 80 = 210 mm
```

เป็นแถบที่รับประกันว่าอยู่ในร่องแม้ rover เบี่ยงเต็มพิสัย — สีเขียวในแถบนี้
จึงเป็นวัชพืชได้โดยไม่ต้องรู้ว่าแถวอยู่ไหน

`40` เป็น **สมมติฐาน** ต้องวัดค่าจริงที่ V1 ถ้าเกิน corridor จะแคบลงและ recall ตก

---

## control.yaml

```yaml
row_follower:
  v_mm_s: 100
  omega_max_deg_s: 40

  gains:
    fake:  {k_lat: 45.0, k_head: 25.0}
    isaac: {k_lat: 45.0, k_head: 25.0}
    esp32: {k_lat: null, k_head: null}    # ยังไม่ tune บนของจริง
```

### `null` เป็น guard ไม่ใช่ค่าที่ถูกลืม

ถ้า `backend: esp32` แล้ว gain เป็น `null` → fail ตอน startup ด้วย `config_invalid`

เหตุผล: slip ของ skid-steer บนดินจริงไม่เท่ากับใน sim การปล่อยให้ค่า sim
ถูกใช้เป็น default บนของจริงคือ rover ที่แกว่งเข้าหาต้นพืชโดยไม่มีใครรู้ว่าทำไม
**การบังคับให้ประกาศว่า "ยังไม่ได้ tune" ถูกกว่าการค้นหาสาเหตุในแปลง**

### ⚠️ gain ผูกกับ `camera_front_tilt_deg`

มุมก้มของกล้องหน้ากำหนดว่า `valley_far` อยู่ห่างไปข้างหน้าเท่าไร ซึ่งคือ
**lookahead distance** ถ้าขยับขายึดกล้อง gain ที่ tune ไว้ใช้ไม่ได้

นี่เป็น coupling ระหว่าง CAD กับ control ที่ gantry ไม่มี (กล้อง gantry มองตรงลง
มุมไม่มีผลต่อ control) — ดู [../cad/parameters/README.md](../cad/parameters/README.md)

### หน่วยของ gain

`RowEstimate` ไม่มีหน่วย mm หรือ deg (กล้องหน้าไม่ calibrate) gain จึงมีหน่วย
`(deg/s) / unitless` และ **ต้อง tune** ไม่มีสูตรคำนวณจากเรขาคณิต

---

## simulation.yaml

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
ข้อนี้เป็น **invariant ข้อที่ 9** ที่บังคับตอน startup ไม่ใช่แค่คำแนะนำ

`row_curvature_mm` และ `crop_gap_probability` **ต้องไม่เป็น 0** ในการทดสอบปกติ:

| Parameter = 0 แล้วเกิดอะไร |
|---|
| `row_curvature_mm: 0` → ร่องตรงเป๊ะ `RowFollower` ผ่านได้ด้วย `gain = 0` เราจะไม่รู้เลยว่ามัน track ได้จริงไหม |
| `crop_gap_probability: 0` → ไม่มีต้นหาย heuristic แยก `row_end`/`row_lost` จะไม่ถูกทดสอบที่จุดที่มันพังจริง |

---

## camera.yaml

```yaml
cameras:

  front:
    calibrated: false          # โดยเจตนา — row following ใช้แค่มุม/offset ในภาพ
    device: /dev/video0

  down:
    device: /dev/video1
    intrinsic: { fx: null, fy: null, cx: null, cy: null }
    distortion: null
    homography: null           # เติมจาก tools/calibration/
    error_budget_mm: 20        # target — ต้องวัดจริงแล้วบันทึกค่าที่วัดได้
```

### กล้องหน้าไม่ calibrate โดยเจตนา

`RowEstimate` เป็นค่า image-space ล้วน (`lateral_err`, `heading_err` เป็น ratio
ใน `[-1, 1]`) และ `heading_err` คำนวณจากผลต่างของ valley สองระยะ
ซึ่ง **ไม่ต้องรู้ geometry กล้องเลย**

ผลคืองาน calibration ของ MVP เบากว่า design gantry — เหลือกล้องเดียว

### `error_budget_mm: 20` ผ่อนจาก 8 ของ design gantry

ผ่อนได้เพราะ **MVP ไม่มีอะไรกระทำต่อตำแหน่ง** — ไม่มี tool ที่ต้องเล็ง
ตัวเลข mm ใช้แค่บันทึกขนาด/ตำแหน่งคร่าว ๆ ใน log

แต่ยัง **ต้องทำ** homography ตอนนี้ ไม่เลื่อนทั้งหมด — เพื่อรู้ตัวเลข error budget จริง
บนดิน ±15 mm **ก่อน** ออกแบบ tool ที่ M3 ไม่ใช่หลังจากนั้น

---

## development.yaml

```yaml
backend: isaac

logging:
  level: INFO
  weed_log_path: logs/weeds.jsonl
```

---

## Backend Switching

เปลี่ยน backend โดยไม่แก้ code:

| Use case | Setting |
|---|---|
| Unit test + closed-loop row following (ไม่ต้องเปิด Isaac) | `backend: fake` |
| Simulation | `backend: isaac` |
| Hardware | `backend: esp32` |

---

## Startup Invariants

ตรวจที่ `controller/startup_checks.py` — **ทุกข้อ fail = ไม่ start**
พร้อมบอกค่าที่ขัดกันเป็นตัวเลข ไม่ใช่ warning

```text
safety      v_max × command_timeout/1000        <= runaway_budget        30 <= 60   ok
safety      v_max × row_loss_frames/loop_hz     <= runaway_budget        30 <= 60   ok
geometry    wheel_diameter        >= 4 × soil_variation                  65 >= 60   ok
geometry    chassis_clearance     >  soil_variation                      35 >  15   ok
geometry    clear_furrow          >  body_width + 2 × runaway_budget    290 > 266   ok
drive       v + omega_max_rad × track/2  <= wheel_v_max               141.9 <= 340  ok
drive       v − omega_max_rad × track/2  >= wheel_v_min                58.1 >=  51
config      gains[backend] ไม่เป็น null
consistency simulation.soil.variation_mm == bed.soil_variation_mm

โดย clear_furrow = row_spacing − 2 × crop_foliage_half_width
```

### ข้อ `drive` ตัวล่างมี margin แค่ 7.1 mm/s

`wheel_v_min_mm_s = 51` เป็นค่าประมาณจาก 15% duty **ยังไม่ได้วัดจากมอเตอร์จริง**

ถ้าวัดจริงที่ V3 ได้เกิน **58 mm/s** → **ต้องลด `omega_max` ลงอีก**
**ห้ามลด `wheel_v_min` เพื่อให้ผ่าน** — ค่านั้นเป็นคุณสมบัติของมอเตอร์ ไม่ใช่ค่าที่เราเลือก

### ตัวเลขที่ invariant จับได้ตอนเขียน spec

สามชุดนี้ถูกแก้เพราะลองคูณเลขแล้วขัดกันเอง ไม่ใช่เพราะคิดว่าจะตั้งผิด:

| แก้อะไร | จาก → เป็น | invariant ที่จับได้ |
|---|---|---|
| `row_spacing_mm` | 250 → 350 | `clear_furrow > body_width + 2 × runaway_budget` |
| `omega_max_deg_s` | 60 → 40 | `v − omega_max_rad × track/2 >= wheel_v_min` |
| นิยามข้อ 5 | `row_spacing` → `clear_furrow` | ใบพืชกินร่องข้างละ 30 mm ที่สูตรเดิมมองไม่เห็น |
| `track_width_mm` | 140 → 120 | `body_width` ต้องเป็นความกว้าง**รวมล้อ** (`track + wheel_width`) ไม่ใช่แค่แชสซี — ที่ 140 เหลือ margin แค่ 4 mm |

---

## Derived Values

ค่าที่มี comment `# derived: cad ...` **ห้ามแก้เพื่อให้ test ผ่าน** —
ต้นทางคือ `cad/parameters/parameters.csv` ถ้า test fail แปลว่า CAD เปลี่ยน
แล้ว config ยังตามไม่ทัน

```bash
pytest tests/unit/test_cad_config_sync.py
```

ตารางความสัมพันธ์ทั้งหมด: [../cad/parameters/README.md](../cad/parameters/README.md#derived-values--ค่าที่ไหลจาก-cad-ไป-config)

เส้นแบ่งความเป็นเจ้าของ:

| | เจ้าของ |
|---|---|
| วัดด้วยเวอร์เนียร์ได้ (track width, ล้อ, chassis, ขายึดกล้อง) | **CAD** |
| เปลี่ยนได้โดยไม่ต้องถอดสกรู (v_max, timeout, gain, threshold) | **config** |

### ค่าที่ต้อง **วัดจริง** แล้วเขียนกลับ

ค่าเหล่านี้ตั้งไว้เป็นค่าประมาณที่สอดคล้องกัน **ไม่ใช่ค่าที่วัดแล้ว**

| ค่า | วัดที่ | ผลถ้าผิด |
|---|---|---|
| `crop_foliage_half_width_mm` | V1 (asset พืช) | กระทบ invariant ข้อ 5 **และ** corridor พร้อมกัน |
| `max_lateral_error_mm` | V1 | corridor แคบ/กว้างผิด recall เพี้ยน |
| `wheel_v_min_mm_s` | V3 (มอเตอร์จริง) | `omega_max` สูงเกิน ล้อข้างในหยุดเงียบ |
| `runaway_budget_mm` | V4 (แปลงจริง) | ค่าความปลอดภัยหลักของ MVP |
| `error_budget_mm` | V4 (calibrate) | ตัวเลขใน weed log ไม่มีความหมาย |
| `gains.esp32` | V4 (tune) | ค้างเป็น `null` = ไม่ start (ตั้งใจ) |
