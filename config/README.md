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
  track_width_mm: 430          # derived: cad track_width
  wheelbase_mm: 400            # derived: cad wheelbase
  body_width_mm: 520           # derived: cad body_width  (รวมล้อ = track + wheel_width)
  wheel_diameter_mm: 250       # derived: cad wheel_diameter
  chassis_clearance_mm: 125    # derived: cad chassis_clearance

  drive:
    v_max_mm_s: 160
    omega_max_deg_s: 25
    wheel_v_max_mm_s: 327      # derived: cad wheel_diameter + มอเตอร์ 25 RPM (poc-v3.md)
    wheel_v_min_mm_s: 51       # deadband — ต้องวัดจริงที่ V3

bed:
  soil_variation_mm: 15
  row_spacing_mm: 750
  crop_foliage_half_width_mm: 30    # ประมาณ — ต้องวัดจาก asset พืชที่ V1
```

### `omega_max_deg_s` เป็น 25 — เลือกจากสมรรถนะการเลี้ยว ไม่ใช่จาก deadband โดยตรง

`omega_max = 25 deg/s` หมายถึงหมุน 180° ใน 7.2 วินาที ซึ่งเหมาะกับเครื่อง 35 kg —
skid steer ที่ track 430 mm บนยาง Ø250 เสียดสีด้านข้างกินแรงบิดจริง (design §6.1)
เลือกก่อนแล้วจึงบีบหน้าต่างความเร็วมอเตอร์ ไม่ใช่ไล่หาจากมอเตอร์ก่อน

```text
`omega_max_deg_s: 25` ไม่ใช่ค่าที่เลือกเพราะชอบ — differential ของล้อเป็นสัดส่วนกับ
track ซึ่งโตจาก 120 เป็น 430 (3.6 เท่า) ที่ 40 deg/s เดิม ล้อข้างในจะได้
160 - 150 = 10 mm/s ซึ่งอยู่ใต้ deadband: ล้อหยุดนิ่งขณะที่ ESP32 ยังคิดว่ากำลังขับ
และรถเลี้ยวแรงกว่าที่สั่งโดยไม่มีอะไรส่งสัญญาณ

ที่ omega_max = 25:  differential = 93.8 mm/s → v_left = 160 - 93.8 = 66.2 mm/s  ✓ เหนือ 51 (margin 15.2)

เพดานจริงของ `omega_max` ที่ track 430 คือ **~29.0 deg/s** (ที่ 29.05° ค่า `v_left`
จะแตะ 51 พอดี) — เลือก 25 เพื่อให้มี margin ไม่ใช่เพราะ 25 เป็นค่าสูงสุดที่ทำได้
```

`v_max_mm_s: 160` เพดานจาก runaway budget คือ 200 (60 mm / 0.3 s) เลือก 160
เพราะมันดัน margin ของ invariant ข้อ 7 จาก 5.2 เป็น 15.2 mm/s ซึ่งเป็นข้อเดียว
ที่พึ่งค่าที่ยังไม่ได้วัด

`wheel_v_min_mm_s: 51` ถูกเก็บไว้เท่าเดิมโดยเจตนาแม้เปลี่ยนมอเตอร์ทั้งชั้น — 15%
duty ของ 327 ให้ 49 แต่ค่าที่เข้มกว่าคือค่าที่เก็บ เหมือนที่ poc-v2 ทำไว้ เกียร์
planetary มีชั้นเฟืองมาก stiction สูง deadband จริงอาจสูงกว่า 15% ไม่ใช่ต่ำกว่า
เก็บ 51 ไว้จนกว่าจะวัดจริงที่ V3

ถ้าล้อข้างในอยู่ใน deadband มันจะ**หยุดหมุนขณะที่ ESP32 คิดว่ากำลังสั่งให้หมุน** —
rover จะเลี้ยวแรงกว่าที่สั่งโดยไม่มีสัญญาณบอก

### `row_spacing_mm` ไม่ใช่ความกว้างที่ rover วิ่งได้

```text
clear_furrow = row_spacing − 2 × crop_foliage_half_width = 750 − 60 = 690 mm
```

invariant ทุกข้อที่เกี่ยวกับช่องว่างด้านข้างใช้ `clear_furrow` **ไม่ใช่** `row_spacing`

### `crop_foliage_half_width_mm` คือ margin ของ invariant ข้อ 5 ทั้งก้อน

```text
row_spacing 750 ผ่านตราบใดที่ crop_foliage_half_width < 55 mm
ถ้า V1 วัดได้ 55+ → row_spacing ต้องเป็น 800+

นี่ไม่ใช่การแก้ config — เป็นข้อจำกัดว่าแปลงต้องปลูกห่างเท่าไร
```

รถกว้างรวมล้อ 520 mm บังคับว่าแถวต้องห่างเท่าไร แถว 750 mm ที่ใบพืชกว้างข้างละ
30 mm แปลว่าพืชระยะต้นอ่อน ซึ่งเป็นช่วงที่กำจัดวัชพืชอยู่แล้ว จึงสอดคล้องกัน
แต่ margin ทั้งหมดของ invariant ข้อ 5 (50 mm) ถูกใช้จ่ายไปกับค่านี้ที่ยังไม่ได้วัด

---

## safety.yaml

```yaml
safety:
  runaway_budget_mm: 60
  row_loss_frames: 3
  command_timeout_ms: 300
  link_lost_ms: 500
  ack_timeout_ms: 200
```

**ไม่มี `enable_estop`** โดยเจตนา — E-stop ตัด motor rail ทางไฟ (§11.4)
config key ปิดรีเลย์ไม่ได้ `false` จึงแปลได้อย่างเดียวว่า "ให้ software เมินสาย sense"
ซึ่งคือ rover ที่หยุดไปแล้วแต่ state machine ยังคิดว่ากำลังวิ่งอยู่

§5.6 บอกว่าแต่ละชั้นต้องทำงานได้โดยไม่พึ่งชั้นบน — ชั้น software จึงไม่ใช่ชั้นที่
ปิดแล้วยังเรียกว่า safe ได้ สาย sense อ่านโดย `controller/safety/emergency_stop.py` เสมอ

`runaway_budget_mm` คือ **ระยะที่ยอมให้ rover วิ่งต่อหลังเสียการควบคุม**
MVP ไม่มี bumper switch ค่านี้จึงเป็นตัวชดเชยหลัก
ต้องเล็กกว่าระยะจากกันชนหน้า rover ถึงขอบแปลงและถึงต้นพืชที่ใกล้สุด —
**ต้องวัดจากแปลงจริงแล้วบันทึก** ไม่ใช่เดา

`link_lost_ms (500) > command_timeout_ms (300)` **โดยเจตนา** — ESP32 ต้องดับมอเตอร์
ก่อนที่ Pi จะประกาศ error เพื่อให้ลำดับเหตุการณ์ใน log อ่านได้ว่าอะไรเกิดก่อน
ถ้ากลับกัน Pi จะเข้า `ERROR` ขณะที่ล้อยังหมุนอยู่ 200 ms

`ack_timeout_ms` คือ deadline ของ Rule 4 ใน [protocol/messages.md](../protocol/messages.md) —
discrete command (`stop` · `emergency_stop` · `reset`) ที่ไม่ได้ `ack` ภายในเวลานี้
→ `ERROR` code `communication_lost` พร้อม `id` ของ command นั้น

**ไม่ใช้กับ `drive`** ซึ่งเป็น streaming — เฟรมที่ตกหนึ่งเฟรมต้องไม่กลายเป็น error

เป็นคนละเรื่องกับอีกสองค่าและต้องแยกกัน: `command_timeout_ms` คือ deadline ที่ **ESP32**
รอ `drive` ตัวถัดไป · `link_lost_ms` คือเพดานของ `link_age_ms` ฝั่ง **Pi**
ทั้งสามเรียงกันเป็น 200 < 300 < 500 เพื่อให้ลำดับใน log อ่านได้ว่าอะไรเกิดก่อน

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
    green_fraction_row_end: 0.10   # สมมติฐาน — ต้องวัดที่ V1
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

### `green_fraction_row_end` แยก `row_end` ออกจาก `row_lost`

ทั้งสองกรณีมาถึงเหมือนกันเป๊ะ — estimate invalid ติดกัน `row_loss_frames` เฟรม
ตัวแยกมีตัวเดียวคือ `green_fraction` ของเฟรมที่ทำให้ watchdog trip (§5.4):

```text
green_fraction <  0.10    เขียวหมดจากเฟรม      สุดร่อง   → STOPPED(row_end_suspected)
green_fraction >= 0.10    ยังเห็นเขียวแต่ไม่มีเส้น  nav พลาด  → ERROR(row_lost)
```

อยู่ใน `perception.yaml` ไม่ใช่ `safety.yaml` เพราะมันเป็นสมบัติของ **mask ที่ ExG ผลิต**
ไม่ใช่ของ watchdog — ถ้า `exg_floor` เปลี่ยน หรือ asset พืชเปลี่ยน ค่านี้ต้องขยับตาม
การวางไว้ข้าง `exg_floor` ทำให้คนที่ tune ExG เห็นว่ามีค่านี้ผูกอยู่

⚠️ **`0.10` ยังไม่ถูก validate** — เป็นสมมติฐานแบบเดียวกับ `max_lateral_error_mm`
V0 พิสูจน์ได้แค่ว่า *ตรรกะ* เปรียบเทียบถูกและ watchdog นับถูก เพราะ harness
ไม่มี pixel และไม่มีต้นพืช การยืนยันว่าเส้นนี้แยกช่องว่างกลางแถวออกจากสุดร่อง
ได้จริงต้องใช้ภาพ render — คือ `crop_gap_midrow.yaml` กับ `row_end.yaml` ที่ V1
**ต้องผ่านทั้งคู่** ผ่านข้างเดียวแปลว่าตั้งเอาใจข้างเดียว

กรณีเท่ากับ threshold พอดีนับเป็น `row_lost` — nav ที่พลาดแล้วถูกบันทึกว่าจบร่องปกติ
คือทิศที่กลบปัญหา ส่วนทิศกลับกันแค่เรียกคนมาดู

### `max_lateral_error_mm` กำหนดความกว้าง corridor

```text
corridor = clear_furrow − 2 × max_lateral_error_mm = 690 − 80 = 610 mm
```

เป็นแถบที่รับประกันว่าอยู่ในร่องแม้ rover เบี่ยงเต็มพิสัย — สีเขียวในแถบนี้
จึงเป็นวัชพืชได้โดยไม่ต้องรู้ว่าแถวอยู่ไหน

`40` เป็น **สมมติฐาน** ต้องวัดค่าจริงที่ V1 ถ้าเกิน corridor จะแคบลงและ recall ตก

---

## control.yaml

```yaml
row_follower:
  v_mm_s: 160
  omega_max_deg_s: 25

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
    row_spacing_mm: 750
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
safety      v_max × command_timeout/1000        <= runaway_budget         48 <= 60   ok
safety      v_max × row_loss_frames/loop_hz     <= runaway_budget         48 <= 60   ok
geometry    wheel_diameter        >= 4 × soil_variation                  250 >= 60   ok
geometry    chassis_clearance     >  soil_variation                      125 >  15   ok
geometry    clear_furrow          >  body_width + 2 × runaway_budget     690 > 640   ok
drive       v + omega_max_rad × track/2  <= wheel_v_max               253.8 <= 327  ok
drive       v − omega_max_rad × track/2  >= wheel_v_min                66.2 >=  51
config      gains[backend] ไม่เป็น null
consistency simulation.soil.variation_mm == bed.soil_variation_mm

โดย clear_furrow = row_spacing − 2 × crop_foliage_half_width
```

ข้อ geometry ทั้งสองข้อเคยเป็นข้อที่คับที่สุดของเครื่องเดิม — ล้อ Ø250 และ
ท้องรถ 125 mm ปลดทั้งคู่ทิ้ง (margin 190 mm และ 110 mm ตามลำดับ)
**`clear_furrow > body_width + 2 × runaway_budget` กลายเป็นข้อที่คับที่สุดแทน**
(margin 50 mm) — ดู [§`crop_foliage_half_width_mm`](#crop_foliage_half_width_mm-คือ-margin-ของ-invariant-ข้อ-5-ทั้งก้อน)

### ข้อ `drive` ตัวล่างมี margin 15.2 mm/s

`wheel_v_min_mm_s = 51` เป็นค่าประมาณจาก 15% duty **ยังไม่ได้วัดจากมอเตอร์จริง**

ถ้าวัดจริงที่ V3 ได้เกิน **66.2 mm/s** → **ต้องลด `omega_max` ลงอีก**
**ห้ามลด `wheel_v_min` เพื่อให้ผ่าน** — ค่านั้นเป็นคุณสมบัติของมอเตอร์ ไม่ใช่ค่าที่เราเลือก

### ตัวเลขที่ invariant จับได้ตอนเขียน spec

ชุดนี้ถูกแก้เพราะลองคูณเลขแล้วขัดกันเอง ไม่ใช่เพราะคิดว่าจะตั้งผิด:

| แก้อะไร | จาก → เป็น | invariant ที่จับได้ |
|---|---|---|
| `row_spacing_mm` | 250 → 350 → **750** | `clear_furrow > body_width + 2 × runaway_budget` — 350 มาจากรอบ MVP เดิม (rover 145 mm), 750 มาจากรอบ scale-up นี้ (rover 520 mm) |
| `omega_max_deg_s` | 60 → 40 | `v − omega_max_rad × track/2 >= wheel_v_min` — ที่รอบ MVP เดิม (`track_width 120`) ปัจจุบันคือ **25** ที่ `track_width 430` (ดูเหตุผลข้างบน) |
| นิยามข้อ 5 | `row_spacing` → `clear_furrow` | ใบพืชกินร่องข้างละ 30 mm ที่สูตรเดิมมองไม่เห็น |
| `track_width_mm` | 140 → 120 | `body_width` ต้องเป็นความกว้าง**รวมล้อ** (`track + wheel_width`) ไม่ใช่แค่แชสซี — ที่ 140 เหลือ margin แค่ 4 mm (ประวัติของเครื่อง 145 mm เดิม ก่อนรอบ scale-up นี้ที่ `track_width` เป็น 430) |

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
