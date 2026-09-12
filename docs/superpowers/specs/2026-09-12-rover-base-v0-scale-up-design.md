# Design: Rover Base V0 — Scale-up เป็น 650 × 520 mm, ล้อ Ø250

**Date:** 2026-09-12
**Status:** Draft — รอ review
**Amends:** [`2026-09-12-rover-mvp-design.md`](2026-09-12-rover-mvp-design.md)
(แก้ R14 และตัวเลขเรขาคณิตทั้งชุด — **ไม่แก้** R1–R13 ข้ออื่น)

**Source spec:** `rover_base_v0_dimensions.md` V0.1 (นอก repo)
**Reference image:** เครื่องระดับ Algribot — body shell + sensor mast + front rotary tool

---

## 1. Context

`2026-09-12-rover-mvp-design.md` ออกแบบ rover ที่ **วิ่งในร่องระหว่างแถวปลูก**
ขนาด 200 × 145 mm ล้อ Ø70 mm หนัก 1.038 kg ขับด้วย Pololu micro metal gearmotor

Spec ใหม่กำหนดเครื่องคนละชั้น: **650 × 520 mm ล้อ Ø250 mm ประมาณ 35 kg ระบบ 24 V**

นี่ไม่ใช่การขยายสัดส่วน — มันเปลี่ยน**ความสัมพันธ์ระหว่างรถกับแถวปลูก** เพราะ
รถกว้าง 520 mm วิ่งในร่องกว้าง 350 mm ไม่ได้ และเปลี่ยน**ข้อจำกัดที่คับที่สุด**
ของทั้งระบบจาก "ล้อเล็กเกินกว่าจะข้ามก้อนดิน" ไปเป็น "รถกว้างเกินกว่าจะลงร่อง"

สิ่งที่ **ไม่** เปลี่ยน: architecture, layer boundary, backend selection,
unit convention, กลไก `# derived: cad` + `test_cad_config_sync.py`,
URDF → base USD + override layer, และ **ตรรกะของ invariant ทั้งเก้าข้อ**

---

## 2. Decisions

| # | Decision | Rationale |
|---|---|---|
| S1 | **Alley driving — ขยาย `row_spacing` 350 → 750 mm** ไม่ใช่คร่อมแถว | รถยังวิ่ง*ระหว่าง*แถวเหมือนเดิม ตรรกะ row following ทั้งชุดใช้ต่อได้ ถ้าเลือกคร่อมแถวต้องเขียน invariant ข้อ 5 ใหม่ทั้งข้อ |
| S2 | **`overall_length` = 650 mm ไม่ใช่ 620** | wheelbase 400 + ล้อ Ø250 = 650 ตามนิยาม "วัดคร่อมล้อ" ที่ความกว้างใช้อยู่แล้ว (430 + 90 = 520 พอดี) — 620 ใน source spec ขัดกับตัวเองข้อ §2 vs §5 |
| S3 | **แชสซีช่วงล้อกว้าง 340 mm ตัวถังบนกว้าง 380 mm** | หน้าในล้ออยู่ที่ ±170 แชสซี 380 mm (±190) ชนล้อ 20 mm ต่อข้าง ตรงช่วง z 125–250 — source spec §8 ขัดกับ acceptance §18 ข้อ "ล้อไม่ชน chassis" |
| S4 | **กล้องทั้งสองตัวอยู่บน mast head** | ล้อ Ø250 บัง — พื้นที่พื้นที่มองตรงลงได้ไม่ถูกล้อบังมีแค่ \|X\| < 75 กับ \|X\| > 325 กล้องล่างที่ความสูงต่ำถูกล้อหน้ากินเฟรมไป 249 จาก 348 mm |
| S5 | **`camera_down_tilt_deg` ยังเป็น 0** | ยกกล้องขึ้น mast แทนการเอียง — เอียงคูณ error จากดินไม่เรียบด้วย `tan(θ)` ตาม `docs/calibration.md` เหตุผลเดิมไม่ได้หายไปเพราะรถใหญ่ขึ้น |
| S6 | **`omega_max` 40 → 25 deg/s** | differential ของล้อเป็นสัดส่วนกับ track ซึ่งโต 120 → 430 (3.6×) ที่ 40 deg/s ล้อข้างในหยุดนิ่งขณะที่ controller คิดว่ากำลังเลี้ยวช้า ๆ |
| S7 | **`v_max` 100 → 160 mm/s** | เพดานจาก runaway budget คือ 200 เลือก 160 เพราะมันดัน margin ของ invariant ข้อ 7 จาก 5.2 เป็น 15.2 mm/s ซึ่งเป็นข้อที่พึ่งค่าที่ยังไม่ได้วัด |
| S8 | **มอเตอร์เป้าหมาย 25 rpm ที่เพลาออก** | invariant บีบสองด้าน: ข้อ 6 ต้องการ ≥ 18.6 rpm ข้อ 7 ต้องการ ≤ 28.6 rpm — หน้าต่างแคบพอที่จะเลือกได้โดยไม่ต้องเดา |
| S9 | **Compute ยังเป็น Raspberry Pi 5** — override source spec §10 ที่ระบุ Jetson | perception เป็น ExG + Otsu ต่อเฟรม ซึ่ง Pi 5 รันสบาย และ `bridge/` + `firmware/` ต่อไว้แล้วทั้งชุด Jetson ได้ประโยชน์ตอน CNN detector ที่ M3 ไม่ใช่ตอนนี้ |
| S10 | **BOM v3 เป็น requirement envelope ไม่ใช่ part number** | รอบนี้ให้ความสำคัญกับ CAD ที่ปลดล็อกงาน development ก่อน การเลือกของจริงเป็นงานแยกที่มี gate ของตัวเอง ตัวเลข mass จึงเป็น estimate ที่ติดธงไว้ |
| S11 | **Sim mesh ถูก generate จาก `parameters.csv`** ด้วย `tools/generate_sim_meshes.py` | Fusion เป็นงานมือ ถ้า sim รอ assembly งาน development หยุด — และ repo แยก `exports/meshes/` (sim) ออกจาก `exports/step\|stl` (ผลิต) ไว้อยู่แล้ว |
| S12 | **ยังไม่ทำ weeding tool** — เตรียม mechanical interface อย่างเดียว | ตรงกับ R2 เดิมและ source spec §1/§13 mast เข้ามาเพราะ perception ขาดไม่ได้ ไม่ใช่เพราะรูปในภาพอ้างอิง |
| S13 | **แทนที่ของเดิม ไม่ทำเป็น variant** | repo ไม่มี plumbing สำหรับหลายขนาด และ abstraction ที่มี implementation เดียวคือต้นทุนเปล่า (YAGNI เหมือน R11) ประวัติอยู่ใน git |
| S14 | **แก้ R14: `body_width` ยังเป็นความกว้างรวมล้อ แต่ค่าเปลี่ยนเป็น 520** | นิยามเดิมถูก เหตุผลเดิมถูก — ส่วนที่ชนใบพืชคือล้อ เปลี่ยนแค่ตัวเลขและ track ที่ได้มาจาก source spec |

---

## 3. ข้อขัดแย้งใน source spec และการตัดสิน

Source spec V0.1 มีสามจุดที่ปิดไม่ลง ต้องตัดสินก่อนเริ่ม CAD

### 3.1 Overall length 620 เป็นไปไม่ได้

```text
wheelbase 400  →  wheel center หน้าอยู่ที่ X = +200
wheel Ø250     →  ขอบล้อยื่นออกไปอีก 125
                  ความยาวคร่อมล้อ = 650 mm

source spec §5 บอก "110 mm หน้า / 110 mm หลัง จาก wheel center ถึงขอบ envelope"
แต่ล้อต้องการ 125 — ขาด 15 mm ต่อข้าง
```

ความกว้างใช้นิยาม "คร่อมล้อ" อยู่แล้วและปิดพอดี (430 + 90 = 520) ความยาวจึงต้องใช้
นิยามเดียวกัน → **650 mm** (ทางเลือกอื่นคือลด wheelbase เป็น 370 เพื่อรักษา 620
ไม่เลือกเพราะ track:wheelbase = 430:400 เป็นสัดส่วนที่ดีสำหรับ skid steer อยู่แล้ว
และ source spec §19 freeze wheelbase 400 ไว้ชัดเจนกว่า 620)

### 3.2 แชสซี 380 mm ชนล้อ

```text
หน้าในล้อ  = track/2 − wheel_width/2 = 215 − 45 = ±170
แชสซี 380  = ±190
             ชน 20 mm ต่อข้าง ตลอดช่วง z 125 → 250 ซึ่งคือที่ที่ §8 วางโครง
```

แก้ด้วยการแยกสองชั้น ซึ่งตรงกับรูปอ้างอิงพอดี — โครงล่างสีเข้มแคบกว่า
ตัวถังขาวด้านบนที่ยื่นคร่อมล้อ

```text
   z=405  ┌──────────────────────────┐   body 500 × 380 × 280
          │                          │
   z=250  ├────┐                ┌────┤   ← หลังคาล้อ
(O)       │    │  frame 340 max │    │       (O)
   z=125  └────┘                └────┘   ← ท้องรถ / ระนาบเพลา
────────────────────────────────────────  พื้น
          ±170 หน้าในล้อ
```

`chassis_plate_width_mm = 340` = `track_width − wheel_width` พอดี ไม่ใช่ค่าที่เลือกเอง

### 3.3 Ground clearance 125 = wheel radius 125 พอดี

ท้องรถอยู่**ที่ระดับเพลา** ซึ่งเป็นค่าสูงสุดทางเรขาคณิตของเพลาแข็งที่ไม่มี portal gear

นี่ไม่ใช่เป้าหมายที่สบาย แต่เป็น**กฎ layout ที่แข็ง**:

> ห้ามมี hub carrier, bearing block, ตัวมอเตอร์, หัวน็อต หรือสายไฟ
> อยู่ต่ำกว่าแนวศูนย์กลางเพลา

source spec §7 เขียนไว้เป็นรายการ "ห้ามต่ำกว่า" — เอกสารนี้ระบุว่ามันคือ
ข้อจำกัดที่บังคับให้ **ขับตรงที่เพลาเท่านั้น**

---

## 4. Parameters

`cad/parameters/parameters.csv` ชุดใหม่ทั้งไฟล์

### 4.1 Chassis

| Parameter | เดิม | ใหม่ | ที่มา |
|---|---:|---:|---|
| `overall_length_mm` | 200 | **650** | derived: `wheelbase + wheel_diameter` (S2) |
| `body_length_mm` | 200 | **500** | source spec §9 |
| `body_width_mm` | 145 | **520** | ความกว้างรวมล้อ = `track + wheel_width` (S14) |
| `chassis_plate_width_mm` | 140 | **340** | `track − wheel_width` — ระยะหน้าในล้อ (S3) |
| `body_height_mm` | 70 | **280** | source spec §9 |
| `chassis_clearance_mm` | 35 | **125** | source spec §2 |

### 4.2 Drive

| Parameter | เดิม | ใหม่ | ที่มา |
|---|---:|---:|---|
| `track_width_mm` | 120 | **430** | source spec §6 |
| `wheelbase_mm` | 120 | **400** | source spec §2 |
| `wheel_diameter_mm` | 70 | **250** | source spec §2 |
| `wheel_width_mm` | 25 | **90** | source spec §6 |
| `motor_mount_bolt_circle_mm` | — | **60** | โครงถูกเจาะตามนี้ 4 × M5 มอเตอร์นอกแพทเทิร์นต้องมีแผ่น adapter |

`motor_mount_pitch_mm` เดิมถูกแทนด้วย `motor_mount_bolt_circle_mm` เพราะมอเตอร์
planetary ชั้นนี้ยึดด้วย bolt circle บนหน้าแปลน ไม่ใช่รูคู่แบบ micro metal gearmotor

> หมายเหตุ: `parameters.csv` เดิมมี `motor_mount_pitch_mm,15` แต่
> `parameters/README.md` เขียน 18 — ค่าทั้งคู่หายไปพร้อมพารามิเตอร์นี้ ไม่ต้องตามแก้

### 4.3 Mast และกล้อง

| Parameter | เดิม | ใหม่ | ที่มา |
|---|---:|---:|---|
| `mast_diameter_mm` | — | **35** | source spec §14 |
| `mast_height_mm` | — | **500** | source spec §12/§14 เหนือตัวถัง |
| `camera_front_height_mm` | 180 | **850** | ใน mast head |
| `camera_front_tilt_deg` | 45 | **50** | → lookahead 713 mm ดู §6.2 |
| `camera_front_offset_x_mm` | 90 | **0** | mast อยู่กึ่งกลาง |
| `camera_down_height_mm` | 220 | **850** | ใน mast head ตัวเดียวกัน (S4) |
| `camera_down_tilt_deg` | 0 | **0** | ไม่เปลี่ยน (S5) |
| `camera_down_offset_x_mm` | 0 | **0** | ไม่เปลี่ยน |

ความสูงที่ได้: ท้องรถ 125 → หลังคาตัวถัง 405 → หัว mast 905 กล้องอยู่ที่ 850
ซึ่งอยู่ใน mast head พอดี

---

## 5. Bed Geometry — ตัวเลขที่ถูกบังคับ ไม่ได้เลือก

Invariant ข้อ 5 คือ `clear_furrow > body_width + 2 × runaway_budget`

```text
ต้องการ   clear_furrow > 520 + 2 × 60 = 640
นิยาม     clear_furrow = row_spacing − 2 × crop_foliage_half_width
→         row_spacing > 640 + 60 = 700
```

**`row_spacing_mm: 350 → 750`** (แถวมาตรฐาน 30 นิ้ว) → `clear_furrow = 690 > 640` margin 50 mm

### 5.1 Margin นี้ถูกใช้จ่ายไปกับค่าที่ยังไม่ได้วัด

`crop_foliage_half_width_mm = 30` ยังเป็นการประมาณ (`ต้องวัดจาก asset พืชที่ V1`)
งบประมาณจึงเป็น:

```text
row_spacing 750 ผ่านตราบใดที่  crop_foliage_half_width < 55 mm
ถ้า V1 วัดได้ 55+  →  row_spacing ต้องขยับเป็น 800+
```

แถว 750 mm ที่ใบพืชกว้างข้างละ 30 mm แปลว่า **พืชระยะต้นอ่อน** ซึ่งเป็นช่วงที่กำจัด
วัชพืชอยู่แล้ว จึงสอดคล้องกัน แต่ต้องเขียนไว้ให้ชัดใน README เพราะรถกว้าง 520 mm
ได้เปลี่ยนค่าประมาณใน config ให้กลายเป็น **ข้อจำกัดว่าลูกค้าปลูกอะไรได้**

### 5.2 ค่าที่ไม่เปลี่ยน

`soil_variation_mm: 15` · `max_lateral_error_mm: 40` · `runaway_budget_mm: 60`
· `command_timeout_ms: 300` · `row_loss_frames: 3` · `loop_hz: 10`

---

## 6. Drive Limits และ Perception Geometry

### 6.1 Drive

```text
v_max_mm_s        100 → 160
omega_max_deg_s    40 →  25
wheel_v_max_mm_s  202 → 327     = (25 rpm / 60) × pi × 250
wheel_v_min_mm_s   51 →  51     ไม่เปลี่ยน — ดูหมายเหตุ

differential ที่ omega_max = radians(25) × 430/2 = 93.8 mm/s
ล้อนอก = 160 + 93.8 = 253.8 mm/s
ล้อใน  = 160 − 93.8 =  66.2 mm/s
```

`wheel_v_min_mm_s` **คงไว้ที่ 51** ด้วยเหตุผลเดียวกับ poc-v2: 15% duty ของ 327
ให้ 49 แต่ค่าที่เข้มกว่าคือค่าที่เก็บ จนกว่าจะวัดจริงที่ V3

`omega_max = 25 deg/s` หมายถึงหมุน 180° ใน 7.2 วินาที ซึ่งเหมาะกับเครื่อง 35 kg —
skid steer ที่ track 430 mm บนยาง Ø250 เสียดสีด้านข้างกินแรงบิดจริง

### 6.2 กล้องหน้า — lookahead

```text
lookahead = camera_front_height / tan(camera_front_tilt)
          = 850 / tan(50°) = 713 mm

= 1.1 เท่าของความยาวรถ  (เดิม 180 / 200 = 0.9 เท่า)
= 4.5 วินาทีที่ v_max
```

เลือกเป็น **เท่าของความยาวรถ** ไม่ใช่เป็นวินาที เพราะเครื่องที่หนักกว่าตอบสนอง yaw
ช้ากว่าและควรมองไกลกว่าในเชิงสัดส่วน ที่ tilt 35° lookahead จะเป็น 1214 mm (1.9 เท่า)
ซึ่งไกลเกินจนตอบสนอง lateral error ช้า ที่ tilt 71° จะได้ 1.8 วินาทีเท่าเดิมแต่กล้อง
จะก้มจนแทบไม่เห็นทิศทางของแถว

> **ไม่มีสูตรจากเรขาคณิตไปหา gain** — `config/README.md` ระบุไว้แล้ว ค่านี้จึงเป็น
> **จุดตั้งต้นสำหรับ tune** ไม่ใช่คำตอบ ต้อง tune ใหม่ที่ V1 และ V4

ตรวจว่ากล้องมองผ่านตัวรถได้:

```text
ray จาก (x=0, z=850) ก้ม 50°  →  ตกลงพื้นที่ x = 713
ที่ x = 250 (หน้าตัวถัง)  z = 850 − 250 × 1.1918 = 552 > 405  ✓
ที่ x = 325 (ขอบล้อหน้า)  z = 850 − 325 × 1.1918 = 463 > 250  ✓
```

แถวปลูกอยู่ที่ Y = ±375 ที่ระยะ 713 mm → ต้องการ HFOV ≥ 38° เลนส์ 60° มาตรฐานพอ

### 6.3 กล้องล่าง — ทำไมต้องขึ้น mast

ล้อ Ø250 ที่ wheelbase 400 กินพื้นที่ X ∈ [+75, +325] และ [−325, −75]
พื้นที่มองตรงลงที่ไม่ถูกล้อบังจึงเหลือแค่ |X| < 75 กับ |X| > 325

```text
กล้องที่ x=+250 z=420 มองตรงลง  footprint X +76 → +424
ล้อหน้ากิน                       X +75 → +325
                                 บังไป 249 จาก 348 mm
```

ทางเลือกที่ถูกปฏิเสธ: ยื่นบูมไปที่ x=+500 (พ้นล้อ แต่ทำให้ความยาวรวมกลายเป็น
~1050 mm และกลายเป็นจุดหน้าสุดที่เปราะ) · เอียงกล้องลงจากตัวถัง (รักษา envelope
แต่ทิ้ง `tilt = 0` ซึ่ง `docs/calibration.md` ยืนยันไว้)

**กล้องล่างขึ้นไปอยู่บน mast ที่ x=0 z=850 มองตรงลงเหมือนเดิม** พร้อม static mask
ตรงมุมเฟรมที่ล้อโผล่เข้ามา

```text
corridor       = clear_furrow − 2 × max_lateral_error = 690 − 80 = 610 mm
ต้องครอบ + ขอบข้างละ 20                                  = 650 mm
HFOV ที่ต้องการ = 2 × atan(325 / 850) = 41.9°           →  spec เลนส์ ≥ 45°
```

เทียบกับของเดิมที่ต้องการ ≥ 60° — เลนส์**ธรรมดาลง** ไม่ใช่กว้างขึ้น เพราะระยะโตเร็วกว่า
ความกว้างที่ต้องครอบ

```text
coverage ตามแนววิ่ง = 2 × 850 × tan(VFOV/2) ≈ 527 mm  (VFOV 34.5° จากเซนเซอร์ 4:3)
ระยะที่รถวิ่งต่อเฟรม = v_max / down_rate_hz = 160 / 2 = 80 mm
527 > 80  ✓ ไม่มีช่องว่างระหว่างเฟรม

GSD = 650 / 1280 px = 0.51 mm/px   (เดิม 0.195 — หยาบขึ้น 2.6 เท่า)
min_weed_area 60 mm² ≈ เส้นผ่านศูนย์กลาง 8.7 mm ≈ 17 px  ✓ ยังตรวจได้
```

**ต้นทุนของทางเลือกนี้:** ล้อโผล่เข้าเฟรมในบริเวณ X ∈ [75, 264] ที่ |Y| ∈ [170, 260]
ทั้งสี่มุม = ~20% ของเฟรม ต้อง mask แบบ static ยางดำไม่เขียวจึงไม่หลุดเข้า ExG mask
อยู่แล้ว แต่มันบังพื้น — mask ทำให้ `touches_side_edge` ไม่สับสนกับขอบล้อ

---

## 7. Invariants — ครบทั้งเก้าที่ค่าใหม่

**ไม่มีการแก้โค้ดใน `controller/startup_checks.py`** ทั้งเก้าข้อเขียนต่อค่าใน config
ไม่ใช่ค่าคงที่ จึงคำนวณตัวเองใหม่ นี่คือผลตอบแทนของ design เดิม

| # | Invariant | เดิม | ใหม่ | |
|---|---|---|---|---|
| 1 | runaway on command timeout | 30 ≤ 60 | **48 ≤ 60** | ✓ |
| 2 | runaway on row loss | 30 ≤ 60 | **48 ≤ 60** | ✓ |
| 3 | `wheel_dia ≥ 4 × soil_var` | 70 ≥ 60 (margin 10) | **250 ≥ 60** (margin 190) | ✓✓ |
| 4 | `chassis_clear > soil_var` | 35 > 15 | **125 > 15** | ✓✓ |
| 5 | `clear_furrow > body + 2×runaway` | 290 > 265 (margin 25) | **690 > 640** (margin 50) | ⚠ คับที่สุด |
| 6 | ล้อนอก ≤ ความเร็วสูงสุดมอเตอร์ | 141.9 ≤ 202 | **253.8 ≤ 327** (margin 73) | ✓ |
| 7 | ล้อใน ≥ deadband | 58.1 ≥ 51 (margin 7.1) | **66.2 ≥ 51** (margin 15.2) | ✓ |
| 8 | gain ถูก tune ตาม backend | ✓ | **ต้อง tune ใหม่** | ⚠ |
| 9 | soil ใน sim ตรงกับแปลง | ✓ | อัปเดต heightfield | ✓ |

ข้อ 3 และ 4 เคยเป็นข้อที่คับที่สุด — ล้อ Ø250 และท้องรถ 125 mm ปลดทั้งคู่ทิ้ง
**ข้อ 5 กลายเป็นข้อที่คับที่สุดแทน** และมันคือข้อที่พึ่ง `crop_foliage_half_width`
ที่ยังไม่ได้วัด

ข้อ 7 ดีขึ้นกว่าเดิม (7.1 → 15.2 mm/s) เพราะ S7 เลือก `v_max = 160` ไม่ใช่ 150

### 7.1 มอเตอร์ถูกบีบจากสองด้าน

```text
ข้อ 6  ล้อนอก 253.8 ≤ wheel_v_max     →  rpm ≥ 18.6
ข้อ 7  deadband ≤ 160 − 93.8 = 66.2
       ที่ 15% duty                    →  rpm ≤ 28.6

เป้าหมาย 25 rpm  →  wheel_v_max 327 mm/s  ·  cruise ที่ 49% duty
```

หน้าต่าง 18.6–28.6 rpm แคบพอที่จะเลือกของได้โดยไม่ต้องเดา และการวิ่งที่ ~49% duty
เป็นไปตามกฎ "ความเร็วมอเตอร์ควรอยู่ที่ 2–3 เท่าของความเร็วใช้งาน"

---

## 8. BOM v3 — Requirement Envelope

S10: รอบนี้ยังไม่เลือก part number ตัวเลขด้านล่างคือ**ข้อกำหนด**ที่คำนวณได้
และเป็นสิ่งที่ `hardware/bom/poc-v3.md` จะบันทึก พร้อม gate "ยืนยันกับ datasheet
ก่อนสั่ง" ทุกบรรทัด

### 8.1 Mass estimate — ~35 kg ±20%

| | kg |
|---|---:|
| ล้อ 4 × (ยาง + วงล้อ + ดุม) | 7.2 |
| มอเตอร์เกียร์ 4 ตัว | 6.0 |
| โครง (อลูมิเนียม 30×30 + แผ่น) | 8.5 |
| Body shell | 3.0 |
| แบตเตอรี่ 24 V LiFePO4 20 Ah | 5.0 |
| Motor driver + power box | 1.5 |
| Pi 5 + ESP32 + สายไฟ | 1.5 |
| Mast + sensor head | 1.2 |
| น็อต + เบ็ดเตล็ด | 1.1 |
| **รวม** | **35.0** |

แยกเป็น base_link 27.8 kg + ล้อ 4 × 1.8 kg — ตัวเลขที่ URDF ใช้ใน §9.2

### 8.2 Torque

```text
rolling (Crr 0.15 ดินร่วน)          1.61 N·m/ล้อ
ทางชัน 15° + rolling                4.33 N·m/ล้อ   ← ข้อกำหนด continuous
skid-steer pivot (mu_lat 0.7)       ~5.5 N·m/ล้อ   ← ข้อกำหนด peak
```

### 8.3 ข้อกำหนดที่ได้

| ของ | ข้อกำหนด |
|---|---|
| มอเตอร์ | 24 V brushed planetary, **25 rpm** ที่เพลาออก (รับได้ 18.6–28.6), **≥ 5 N·m continuous**, **≥ 10 N·m peak**, มี encoder ในตัว, หน้าแปลน bolt circle 60 mm |
| Driver | 4 ช่อง (หรือ dual 2 ตัว) **≥ 10 A/ช่อง continuous ที่ 24 V** |
| E-stop | รีเลย์ latching ตัด rail 24 V ของมอเตอร์ — สถาปัตยกรรมเดิม §11.4 รับกระแสมากขึ้นเท่านั้น |
| แบตเตอรี่ | **24 V LiFePO4 20 Ah / 480 Wh** วางต่ำและกึ่งกลางตาม source spec §10 → ~4 ชม. ที่ ~110 W |
| Compute | Raspberry Pi 5 (S9) + ESP32 — ไม่เปลี่ยน |

**Power budget:** drive ~80 W continuous / ~250 W peak · Pi 5 + sensors ~30 W

---

## 9. CAD Artifacts

### 9.1 Sim mesh ถูก generate (S11)

`tools/generate_sim_meshes.py` อ่าน `parameters.csv` แล้วเขียน:

```text
cad/urdf/meshes/base_link.obj      visual  (โครงล่าง + ตัวถังบน + mast + sensor head)
cad/urdf/meshes/wheel.obj          visual  (ทรงกระบอก Ø250 × 90)
cad/exports/meshes/*.obj  *.stl    สำเนาเดียวกัน
```

และพิมพ์ **inertia tensor** ของ box/cylinder ที่คำนวณเชิงวิเคราะห์ออกมาให้ URDF ใช้

เหตุผล: Fusion เป็นงานมือ ถ้า sim รอ assembly งาน development หยุด — และ repo
แยก `exports/meshes/` (simplified visual + collision สำหรับ sim) ออกจาก
`exports/step|stl` (ผลิต / review / ส่งร้าน) ไว้อยู่แล้ว

> สคริปต์นี้**ไม่ผูกกับ Fusion API** ซึ่งเป็นสิ่งที่ `cad/parameters/README.md`
> เตือนไว้ มันอ่าน CSV อย่างเดียว geometry สำหรับ**ผลิต**ยังเป็นของ Fusion เหมือนเดิม
> และตามมาทีหลังได้โดยไม่บล็อกใคร

### 9.2 URDF

| Element | เดิม | ใหม่ |
|---|---|---|
| wheel joint origin | (±0.060, ±0.060, 0.035) | **(±0.200, ±0.215, 0.125)** |
| wheel collision | cylinder r 0.035 × 0.025 | **cylinder r 0.125 × 0.090** |
| wheel velocity limit | 5.7714 rad/s | **2.616** rad/s (= 327 / 125) |
| wheel effort limit | 0.4903 N·m | **10.0** N·m |
| base_link collision | box เดียว | **สองกล่อง** — ล่าง 0.500×0.340×0.125 ที่ z 0.1875 · บน 0.500×0.380×0.155 ที่ z 0.3275 |
| mast collision | — | **cylinder r 0.0175 × 0.500** ที่ (0, 0, 0.655) |
| `camera_front` origin | (0.090, 0, 0.180) tilt 45° | **(0, 0, 0.850) tilt 50°** |
| `camera_down` origin | (0, 0, 0.220) | **(0, 0, 0.850)** |
| base_link mass | 0.729560 | **~27.8** (estimate) |
| wheel mass ต่อล้อ | 0.075100 | **~1.8** (estimate) |
| รวม | 1.038 kg | **~35 kg** (estimate) |

Wheel center ที่ได้คือ (±200, ±215, 125) mm ตรงกับ source spec §11 ทุกตัว

Inertia tensor ทุกตัวมาจาก `generate_sim_meshes.py` ที่คำนวณจาก primitive
ไม่ได้เขียนมือ — ไม่มีตัวเลข mass property ที่แต่งขึ้น

---

## 10. สิ่งที่เปลี่ยน แยกตามเหตุผล

```text
GEOMETRY (ต้นทาง)   cad/parameters/parameters.csv          ← สวิตช์
                    cad/parameters/README.md · cad/README.md

GENERATED           tools/generate_sim_meshes.py           (ใหม่)
                    cad/{urdf,exports}/meshes/*.obj · exports/stl/*
                    cad/urdf/weeding_rover.urdf

CONFIG (derived)    rover.yaml · control.yaml · perception.yaml
                    camera.yaml · simulation.yaml · README.md
                    drive_mixing_vectors.csv               ← regenerate ทั้งตาราง

SIM (tune ใหม่)     sim/isaac/robots/rover/rover.usd

HARDWARE            hardware/bom/poc-v3.md                 (ใหม่)
                    mechanical/dimensions.md · assembly.md
                    electrical/power.md · wiring.md        ← rail 24 V

NARRATIVE           README.md · docs/architecture.md       ← row spacing 350 → 750

ไม่แตะ              controller/ · perception/ · bridge/ · firmware/ · protocol/
```

`drive_mixing_vectors.csv` ต้อง regenerate ทั้งตารางเพราะ `track_width` อยู่ในสูตร —
`cad/parameters/README.md` เตือนไว้แล้วว่าเป็นข้อที่ test จับไม่ได้

---

## 11. Verification

| Check | จับอะไร |
|---|---|
| `test_cad_config_sync.py` | config ที่ยังตามไม่ทัน CAD |
| `test_urdf_matches_cad.py` | joint origin, limit, collision, mesh path |
| `test_startup_invariants.py` | ทั้งเก้าข้อที่ค่าใหม่ |
| `test_drive_mixing.py` | ตาราง mixing ที่ regenerate ตาม track 430 |
| `test_coordinate_frames.py` | frame ยังถูก (ควรผ่านโดยไม่ต้องแก้ — convention ไม่เปลี่ยน) |
| `test_rover_usd_override.py` | override layer ยัง layer ได้ |
| scenarios | end-to-end หลัง Isaac ถูก tune ใหม่ |

### 11.1 สอง test ที่เปลี่ยน "สมมติฐาน" ไม่ใช่เปลี่ยน "ตัวเลข"

| Test | เปลี่ยนเป็น | เหตุผลที่ต้องเขียนลงไป |
|---|---|---|
| `test_base_link_collision_is_a_box` | หลายกล่อง | โครงต้องคอดเข้าเพื่อหลบล้อ — รูปทรงกล่องเดียวใช้บรรยายรถคันนี้ไม่ได้ |
| `test_total_mass_is_under_the_gearbox_ceiling` | เพดานใหม่ = torque limit ของเกียร์ใหม่ | เพดาน 2.3 kg มาจากเกียร์ 250:1 ตัวเล็กที่ไม่มีอยู่แล้ว |

ทั้งสองข้อ**ห้ามแก้ให้ผ่านเฉย ๆ** ต้องแก้พร้อมเหตุผลในไฟล์ test

---

## 12. Risks — สิ่งที่ไม่มี test จับได้

| # | Risk | ผลถ้าพลาด | จัดการที่ไหน |
|---|---|---|---|
| 1 | **gain ของ row follower ใช้ไม่ได้** — lookahead 180 → 713 mm | rover แกว่งเข้าหาต้นพืชโดยไม่มีใครรู้ว่าทำไม | tune ด้วยมือที่ V1 และ V4 |
| 2 | **`wheel_v_min` margin 15.2 mm/s** บนค่าประมาณ 15% duty | ล้อข้างในหยุดเงียบ เลี้ยวแรงกว่าที่สั่ง | วัดจริงที่ V3 ถ้าเกิน 66 mm/s ต้องลด `omega_max` |
| 3 | **`crop_foliage_half_width < 55 mm`** | invariant ข้อ 5 พัง → `row_spacing` ต้องเป็น 800+ | วัดจาก asset พืชที่ V1 — เป็น**ข้อจำกัดต่อการปลูก** ไม่ใช่แค่ config |
| 4 | **physics ใน Isaac tune ไว้กับของ 1 kg** | friction, drive gain, damping, contact offset ใช้กับ 35 kg ไม่ได้ | tune ใหม่ทั้งชุดใน `rover.usd` ที่ P6 |
| 5 | **mass เป็น estimate ±20%** | inertia ใน sim คลาดจากของจริง | แทนที่ด้วยค่าจริงเมื่อเลือกของที่ BOM รอบถัดไป |
| 6 | **35 kg ไม่ใช่ของที่คนเดียวยก** | การจัดการในแปลงเปลี่ยน — ต้องสองคนหรือทางลาด | เขียนไว้ใน `assembly.md` |

Risk 1–3 เป็นชนิดเดียวกับที่ `cad/parameters/README.md` ระบุว่า
"ต้องอาศัยคนจำ" — จึงถูกเขียนซ้ำที่นั่นด้วย

---

## 13. Work Breakdown

เรียงให้ sim กลับมารันได้เร็วที่สุด

```text
P0  BOM v3 requirement envelope     rpm / effort / mass เป็น input ของทุกเฟสถัดไป
P1  parameters.csv + CAD READMEs    ← สวิตช์ test เริ่มแดงที่นี่
P2  generate_sim_meshes.py + mesh
P3  URDF + test 15 ข้อเขียว
P4  config cascade + mixing regenerate
P5  invariant ทั้งเก้าเขียว
P6  USD tune ใหม่ + scenarios        เฟสยาวและเป็น empirical
P7  docs · dimensions · assembly · power · wiring · narrative
```

ระหว่าง P1–P5 repo อยู่ในสภาพ test แดง ซึ่งเป็นเจตนา — test คือ worklist ที่บอกว่า
อะไรยังตามไม่ครบ

---

## 14. Out of Scope

```text
✗ Weeding tool ทุกชนิด — rotary tine, blade, laser, linear actuator   (M3 ตาม R2)
✗ Fusion assembly rover.f3d                    งานมือ ตามมาทีหลัง ไม่บล็อก P2–P6
✗ STEP / STL สำหรับผลิต                        มาพร้อม Fusion assembly
✗ การเลือก part number จริง                    BOM รอบถัดไป (S10)
✗ Jetson migration                             M3 พร้อม CNN detector (S9)
✗ Suspension / rocker-bogie                    เพลาแข็ง 4 จุดตาม source spec
✗ GNSS / RTK / IMU integration                 mast รองรับไว้ ยังไม่ต่อ
```

---

## 15. Acceptance — เทียบกับ source spec §18

| source spec §18 | สถานะใน design นี้ |
|---|---|
| Overall width ไม่เกิน 520 mm | ✓ 520 พอดี (track 430 + wheel 90) |
| Overall length ประมาณ 620 mm | ⚠ **650** — S2 ตัดสินว่า 620 ขัดกับ wheelbase 400 + Ø250 |
| Wheel Ø250 mm | ✓ |
| Wheelbase 400 mm | ✓ |
| Ground clearance ≥ 125 mm | ✓ 125 พอดี — เป็นกฎ layout ที่แข็ง ดู §3.3 |
| ล้อทั้ง 4 ไม่ชน chassis | ✓ ด้วย S3 (แชสซีช่วงล้อ 340 mm) |
| มีพื้นที่ Battery 24 V | ✓ ต่ำและกึ่งกลาง |
| มีพื้นที่ Motor Driver | ✓ |
| มีพื้นที่ Compute / MCU | ✓ Pi 5 + ESP32 (S9) |
| มีจุดติด E-Stop | ✓ latching relay ตัด rail 24 V |
| มี Reserved Front Tool Interface | ✓ 100–150 mm + cross member + M6/M8 |
| มี Reserved Sensor Mast | ✓ mast มีจริงแล้ว ไม่ใช่แค่ reserved (S4) |
| Battery อยู่ต่ำและใกล้ CG | ✓ |
| ถอดล้อ / motor เพื่อ maintenance ได้ | ✓ ยึดหน้าแปลน bolt circle 60 mm |
| export URDF / USD ไปใช้ใน Isaac ได้ | ✓ P2–P3 |

---

## 16. Next

หลัง review เอกสารนี้ → `writing-plans` เพื่อสร้าง implementation plan ตาม P0–P7
