# CAD Parameters

Fusion 360 user parameters — **ต้นทางของตัวเลขขนาดทุกตัว**

Export เป็น `parameters.csv` แล้ว commit ทุกครั้งที่แก้ขนาด

Rover Base V0 — 650 × 520 mm, ล้อ Ø250, 4WD skid steer, ~35 kg
Design: [`docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md`](../../docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md)

---

## Naming Convention

```text
<part>_<dimension>_<unit>

track_width_mm
wheel_diameter_mm
camera_front_tilt_deg
```

ลงท้ายด้วยหน่วยเสมอ — ป้องกันความผิดพลาด mm/cm และ deg/rad ที่หาไม่เจอ
จนกว่าจะสร้างของจริง

---

## Parameters

### Chassis

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `body_length_mm` | 500 | ความยาวตัวถัง (ไม่รวมล้อ) |
| `chassis_plate_width_mm` | 340 | ความกว้างโครงช่วงล้อ (z 125–250, ระดับเดียวกับล้อ) ดูหมายเหตุ |
| `body_width_mm` | **520** | **ความกว้างรวมล้อ = จุดกว้างสุดของรถ** ดูหมายเหตุ |
| `body_height_mm` | 280 | ความสูงตัวถัง (ไม่รวม mast) |
| `chassis_clearance_mm` | 125 | จากพื้นถึงท้องรถ = รัศมีล้อพอดี |

### Drive

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `track_width_mm` | 430 | ระยะกึ่งกลางล้อซ้ายถึงกึ่งกลางล้อขวา |
| `wheelbase_mm` | 400 | ระยะกึ่งกลางเพลาหน้าถึงกึ่งกลางเพลาหลัง |
| `wheel_diameter_mm` | 250 | เส้นผ่านศูนย์กลางล้อรวมยาง — ดู [`hardware/bom/poc-v3.md`](../../hardware/bom/poc-v3.md) |
| `wheel_width_mm` | 90 | ความกว้างหน้ายาง |
| `motor_mount_bolt_circle_mm` | 60 | bolt circle หน้าแปลนมอเตอร์ 4 × M5 |

ไม่มี belt / pulley / stepper — ขับตรงจากมอเตอร์เกียร์ที่เพลาล้อ

`motor_mount_bolt_circle_mm` แทนที่ `motor_mount_pitch_mm` เดิมทั้งตัว — มอเตอร์
planetary ชั้นนี้ยึดด้วย bolt circle บนหน้าแปลน ไม่ใช่รูคู่แบบ micro metal gearmotor
ของเครื่องเดิม พารามิเตอร์เก่าไม่มีอยู่ใน `parameters.csv` แล้ว (ดู git history
ของไฟล์นี้สำหรับค่าเดิม)

### Mast และกล้อง

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `mast_diameter_mm` | 35 | ท่อ mast |
| `mast_height_mm` | 500 | ความสูง mast เหนือหลังคาตัวถัง |
| `camera_front_height_mm` | 850 | ความสูงเหนือ soil reference (ใน mast head) |
| `camera_front_tilt_deg` | 50 | มุมก้มจากแนวนอน |
| `camera_front_offset_x_mm` | 0 | mast อยู่กึ่งกลางตัวรถ |
| `camera_down_height_mm` | 850 | ความสูงเหนือ soil reference (ใน mast head เดียวกัน) |
| `camera_down_tilt_deg` | 0 | 0 = มองตรงลง |
| `camera_down_offset_x_mm` | 0 | อยู่กึ่งกลางตัวรถ |

ความสูงที่ได้: ท้องรถ 125 → หลังคาตัวถัง 405 (125 + 280) → หัว mast 905
(405 + 500) กล้องทั้งสองตัวอยู่ที่ 850 mm ซึ่งอยู่ใน mast head พอดี — คนละ
ตำแหน่งกับเวอร์ชันก่อน scale-up ที่กล้องหน้าอยู่บนตัวถัง (180 mm) และกล้องล่าง
แยกเสาของตัวเอง (220 mm) ตอนนี้ **กล้องทั้งสองตัวขึ้น mast เดียวกัน** เพราะล้อ
Ø250 บังพื้นที่มองตรงลงจนต้องยกกล้องล่างขึ้นไปพ้นล้อ (design §6.3)

`camera_down_tilt_deg = 0` ตั้งใจ — ยิ่งเอียงยิ่งทำให้ error จากดินไม่เรียบใหญ่ขึ้น
เหตุผลเดียวกับกล้องของ design gantry ดู
[../../docs/calibration.md](../../docs/calibration.md#soil-plane-error-budget)

---

## ความสัมพันธ์ที่ test บังคับ

สามค่านี้เป็น **ผลลัพธ์** ไม่ใช่ค่าที่เลือกอิสระ — `test_cad_config_sync.py`
บังคับความสัมพันธ์ไว้ ห้ามแก้ค่าใดค่าหนึ่งโดยไม่ตรวจอีกสองตัว

```text
overall_length_mm      = wheelbase_mm + wheel_diameter_mm
                        = 400 + 250 = 650

body_width_mm           = max(chassis_plate_width_mm, track_width_mm + wheel_width_mm)
                        = max(340, 430 + 90) = 520

chassis_plate_width_mm <= track_width_mm - wheel_width_mm
                        = 430 - 90 = 340
```

| Test | ตรวจอะไร |
|---|---|
| `test_overall_length_is_measured_across_the_wheels` | `overall_length_mm` วัดคร่อมล้อ ไม่ใช่แค่ตัวถัง |
| `test_body_width_is_the_widest_point_not_the_chassis_plate` | `body_width_mm` คือจุดกว้างสุด (รวมล้อ) |
| `test_the_chassis_plate_clears_the_inner_faces_of_the_wheels` | โครงช่วงล้อไม่ชนหน้าในล้อ |

---

## ⚠️ `body_width_mm` ต้องเป็นความกว้างรวมล้อ ไม่ใช่ความกว้างแชสซี

```text
body_width = max(chassis_plate_width, track_width + wheel_width)
           = max(340, 430 + 90)
           = 520 mm
```

**ส่วนที่ชนใบพืชคือล้อ ไม่ใช่ตัวถัง** — `body_width_mm` ต้องสะท้อนขอบล้อ
ไม่ใช่ขอบแชสซี ไม่งั้น invariant ข้อ 5 จะผ่านโดยที่ล้อยังเบียดใบพืชอยู่

> เวอร์ชันก่อน scale-up (รถ 145 mm) เคยลด `track_width` จาก 140 เป็น 120
> ด้วยเหตุผลเดียวกันนี้ — เรื่องนั้นเป็นประวัติของเครื่องคนละขนาด ดู git history
> ของไฟล์นี้ ไม่ใช่ข้อจำกัดของรถคันนี้

### แชสซี 340 mm ไม่ชนล้อ ตัวถัง 380 mm ยื่นคร่อมได้ (design §3.2)

```text
หน้าในล้อ  = track/2 − wheel_width/2 = 215 − 45 = ±170   →  ระยะห่าง 340 mm

   z=405  ┌──────────────────────────┐   body 500 × 380 × 280
          │                          │
   z=250  ├────┐                ┌────┤   ← หลังคาล้อ (wheel top)
(O)       │    │  frame 340 max │    │       (O)
   z=125  └────┘                └────┘   ← ท้องรถ / ระนาบเพลา
────────────────────────────────────────  พื้น
          ±170 หน้าในล้อ
```

โครงช่วงล้อ (`chassis_plate_width_mm`) อยู่ในช่วง z 125–250 mm ซึ่งเป็นระดับ
เดียวกับตัวล้อพอดี — กว้างเกิน 340 mm ตรงนี้คือชนหน้าในล้อ ตัวถังด้านบน
(z 250–405 mm) อยู่เหนือหลังคาล้อแล้ว จึงยื่นคร่อมได้ถึง 380 mm โดยไม่ชน

⚠️ **340 คือ "ค่าสูงสุด" ไม่ใช่ค่าที่มี clearance** — ที่ 340 โครงแตะหน้าในล้อพอดี
(±170 ทั้งคู่) ไม่เหลือช่องให้ค่าความคลาดเคลื่อน ±2 mm ที่ source spec section 16
ยอมให้ ตอนขึ้นรูปจริงใน Fusion ควรใช้ **320** เพื่อให้เหลือข้างละ 10 mm ค่า 340
ในตารางเป็นค่า "ขอบเขต" ที่ test บังคับ ไม่ใช่ค่าที่แนะนำให้ตัดเหล็ก

---

## Coverage Checks — คำนวณได้ ต้องตรวจก่อนสั่งกล้อง

สองข้อนี้ **ไม่ใช่ startup invariant** (ค่า FOV ไม่อยู่ใน config) แต่ต้องตรวจ
ก่อนซื้อของ และวัดจริงที่ V1

### กล้องล่างต้องครอบ corridor + ขอบ

```text
กล้องล่างต้องครอบ corridor + ขอบ
  corridor              610 mm   (clear_furrow 690 − 2 × max_lateral_error 40)
  ต้องครอบ + ขอบข้างละ 20  650 mm
  HFOV ที่ต้องการ >= 2 × atan(325 / 850) = 41.9°   →  ต้องการ >= 45°
```

ต้องมีขอบ ไม่ใช่ครอบแค่ corridor พอดี — กฎ `touches_side_edge` ทำงานได้เฉพาะ
เมื่อใบพืชผล**แตะขอบเฟรมจริง** ถ้าเฟรมกว้างเท่า corridor พอดี ใบพืชผลจะไม่มีที่ยืน
แล้วจะถูกนับเป็นวัชพืช

เทียบกับของเดิมที่ต้องการ ≥ 60° — เลนส์**ธรรมดาลง** ไม่ใช่กว้างขึ้น เพราะระยะ
850 mm โตเร็วกว่าความกว้างที่ต้องครอบ (design §6.3)

### ต้องไม่มีช่องว่างระหว่างเฟรม

```text
ต้องไม่มีช่องว่างระหว่างเฟรม
  เซนเซอร์ down คือ 1280 × 720 (16:9) จาก config/simulation.yaml
  VFOV ที่ HFOV 45° = 2 × atan(tan(22.5°) × 720/1280) = 26.2°
  coverage ตามแนววิ่ง = 2 × 850 × tan(13.1°) ≈ 396 mm
  ระยะที่รถวิ่งต่อเฟรม = 160 / 2 = 80 mm
  396 > 80  ✓
```

⚠️ **396 mm ไม่ใช่ 527 mm** — design spec §6.3 คำนวณ coverage ตามแนววิ่งไว้ที่
527 mm โดยสมมติเซนเซอร์ 4:3 (`VFOV 34.5°` จากอัตราส่วนภาพ 4:3) แต่
`config/simulation.yaml` ระบุกล้อง down เป็น **1280 × 720 (16:9)** จริง ซึ่งให้
VFOV แคบกว่าที่ HFOV เดียวกัน (26.2° ไม่ใช่ 34.5°) coverage จึงเหลือ 396 mm
ไม่ใช่ 527 mm ตัวเลขในเอกสารนี้คือค่าที่ตรงกับเซนเซอร์จริง — **อย่าแก้กลับเป็น
527** ถึงแม้ design spec จะยังพิมพ์ 527 ไว้เป็นบันทึกประวัติ (spec ไม่แก้ย้อนหลัง)
396 ยังมากกว่าระยะวิ่งต่อเฟรม (80 mm) มาก จึงยังผ่านสบาย

### กล้องหน้า — lookahead

```text
กล้องหน้า — lookahead
  lookahead = 850 / tan(50°) = 713 mm  = 1.1 เท่าของความยาวรถ
  เวลาที่ได้ = 713 / 160 = 4.5 s
```

**ตัวเลขนี้คือสิ่งที่ gain ของ row follower ถูก tune กับมัน** เปลี่ยนความสูง
หรือมุม แล้ว gain ใช้ไม่ได้ — ไม่มี test จับได้ ต้อง tune ใหม่ที่ V1 และ V4
(lookahead ขยับจาก 180 mm เป็น 713 mm ในรอบ scale-up นี้ ดู `config/control.yaml`)

กล้องหน้า **ไม่ต้องมีข้อกำหนด FOV เป็นองศา** เพราะ `RowEstimate` เป็นค่า
image-space ล้วน ขอแค่เห็นแถวพืชสองข้างอยู่ในเฟรมที่ระยะ lookahead

---

## Derived Values — ค่าที่ไหลจาก CAD ไป config

นี่คือรายการที่ต้อง sync ทุกครั้งที่ CAD เปลี่ยน

| CAD parameter | → ปลายทาง | ความสัมพันธ์ |
|---|---|---|
| `track_width_mm` | `config/rover.yaml` `rover.track_width_mm` | เท่ากัน |
| `track_width_mm` | `cad/urdf` wheel joint origin Y | `± track_width/2` (เมตร) |
| `track_width_mm` | `config/drive_mixing_vectors.csv` | **ต้อง regenerate ทั้งตาราง** |
| `wheelbase_mm` | `config/rover.yaml` · `cad/urdf` joint origin X | `± wheelbase/2` (เมตร) |
| `wheelbase_mm` + `wheel_diameter_mm` | `overall_length_mm` (เอกสารนี้) | `wheelbase + wheel_diameter` — บังคับด้วย test |
| `wheel_diameter_mm` | `config/rover.yaml` · `cad/urdf` wheel radius | radius = `d/2` (เมตร) |
| `wheel_diameter_mm` + BOM `motor_rpm` ([`poc-v3.md`](../../hardware/bom/poc-v3.md)) | `config/rover.yaml` `wheel_v_max_mm_s` | `(rpm/60) × pi × d` |
| `body_width_mm` | `config/rover.yaml` `rover.body_width_mm` | เท่ากัน — **ใช้ใน invariant ข้อ 5** |
| `chassis_clearance_mm` | `config/rover.yaml` | เท่ากัน — **ใช้ใน invariant ข้อ 4** |
| `mast_height_mm` + `mast_diameter_mm` | `cad/urdf` mast collision cylinder | radius = `mast_diameter/2`, สูง = `mast_height`, จุดศูนย์กลางที่ z = `chassis_clearance + body_height + mast_height/2` |
| `camera_front_*` | `config/camera.yaml` extrinsic | ค่าเริ่มต้น (กล้องหน้าไม่ calibrate) |
| `camera_down_*` | `config/camera.yaml` extrinsic | ค่าเริ่มต้นก่อน calibrate |
| `camera_front_height_mm` + `tilt_deg` | `config/control.yaml` **gain** | **ไม่มีสูตร — ต้อง tune** |

### `wheel_v_max` ตรวจสอบได้

```text
wheel_v_max = (motor_rpm / 60) × pi × wheel_diameter_mm
            = (25 / 60) × 3.14159 × 250
            = 0.41667 × 785.4
            = 327.2 mm/s   →  config: wheel_v_max_mm_s: 327
```

`motor_rpm = 25` มาจาก [`hardware/bom/poc-v3.md`](../../hardware/bom/poc-v3.md)
(หน้าต่างความเร็วที่ใช้ได้คือ 18.6–28.6 RPM บีบจาก invariant ข้อ 6 และ 7) —
**ไม่ใช่จาก CAD** `test_the_bom_still_states_the_motor_speed` และ
`test_wheel_v_max_follows_from_wheel_diameter_and_motor_rpm` บังคับทั้งค่าและสูตรนี้

### Startup invariants ที่ค่าจาก CAD เข้าไปเกี่ยวข้อง (design §7)

```text
geometry  wheel_diameter        >= 4 × soil_variation                  250 >= 60   ok  margin 190 mm
geometry  chassis_clearance     >  soil_variation                      125 >  15   ok  margin 110 mm
geometry  clear_furrow          >  body_width + 2 × runaway_budget     690 > 640   ok  margin  50 mm
drive     v + omega_max_rad × track/2  <= wheel_v_max                253.8 <= 327  ok  margin  73.2 mm/s
drive     v − omega_max_rad × track/2  >= wheel_v_min                 66.2 >=  51  ok  margin  15.2 mm/s
```

ข้อ geometry ทั้งสองข้อมี margin กว้างขึ้นมาก (190 mm, 110 mm) — ล้อ Ø250 และ
ท้องรถ 125 mm ปลดข้อจำกัดที่เคยคับที่สุดของเครื่องเดิมทิ้งทั้งคู่

⚠️ **`clear_furrow > body_width + 2 × runaway_budget` กลายเป็นข้อที่คับที่สุดแทน**
(margin 50 mm) และมันคือข้อที่พึ่ง `crop_foliage_half_width_mm` ซึ่งยังไม่ได้วัด
— ดู [`../../config/README.md`](../../config/README.md#crop_foliage_half_width_mm-คือ-margin-ของ-invariant-ข้อ-5-ทั้งก้อน)

⚠️ **invariant `drive` ตัวล่างมี margin 15.2 mm/s** — `wheel_v_min_mm_s = 51`
เป็นค่าประมาณจาก 15% duty **ยังไม่ได้วัด** ถ้าวัดจริงที่ V3 ได้เกิน **66.2 mm/s**
ต้องลด `omega_max` (เพดานจริง ≈ 29.0 deg/s ที่ `v_left` แตะ 51 พอดี) ห้ามลด
`wheel_v_min`

---

## ค่าที่ตั้งไว้และควรตรวจสอบก่อนสร้างจริง

ตัวเลขในเอกสารนี้เป็น **ค่าเริ่มต้นที่สอดคล้องกัน** ไม่ใช่ค่าที่วัดจากของจริง

| ต้องยืนยัน | ที่ไหน | ถ้าผิดแล้วเกิดอะไร |
|---|---|---|
| `motor_rpm` ~25 (หน้าต่าง 18.6–28.6 RPM) | ก่อนสั่งมอเตอร์ | นอกหน้าต่างนี้ทำให้ invariant ข้อ 6 (ล้อนอกเกิน `wheel_v_max`) หรือข้อ 7 (ล้อในต่ำกว่า deadband) ไม่ผ่าน |
| `wheel_v_min` ที่วัดได้ (V3) — ต้อง ≤ 66.2 mm/s | V3 | เกินแล้วต้องลด `omega_max_deg_s` ห้ามลด `wheel_v_min` |
| FOV กล้องล่าง >= 45° ที่ 850 mm | ก่อนสั่งกล้อง | เห็นร่องไม่ครบ ใบพืชผลไม่แตะขอบเฟรม → นับเป็นวัชพืช |
| `crop_foliage_half_width_mm` **< 55 mm** | V1 (asset พืช) | ถ้า V1 วัดได้ 55+ ต้องขยับ `row_spacing_mm` เป็น 800+ — กระทบทั้ง invariant ข้อ 5 และ corridor พร้อมกัน |
| `soil_variation_mm` <= 62 mm (ไม่ใช่ข้อที่คับที่สุดอีกต่อไป) | V4 (แปลงจริง) | ล้อ Ø250 ไม่พอ ต้องเปลี่ยนล้อใหญ่ขึ้น — margin กว้างกว่าเดิมมาก (190 mm ที่ 15 mm) จึงไม่ใช่จุดเสี่ยงหลักแล้ว |

ความเร็วมอเตอร์ควรอยู่ที่ **2–3 เท่า** ของความเร็วใช้งาน ไม่ใช่มากที่สุดที่หาได้ —
นี่เป็นเรื่องที่ต่างจากงาน stepper ที่เร็วกว่าไม่เสียหาย (`327 / 160` ≈ 2 เท่า ✓)

---

## Sync Enforcement

```bash
pytest tests/unit/test_cad_config_sync.py
```

Test อ่าน `parameters.csv` แล้วเทียบกับ `config/rover.yaml` ตามตารางด้านบน
fail เมื่อค่าไม่ตรง

ใน YAML ค่า derived ทุกตัวต้องมี comment ชี้กลับมา:

```yaml
rover:
  track_width_mm: 430          # derived: cad track_width
```

ไม่มี generator อัตโนมัติโดยตั้งใจ — tooling ที่ผูกกับ Fusion API มีต้นทุนดูแลสูง
เกินความจำเป็นของ POC ตาราง + test จับ drift ได้เพียงพอในขนาดงานนี้

### สิ่งที่ test จับไม่ได้

`test_cad_config_sync.py` ตรวจว่า**ตัวเลขตรงกัน** แต่ตรวจไม่ได้ว่า:

```text
✗ gain ยังเหมาะกับ camera_front_tilt_deg ใหม่หรือไม่
✗ drive_mixing_vectors.csv ถูก regenerate หลังแก้ track_width หรือยัง
✗ FOV กล้องจริงครอบ corridor ได้หรือไม่
```

สามข้อนี้ต้องอาศัยคนจำ — จึงถูกเขียนไว้ที่นี่และใน
[../README.md](../README.md#️-พารามิเตอร์ที่แก้แล้วกระทบมากกว่าที่คิด)
