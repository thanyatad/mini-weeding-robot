# Dimensions

ขนาดทางกายภาพทั้งหมด — **derived จาก CAD** ไม่ใช่ค่าที่ตั้งที่นี่

ต้นทาง: [../../cad/parameters/README.md](../../cad/parameters/README.md)
ถ้าตัวเลขที่นี่กับ `cad/parameters/parameters.csv` ไม่ตรง → CSV ถูก

Rover Base V0 — 650 × 520 mm, ล้อ Ø250, ~35 kg (ดู
[docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md](../../docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md))

---

## Rover — Top / Side View

```text
◄──────────────────────── overall_length 650 ────────────────────────►
                ◄────────── body_length 500 ──────────►
            ┌────────── mast (Ø35) ──────────┐            ▲
            │           camera_down │        │            │
            │        ╲  camera_front│        │           850 mm  (ทั้งสองกล้อง — mast head)
            │         ╲             │        │            │
    ┌───────┴────────────────────────┴───────┴───┐        │  ▲
    │            body shell 380 mm กว้าง          │        │  280 mm  (body_height)
    └──┬────────────────────────────────────┬─────┘        │  ▼  ▲
   ┌───┴──┐  frame 340 mm max (z 125–250) ┌──┴───┐         │     125 mm  (chassis_clearance = wheel radius)
   │wheel │                              │ wheel │         │     ▼
═══╧══════╧══════════════════════════════╧═══════╧═══      ▼  soil reference (Z = 0)
   ◄────────────────── track 430 ──────────────────►
   ◄──────────────── wheelbase 400 ────────────────►  (ไม่ปรากฏในภาพนี้ — วัดตามแนว X)
```

| Dimension | ค่า | CAD parameter |
|---|---|---|
| ความยาวรวม (คร่อมล้อ) | 650 mm | `overall_length_mm` (= wheelbase + wheel_diameter) |
| ความยาวตัวถัง | 500 mm | `body_length_mm` |
| ความกว้างโครงช่วงล้อ (z 125–250) | 340 mm | `chassis_plate_width_mm` |
| **ความกว้างรวมล้อ (จุดกว้างสุด)** | **520 mm** | `body_width_mm` |
| ความกว้าง body shell (z 250–405) | 380 mm | ไม่ใช่ parameter — encode ใน `tools/generate_sim_meshes.py` (`BODY_SHELL_WIDTH_MM`) |
| ความสูงตัวถัง | 280 mm | `body_height_mm` |
| ระยะท้องรถ | 125 mm | `chassis_clearance_mm` |
| ความสูง mast | 500 mm | `mast_height_mm` |
| ความสูงกล้อง (mast head, ทั้งสองตัว) | 850 mm | `camera_front_height_mm` / `camera_down_height_mm` |

### ⚠️ `body_width` ≠ ความกว้างแชสซี — และแชสซีก็ไม่ใช่ความกว้าง body shell

```text
body_width = max(chassis_plate_width, track_width + wheel_width)
           = max(340, 430 + 90)
           = 520 mm
```

**ส่วนที่ชนใบพืชคือล้อ ไม่ใช่ตัวถัง** — `config/rover.yaml` `body_width_mm` ต้องเป็น
**520** เพราะ startup invariant ข้อ 5 ใช้ค่านี้เทียบกับความกว้างร่อง

รถนี้มีความกว้างสามระดับ ไม่ใช่สองแบบเครื่องทั่วไป:

```text
340 mm   โครงช่วงล้อ (z 125–250, ระดับเดียวกับล้อ) — ห้ามเกิน track − wheel_width
380 mm   body shell (z 250–405, เหนือหลังคาล้อ) — ยื่นคร่อมได้เพราะพ้นแนวล้อแล้ว
520 mm   จุดกว้างสุดของรถ (ขอบยางถึงขอบยาง) — ค่าที่ต้องบันทึกลง config
```

นี่เป็นความผิดพลาดประเภทเดียวกับ `rail_length` ≠ `travel` ของ design gantry:
**ตัวเลขที่ดูเหมือนเป็นตัวเดียวกัน แต่ตัวที่ระบบต้องใช้คือตัวที่ใหญ่ที่สุด**

---

## Drive

| Dimension | ค่า | CAD parameter |
|---|---|---|
| Track width (กึ่งกลางล้อซ้าย–ขวา) | 430 mm | `track_width_mm` |
| Wheelbase (กึ่งกลางเพลาหน้า–หลัง) | 400 mm | `wheelbase_mm` |
| เส้นผ่านศูนย์กลางล้อ | 250 mm | `wheel_diameter_mm` |
| ความกว้างหน้ายาง | 90 mm | `wheel_width_mm` |
| Bolt circle หน้าแปลนมอเตอร์ | 60 mm | `motor_mount_bolt_circle_mm` |

Footprint 430 × 400 mm — ใกล้จัตุรัสแต่ไม่เป๊ะ สัดส่วน track:wheelbase = 430:400
เป็นสัดส่วนที่ดีสำหรับ skid steer อยู่แล้ว (source spec §19 freeze `wheelbase 400`
ไว้ชัดเจนกว่าการลดเพื่อรักษาความยาวรวม 620 เดิม — ดู design §3.1)

### Wheel Centres (rover frame, มม.) — design §9.2

| ล้อ | X | Y | Z |
|---|---:|---:|---:|
| หน้าซ้าย | +200 | +215 | 125 |
| หน้าขวา | +200 | −215 | 125 |
| หลังซ้าย | −200 | +215 | 125 |
| หลังขวา | −200 | −215 | 125 |

X = ±wheelbase/2, Y = ±track/2, Z = chassis_clearance (ระดับเพลา) — ตรงกับ
source spec §11 ทุกตัว และเป็นค่าที่ `cad/urdf` ใช้จริง (หน่วยเมตร)

---

## Layout Rules — กฎที่แข็ง ไม่ใช่เป้าหมายที่สบาย (design §3.3, §3.2)

```text
ห้ามมีอะไรต่ำกว่าแนวศูนย์กลางเพลา (ground clearance = wheel radius)
โครงช่วง z 125–250 กว้างได้ไม่เกิน 340 mm
```

กฎแรก: `chassis_clearance_mm = 125` เท่ากับรัศมีล้อพอดี ท้องรถอยู่**ที่ระดับเพลา**
ซึ่งเป็นค่าสูงสุดทางเรขาคณิตของเพลาแข็งที่ไม่มี portal gear ห้ามมี hub carrier,
bearing block, ตัวมอเตอร์, หัวน็อต หรือสายไฟอยู่ต่ำกว่าแนวศูนย์กลางเพลา — แปลว่า
**ขับตรงที่เพลาเท่านั้น** ไม่มีเฟืองทด ไม่มีโซ่ ไม่มีสายพานที่ห้อยลงมา

กฎที่สอง: โครงช่วงล้อ (z 125–250, ระดับเดียวกับล้อ) กว้างได้ไม่เกิน 340 mm —
ตัวถังเหนือ z 250 ยื่นได้ถึง 380 mm เพราะพ้นแนวล้อแล้ว (ดู `⚠️ body_width` ด้านบน)

---

## Transmission — ขับตรง ไม่มีสายพาน

| รายการ | ค่า |
|---|---|
| Motor | DC gear motor 24 V, planetary + encoder ในตัว |
| ความเร็วรอบ nominal | **~25 RPM** (หน้าต่าง 18.6–28.6 RPM ดู `hardware/bom/poc-v3.md`) |
| การส่งกำลัง | ขับตรงที่เพลาล้อ (ไม่มี belt / pulley) |
| ความเร็วล้อสูงสุด | **327 mm/s** |
| ความเร็วใช้งาน | 160 mm/s (49% duty) |
| Deadband (ประมาณ) | ~51 mm/s (15% duty) — **ต้องวัดจริงที่ V3** |

```text
wheel_v_max = (motor_rpm / 60) × pi × wheel_diameter
            = (25 / 60) × pi × 250
            = 327 mm/s

ความเร็วใช้งาน 160 mm/s → 160 / 327 = 49% duty
```

### ⚠️ มอเตอร์ที่เร็วกว่า **แย่กว่า** ไม่ใช่ดีกว่า

นี่เป็นเรื่องที่ต่างจากงาน stepper ของ design gantry ที่ resolution เหลือเฟือ
ไม่เสียหาย มอเตอร์เกียร์ DC ไม่ออกตัวใต้ ~15% duty **ความเร็วมอเตอร์ควรอยู่ที่
2–3 เท่าของความเร็วใช้งาน** ไม่ใช่มากที่สุดที่หาได้ (327 / 160 ≈ 2 เท่า ✓ —
ตรงกับกฎเดียวกันที่ตัดเกียร์ 125:1 ทิ้งใน poc-v2 เพราะเร็วไป 4%)

ตัวจำกัดความแม่นยำของ rover ไม่ใช่ resolution ของมอเตอร์ — มันคือ **slip
ของ skid-steer** และ **ความแม่นของ row estimator** ซึ่งแก้ด้วยมอเตอร์เร็วกว่าไม่ได้

### Deadband กำหนด `omega_max`

```text
ที่ omega = 25 deg/s:  v_left = 160 − 93.8 = 66.2 mm/s  (41%)  ✓ margin 15.2 mm/s

เพดานจริง: omega ≈ 29.0 deg/s ทำให้ v_left แตะ 51 พอดี → เลือก 25 เพื่อมี margin
```

ล้อข้างในที่อยู่ใน deadband จะ **หยุดหมุนขณะที่ ESP32 คิดว่ากำลังสั่งให้หมุน**
rover จะเลี้ยวแรงกว่าที่สั่งโดยไม่มีสัญญาณบอก — ไม่มี encoder ไม่มีใครรู้

---

## Camera

| Dimension | ค่า | CAD parameter |
|---|---|---|
| กล้องหน้า — ความสูง | 850 mm | `camera_front_height_mm` |
| กล้องหน้า — มุมก้ม | 50° | `camera_front_tilt_deg` |
| กล้องหน้า — เลื่อนหน้า | 0 mm (กึ่งกลาง mast) | `camera_front_offset_x_mm` |
| กล้องล่าง — ความสูง | 850 mm | `camera_down_height_mm` |
| กล้องล่าง — มุมก้ม | 0° (ตรงลง) | `camera_down_tilt_deg` |

กล้องทั้งสองตัวอยู่บน **mast เดียวกัน** ที่หัว mast (905 mm) — กล้องล่างต้องขึ้นมา
จากตัวถังเพราะล้อ Ø250 บังพื้นที่มองตรงลงจนเหลือน้อยเกินไป (design §6.3)

### กล้องหน้า — lookahead

```text
lookahead = 850 / tan(50°) = 713 mm ข้างหน้า  = 1.1 เท่าของความยาวรถ
เวลาที่ได้ = 713 / 160 = 4.5 s
```

**ไม่มีข้อกำหนด FOV เป็นองศา** เพราะ `RowEstimate` เป็นค่า image-space ล้วน —
ขอแค่เห็นแถวพืชสองข้างในเฟรมที่ระยะ lookahead

⚠️ แต่ตัวเลข lookahead นี้ **คือสิ่งที่ gain ของ row follower ถูก tune กับมัน**
ขยับขายึดกล้อง → gain ใช้ไม่ได้ → ต้อง tune ใหม่ที่ V1 และ V4
**ไม่มี test จับได้** (lookahead ขยับจาก 180 mm เป็น 713 mm ในรอบ scale-up นี้)

### กล้องล่าง — ต้องครอบร่อง

ที่ความสูง 850 mm ต้องครอบความกว้าง **650 mm** (corridor 610 + ขอบข้างละ 20):

```text
HFOV แนวนอนที่ต้องการ >= 2 × atan(325 / 850) = 41.9°   →  ต้องการ >= 45°
VFOV ที่ HFOV 45° (เซนเซอร์ 1280×720, 16:9) = 26.2° → coverage ตามแนววิ่ง ≈ 396 mm
ระยะที่รถวิ่งต่อเฟรม = 160 / 2 Hz = 80 mm  →  396 > 80 ✓ มี overlap
```

เทียบกับของเดิมที่ต้องการ ≥ 60° — เลนส์**ธรรมดาลง** ไม่ใช่กว้างขึ้น เพราะระยะ
850 mm โตเร็วกว่าความกว้างที่ต้องครอบ

⚠️ coverage ตามแนววิ่ง (396 mm) คำนวณจากเซนเซอร์ 16:9 จริง (`config/simulation.yaml`
1280 × 720) — ไม่ใช่ 527 mm ที่ design spec §6.3 เคยคำนวณไว้โดยสมมติเซนเซอร์ 4:3
ตัวเลขนี้คือค่าที่ตรงกับของจริง อย่าแก้กลับเป็น 527

ต้องมีขอบ ไม่ใช่ครอบแค่ corridor พอดี — กฎ `touches_side_edge` ที่ใช้ตัดใบพืชผลออก
ทำงานได้เฉพาะเมื่อใบพืชผลแตะขอบเฟรมจริง

**ตรวจสเปกก่อนซื้อ** ดู [../bom/poc-v3.md](../bom/poc-v3.md#compute--ไม่เปลี่ยนจาก-v2)

### ⚠️ เสากล้อง 500 mm ทำให้จุดศูนย์ถ่วงสูง

หัว mast อยู่ที่ 905 mm บนรถกว้างรวมล้อ 520 mm — **ต้องวางแบตเตอรี่และมอเตอร์
ให้ต่ำที่สุด** ไม่งั้นพลิกง่ายบนดินขรุขระ เครื่อง ~35 kg พลิกแล้วอันตรายกว่าเครื่อง
เดิมมาก ไม่ใช่แค่เรื่อง sim

ถ้า V1 แสดงว่าพลิกใน sim ให้แก้ที่การจัดวางมวล **ไม่ใช่ลดความเร็ว** —
`v_max` ผูกกับ `runaway_budget` invariant อยู่แล้ว

---

## Bed (แปลงทดสอบ)

```text
◄──────────────────── 2000 mm ────────────────────►

═════════════ crop row 1 ═════════════   ▲
                                          │ 750 mm
        ╔═══╗                             │
        ║ R ║  rover ในร่อง                │  ← clear furrow 690 mm
        ╚═══╝                             │
═════════════ crop row 2 ═════════════   ▼
                                          │ 750 mm
═════════════ crop row 3 ═════════════   ▼

ขอบแปลงยกสูงกว่า 125 mm  (> wheel_diameter / 2)
```

| Dimension | ค่า |
|---|---|
| ความยาว | 2000 mm |
| จำนวนแถว | 3 |
| `row_spacing` | 750 mm |
| `crop_foliage_half_width` | 30 mm (ประมาณ — วัดที่ V1, ต้อง < 55 mm) |
| **clear furrow** | **690 mm** |
| `crop_spacing` ตามแถว | 80 mm |
| ความสูงขอบแปลง | > 125 mm |

### `row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้

```text
clear_furrow = row_spacing − 2 × crop_foliage_half_width = 750 − 60 = 690 mm
ช่องว่างข้างละ = (690 − 520) / 2 = 85 mm
```

invariant ข้อ 5 ต้องการช่องว่างข้างละ ≥ `runaway_budget` (60 mm) → 85 ≥ 60 ✓
(margin รวมสองข้าง 50 mm — ดู [`config/README.md`](../../config/README.md#crop_foliage_half_width_mm-คือ-margin-ของ-invariant-ข้อ-5-ทั้งก้อน)
สำหรับว่า margin นี้ถูกใช้จ่ายไปกับอะไร)

### ขอบแปลงยกไม่ใช่ของประดับ

MVP **ไม่มี bumper switch** ขอบยกเป็นสิ่งเดียวที่กัน rover ตกแปลงทางกายภาพ
ถ้าขอบไม่ยกสูงกว่ารัศมีล้อ (125 mm) rover จะไต่ข้ามไปได้

---

## Consistency

ตัวเลขในเอกสารนี้ต้องตรงกับ:

```text
cad/parameters/parameters.csv       ต้นทาง
config/rover.yaml                   derived (มี comment ชี้กลับ)
cad/urdf/weeding_rover.urdf         derived (หน่วยเมตร)
config/drive_mixing_vectors.csv     derived จาก track_width — regenerate เมื่อเปลี่ยน
```

ตรวจด้วย:

```bash
pytest tests/unit/test_cad_config_sync.py
pytest tests/unit/test_drive_mixing.py
pytest tests/unit/test_urdf_matches_cad.py
```

### สิ่งที่ test จับไม่ได้

```text
✗ gain ยังเหมาะกับ camera_front_tilt_deg ใหม่หรือไม่
✗ FOV กล้องจริงครอบ corridor ได้หรือไม่
✗ จุดศูนย์ถ่วงต่ำพอหรือไม่
```

สามข้อนี้ต้องอาศัยคนตรวจ — จึงถูกเขียนกำกับไว้ทุกที่ที่มันเกี่ยวข้อง
