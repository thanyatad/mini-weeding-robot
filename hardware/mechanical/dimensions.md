# Dimensions

ขนาดทางกายภาพทั้งหมด — **derived จาก CAD** ไม่ใช่ค่าที่ตั้งที่นี่

ต้นทาง: [../../cad/parameters/README.md](../../cad/parameters/README.md)
ถ้าตัวเลขที่นี่กับ `cad/parameters/parameters.csv` ไม่ตรง → CSV ถูก

---

## Rover

```text
            ┌── camera_down ──┐        ▲
            │       │         │        │
            │  camera_front   │        220 mm
            │    ╲  │         │        │
    ┌───────┴───────┴─────────┴───┐    │  ▲
    │        electronics tray     │    │  105 mm
    └──┬──────────────────────┬───┘    │  ▼  ▲
   ┌───┴──┐              ┌────┴──┐     │     35 mm
   │wheel │              │ wheel │     │     ▼
═══╧══════╧══════════════╧═══════╧═══  ▼  soil reference (Z = 0)
```

| Dimension | ค่า | CAD parameter |
|---|---|---|
| ความยาวตัวถัง | 200 mm | `body_length_mm` |
| ความกว้างแผ่นแชสซี | 140 mm | `chassis_plate_width_mm` |
| **ความกว้างรวมล้อ** | **146 mm** | `body_width_mm` |
| ความสูงตัวถัง | 70 mm | `body_height_mm` |
| ระยะท้องรถ | 35 mm | `chassis_clearance_mm` |
| ความสูงรวม (ถึงกล้องล่าง) | 220 mm | `camera_down_height_mm` |

### ⚠️ `body_width` ≠ ความกว้างแชสซี

```text
body_width = max(chassis_plate_width, track_width + wheel_width)
           = max(140, 120 + 26)
           = 146 mm
```

ล้อยื่นออกนอกแชสซีข้างละ 3 mm — **ส่วนที่ชนใบพืชคือล้อ ไม่ใช่ตัวถัง**

`config/rover.yaml` `body_width_mm` ต้องเป็น **146** ไม่ใช่ 140 เพราะ
startup invariant ข้อ 5 ใช้ค่านี้เทียบกับความกว้างร่อง

นี่เป็นความผิดพลาดประเภทเดียวกับ `rail_length` ≠ `travel` ของ design gantry:
**ตัวเลขที่ดูเหมือนเป็นตัวเดียวกัน แต่ตัวที่ระบบต้องใช้คือตัวที่ใหญ่กว่า**

---

## Drive

| Dimension | ค่า | CAD parameter |
|---|---|---|
| Track width (กึ่งกลางล้อซ้าย–ขวา) | 120 mm | `track_width_mm` |
| Wheelbase (กึ่งกลางเพลาหน้า–หลัง) | 120 mm | `wheelbase_mm` |
| เส้นผ่านศูนย์กลางล้อ | 65 mm | `wheel_diameter_mm` |
| ความกว้างหน้ายาง | 26 mm | `wheel_width_mm` |

Footprint เป็น **สี่เหลี่ยมจัตุรัส 120 × 120 mm** — ทำให้ skid-steer เลี้ยว
คาดเดาได้สม่ำเสมอกว่า footprint ที่ยาวกว่ากว้าง (จุดหมุนนิ่งกว่า slip สมมาตรกว่า)

### ทำไม track_width เป็น 120 ไม่ใช่ 140

```text
track 140 → overall 166 → clear_furrow 290 > 286   margin  4 mm   ✗ แคบเกิน
track 120 → overall 146 → clear_furrow 290 > 266   margin 24 mm   ✓
```

ลด track ดีกว่าขยายแปลง เพราะได้ margin ของ deadband เพิ่มด้วย
(`v_left` ที่ `omega_max` ขยับจาก 51.1 → 58.1 mm/s) และกล้องล่างยังใช้ FOV 60°
มาตรฐานได้ ไม่ต้องเปลี่ยนไปเลนส์กว้าง

---

## Transmission — ขับตรง ไม่มีสายพาน

| รายการ | ค่า |
|---|---|
| Motor | DC gear motor 12 V |
| ความเร็วรอบ nominal | **~100 RPM** |
| การส่งกำลัง | ขับตรงที่เพลาล้อ (ไม่มี belt / pulley) |
| ความเร็วล้อสูงสุด | **340 mm/s** |
| ความเร็วใช้งาน | 100 mm/s (29% duty) |
| Deadband (ประมาณ) | ~51 mm/s (15% duty) — **ต้องวัดจริงที่ V3** |

```text
wheel_v_max = (motor_rpm / 60) × pi × wheel_diameter
            = (100 / 60) × pi × 65
            = 340 mm/s

ความเร็วใช้งาน 100 mm/s → 100 / 340 = 29% duty
```

### ⚠️ มอเตอร์ที่เร็วกว่า **แย่กว่า** ไม่ใช่ดีกว่า

นี่เป็นเรื่องที่ต่างจากงาน stepper ของ design gantry ที่ resolution เหลือเฟือ
ไม่เสียหาย

```text
มอเตอร์ 100 RPM → ความเร็วใช้งานอยู่ที่ 29% duty   ✓ คุมได้
มอเตอร์ 300 RPM → ความเร็วใช้งานอยู่ที่ 10% duty   ✗ อยู่ใน deadband ตลอดเวลา
```

มอเตอร์เกียร์ DC ไม่ออกตัวใต้ ~15% duty **ความเร็วมอเตอร์ควรอยู่ที่ 2–3 เท่า
ของความเร็วใช้งาน** ไม่ใช่มากที่สุดที่หาได้

ตัวจำกัดความแม่นยำของ rover ไม่ใช่ resolution ของมอเตอร์ — มันคือ **slip
ของ skid-steer** และ **ความแม่นของ row estimator** ซึ่งแก้ด้วยมอเตอร์เร็วกว่าไม่ได้

### Deadband กำหนด `omega_max`

```text
ที่ omega = 60 deg/s:  v_left = 100 − 62.8 = 37.2 mm/s  (11%)  ✗ deadband
ที่ omega = 40 deg/s:  v_left = 100 − 41.9 = 58.1 mm/s  (17%)  ✓

เพดานจริง: omega = 46.8 deg/s ทำให้ v_left แตะ 51 พอดี → เลือก 40 เพื่อมี margin
```

ล้อข้างในที่อยู่ใน deadband จะ **หยุดหมุนขณะที่ ESP32 คิดว่ากำลังสั่งให้หมุน**
rover จะเลี้ยวแรงกว่าที่สั่งโดยไม่มีสัญญาณบอก — ไม่มี encoder ไม่มีใครรู้

---

## Camera

| Dimension | ค่า | CAD parameter |
|---|---|---|
| กล้องหน้า — ความสูง | 180 mm | `camera_front_height_mm` |
| กล้องหน้า — มุมก้ม | 45° | `camera_front_tilt_deg` |
| กล้องหน้า — เลื่อนหน้า | 90 mm | `camera_front_offset_x_mm` |
| กล้องล่าง — ความสูง | 220 mm | `camera_down_height_mm` |
| กล้องล่าง — มุมก้ม | 0° (ตรงลง) | `camera_down_tilt_deg` |

### กล้องหน้า — lookahead

```text
lookahead = 180 / tan(45°) = 180 mm ข้างหน้า
เวลาที่ได้ = 180 / 100 = 1.8 s
```

**ไม่มีข้อกำหนด FOV เป็นองศา** เพราะ `RowEstimate` เป็นค่า image-space ล้วน —
ขอแค่เห็นแถวพืชสองข้างในเฟรมที่ระยะ lookahead

⚠️ แต่ตัวเลข lookahead นี้ **คือสิ่งที่ gain ของ row follower ถูก tune กับมัน**
ขยับขายึดกล้อง → gain ใช้ไม่ได้ → ต้อง tune ใหม่ที่ V1 และ V4
**ไม่มี test จับได้**

### กล้องล่าง — ต้องครอบร่อง

ที่ความสูง 220 mm ต้องครอบความกว้าง **250 mm** (corridor 210 + ขอบข้างละ 20):

```text
FOV แนวนอนที่ต้องการ >= 2 × atan(125 / 220) = 59.2°   →  ต้องการ >= 60°
FOV แนวตั้งที่ได้ ~45° → coverage ตามแนววิ่ง ≈ 182 mm
ระยะที่รถวิ่งต่อเฟรม = 100 / 2 Hz = 50 mm  →  182 > 50 ✓ มี overlap
```

ต้องมีขอบ ไม่ใช่ครอบแค่ corridor พอดี — กฎ `touches_side_edge` ที่ใช้ตัดใบพืชผลออก
ทำงานได้เฉพาะเมื่อใบพืชผลแตะขอบเฟรมจริง

Webcam ทั่วไป ~65–70° แนวทแยง ซึ่งแปลงเป็นแนวนอนแล้วอาจได้แค่ ~55° —
**ตรวจสเปกก่อนซื้อ** ดู [../bom/poc-v2.md](../bom/poc-v2.md#s2--กล้องล่าง-weed-detection)

### ⚠️ เสากล้อง 220 mm ทำให้จุดศูนย์ถ่วงสูง

เสาสูงกว่าตัวถัง 150 mm บนรถกว้างรวมล้อ 146 mm — **ต้องวางแบตเตอรี่และมอเตอร์
ให้ต่ำที่สุด** ไม่งั้นพลิกง่ายบนดินขรุขระ

ถ้า V1 แสดงว่าพลิกใน sim ให้แก้ที่การจัดวางมวล **ไม่ใช่ลดความเร็ว** —
`v_max` ผูกกับ `runaway_budget` invariant อยู่แล้ว

---

## Bed (แปลงทดสอบ)

```text
◄──────────────────── 2000 mm ────────────────────►

═════════════ crop row 1 ═════════════   ▲
                                          │ 350 mm
        ╔═══╗                             │
        ║ R ║  rover ในร่อง                │  ← clear furrow 290 mm
        ╚═══╝                             │
═════════════ crop row 2 ═════════════   ▼
                                          │ 350 mm
═════════════ crop row 3 ═════════════   ▼

ขอบแปลงยกสูงกว่า 33 mm  (> wheel_diameter / 2)
```

| Dimension | ค่า |
|---|---|
| ความยาว | 2000 mm |
| ความกว้าง | 1000 mm |
| จำนวนแถว | 3 |
| `row_spacing` | 350 mm |
| `crop_foliage_half_width` | 30 mm (ประมาณ — วัดที่ V1) |
| **clear furrow** | **290 mm** |
| `crop_spacing` ตามแถว | 80 mm |
| Edge margin | 150 mm |
| ความสูงขอบแปลง | > 33 mm |

### `row_spacing` ไม่ใช่ความกว้างที่ rover วิ่งได้

```text
clear_furrow = row_spacing − 2 × crop_foliage_half_width = 350 − 60 = 290 mm
ช่องว่างข้างละ = (290 − 146) / 2 = 72 mm
```

invariant ข้อ 5 ต้องการช่องว่างข้างละ ≥ `runaway_budget` (60 mm) → 72 ≥ 60 ✓

### ขอบแปลงยกไม่ใช่ของประดับ

MVP **ไม่มี bumper switch** ขอบยกเป็นสิ่งเดียวที่กัน rover ตกแปลงทางกายภาพ
ถ้าขอบไม่ยกสูงกว่ารัศมีล้อ rover จะไต่ข้ามไปได้

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
```

### สิ่งที่ test จับไม่ได้

```text
✗ gain ยังเหมาะกับ camera_front_tilt_deg ใหม่หรือไม่
✗ FOV กล้องจริงครอบ corridor ได้หรือไม่
✗ จุดศูนย์ถ่วงต่ำพอหรือไม่
```

สามข้อนี้ต้องอาศัยคนตรวจ — จึงถูกเขียนกำกับไว้ทุกที่ที่มันเกี่ยวข้อง
