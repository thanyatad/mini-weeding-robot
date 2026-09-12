# CAD Parameters

Fusion 360 user parameters — **ต้นทางของตัวเลขขนาดทุกตัว**

Export เป็น `parameters.csv` แล้ว commit ทุกครั้งที่แก้ขนาด

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
| `body_length_mm` | 200 | ความยาวตัวถัง (ไม่รวมล้อ) |
| `chassis_plate_width_mm` | 140 | ความกว้างแผ่นแชสซี |
| `body_width_mm` | **145** | **ความกว้างรวมล้อ = จุดกว้างสุดของรถ** ดูหมายเหตุ |
| `body_height_mm` | 70 | ความสูงตัวถัง (ไม่รวมเสากล้อง) |
| `chassis_clearance_mm` | 35 | จากพื้นถึงท้องรถ |

### Drive

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `track_width_mm` | 120 | ระยะกึ่งกลางล้อซ้ายถึงกึ่งกลางล้อขวา |
| `wheelbase_mm` | 120 | ระยะกึ่งกลางเพลาหน้าถึงกึ่งกลางเพลาหลัง |
| `wheel_diameter_mm` | 70 | เส้นผ่านศูนย์กลางล้อรวมยาง — Pololu #3272 |
| `wheel_width_mm` | 25 | ความกว้างหน้ายาง — Pololu #3272 |
| `motor_mount_pitch_mm` | 18 | ระยะรูยึดมอเตอร์ |

ไม่มี belt / pulley / stepper — ขับตรงจากมอเตอร์เกียร์ที่เพลาล้อ

### ⚠️ `body_width_mm` ต้องเป็นความกว้างรวมล้อ ไม่ใช่ความกว้างแชสซี

```text
body_width = max(chassis_plate_width, track_width + wheel_width)
           = max(140, 120 + 25)
           = 145 mm
```

ล้อยื่นออกนอกแชสซีข้างละ 3 mm — **ส่วนที่ชนใบพืชคือล้อ ไม่ใช่ตัวถัง**

ข้อนี้เป็นที่มาของการลด `track_width` จาก 140 เป็น 120: ที่ track 140
ความกว้างรวมจะเป็น 166 mm ทำให้ invariant ข้อ 5 เหลือ margin แค่ **4 mm**
ซึ่งน้อยเกินไปสำหรับค่าที่ยังไม่ได้วัดจริง (`crop_foliage_half_width`)

```text
track 140 → overall 165 → clear_furrow 290 > 285   margin  5 mm   ✗ แคบเกิน
track 120 → overall 145 → clear_furrow 290 > 265   margin 25 mm   ✓
```

ลด track ดีกว่าขยาย `row_spacing` เพราะได้ margin ของ deadband เพิ่มด้วย
(`v_left` ที่ `omega_max` ขยับจาก 51.1 เป็น 58.1 mm/s)

⚠️ `track_width 120` ยังกำหนด**ความยาวมอเตอร์**ด้วย: หน้าในของล้ออยู่ที่ ±47.5 mm
มอเตอร์ซ้าย/ขวายื่นเข้าหากัน ยาวเกิน 47 mm ต่อตัวคือชนกลางลำ ตัดมอเตอร์ยอดนิยม
อย่าง GA25-370 (~65 mm) และ JGB37 (~72 mm) ออกทั้งหมด ดู
[../../hardware/bom/poc-v2.md](../../hardware/bom/poc-v2.md) และกล้องล่างยังใช้
FOV 60° มาตรฐานได้ ไม่ต้องเปลี่ยนไปใช้เลนส์กว้าง

### Camera

| Parameter | ค่า | ความหมาย |
|---|---|---|
| `camera_front_height_mm` | 180 | ความสูงเหนือ soil reference |
| `camera_front_tilt_deg` | 45 | มุมก้มจากแนวนอน |
| `camera_front_offset_x_mm` | 90 | เลื่อนไปข้างหน้าจาก `base_link` |
| `camera_down_height_mm` | 220 | ความสูงเหนือ soil reference |
| `camera_down_tilt_deg` | 0 | 0 = มองตรงลง |
| `camera_down_offset_x_mm` | 0 | อยู่กึ่งกลางตัวรถ |

`camera_down_tilt_deg = 0` ตั้งใจ — ยิ่งเอียงยิ่งทำให้ error จากดินไม่เรียบใหญ่ขึ้น
เหตุผลเดียวกับกล้องของ design gantry ดู
[../../docs/calibration.md](../../docs/calibration.md#soil-plane-error-budget)

`camera_down_height_mm = 220` ทำให้เสากล้องสูงกว่าตัวถัง 150 mm บนรถกว้างรวมล้อ 145 mm —
**ต้องวางแบตเตอรี่และมอเตอร์ให้ต่ำที่สุด** ไม่งั้นจุดศูนย์ถ่วงสูงและพลิกง่ายบนดินขรุขระ

---

## Coverage Checks — คำนวณได้ ต้องตรวจก่อนสั่งกล้อง

สองข้อนี้ **ไม่ใช่ startup invariant** (ค่า FOV ไม่อยู่ใน config) แต่ต้องตรวจ
ก่อนซื้อของ และวัดจริงที่ V1

### กล้องล่างต้องครอบ corridor + ขอบ

```text
corridor                210 mm     (clear_furrow 290 − 2 × max_lateral_error 40)
ต้องครอบ + ขอบข้างละ 20    250 mm     ขอบไว้ให้ใบพืชผลโผล่มาแล้วถูกกฎ touches_side_edge ตัด

FOV แนวนอนที่ต้องการ  >= 2 × atan(125 / 220) = 59.2°   →  ต้องการ >= 60°
```

ต้องมีขอบ ไม่ใช่ครอบแค่ corridor พอดี — กฎ `touches_side_edge` ทำงานได้เฉพาะ
เมื่อใบพืชผล**แตะขอบเฟรมจริง** ถ้าเฟรมกว้างเท่า corridor พอดี ใบพืชผลจะไม่มีที่ยืน
แล้วจะถูกนับเป็นวัชพืช

### ต้องไม่มีช่องว่างระหว่างเฟรม

```text
coverage ตามแนววิ่ง  = 2 × 220 × tan(VFOV/2)  ≈ 182 mm   (ที่ VFOV 45°)
ระยะที่รถวิ่งต่อเฟรม  = v_max / down_rate_hz = 100 / 2  =  50 mm

182 > 50  ✓ มี overlap พอ ไม่มีวัชพืชหลุดระหว่างเฟรม
```

ถ้าเพิ่ม `v_max` หรือลด `down_rate_hz` ต้องคำนวณข้อนี้ใหม่ — recall ที่ตกเพราะ
ช่องว่างระหว่างเฟรมจะดูเหมือน detector ห่วย ทั้งที่เป็นปัญหา timing

### กล้องหน้า — lookahead

```text
lookahead  = camera_front_height_mm / tan(camera_front_tilt_deg)
           = 180 / tan(45°) = 180 mm ข้างหน้า

เวลาที่ได้ = 180 / v_max = 1.8 s
```

**ตัวเลขนี้คือสิ่งที่ gain ของ row follower ถูก tune กับมัน** เปลี่ยนความสูง
หรือมุม แล้ว gain ใช้ไม่ได้ — ไม่มี test จับได้ ต้อง tune ใหม่ที่ V1 และ V4

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
| `wheel_diameter_mm` | `config/rover.yaml` · `cad/urdf` wheel radius | radius = `d/2` (เมตร) |
| `wheel_diameter_mm` + BOM `motor_rpm` | `config/rover.yaml` `wheel_v_max_mm_s` | `(rpm/60) × pi × d` |
| `body_width_mm` | `config/rover.yaml` `rover.body_width_mm` | เท่ากัน — **ใช้ใน invariant ข้อ 5** |
| `chassis_clearance_mm` | `config/rover.yaml` | เท่ากัน — **ใช้ใน invariant ข้อ 4** |
| `camera_front_*` | `config/camera.yaml` extrinsic | ค่าเริ่มต้น (กล้องหน้าไม่ calibrate) |
| `camera_down_*` | `config/camera.yaml` extrinsic | ค่าเริ่มต้นก่อน calibrate |
| `camera_front_height_mm` + `tilt_deg` | `config/control.yaml` **gain** | **ไม่มีสูตร — ต้อง tune** |

### `wheel_v_max` ตรวจสอบได้

```text
wheel_v_max = (motor_rpm / 60) × pi × wheel_diameter_mm
            = (55 / 60) × 3.14159 × 70
            = 0.9167 × 219.9
            = 201.6 mm/s   →  config: wheel_v_max_mm_s: 202
```

### Startup invariants ที่ค่าจาก CAD เข้าไปเกี่ยวข้อง

```text
geometry  wheel_diameter        >= 4 × soil_variation                  70 >= 60   ok  margin 10 mm
geometry  chassis_clearance     >  soil_variation                      35 >  15   ok
geometry  clear_furrow          >  body_width + 2 × runaway_budget    290 > 265   ok  margin 25 mm
drive     v + omega_max_rad × track/2  <= wheel_v_max               141.9 <= 202  ok
drive     v − omega_max_rad × track/2  >= wheel_v_min                58.1 >=  51
```

⚠️ **`wheel_diameter >= 4 × soil_variation` เหลือ margin 10 mm** —
ถ้า `soil_variation_mm` ที่วัดจริงเกิน 17.5 mm ต้องเปลี่ยนล้อใหญ่ขึ้น
ล้อ 70 mm บนก้อนดิน 15 mm คือการไต่สิ่งกีดขวางสูง 21% ของเส้นผ่านศูนย์กลาง
ซึ่งเป็นขอบของสิ่งที่ล้อไม่มีช่วงล่างทำได้

⚠️ **invariant `drive` ตัวล่างมี margin 7.1 mm/s** — `wheel_v_min_mm_s = 51`
เป็นค่าประมาณจาก 15% duty **ยังไม่ได้วัด** ถ้าวัดจริงที่ V3 ได้เกิน **58 mm/s**
ต้องลด `omega_max` (เพดานคือ 46 deg/s) ห้ามลด `wheel_v_min`

---

## ค่าที่ตั้งไว้และควรตรวจสอบก่อนสร้างจริง

ตัวเลขในเอกสารนี้เป็น **ค่าเริ่มต้นที่สอดคล้องกัน** ไม่ใช่ค่าที่วัดจากของจริง

| ต้องยืนยัน | ที่ไหน | ถ้าผิดแล้วเกิดอะไร |
|---|---|---|
| `motor_rpm` ~100 ไม่ใช่ 300 | ก่อนสั่งมอเตอร์ | 300 RPM ทำให้ความเร็วใช้งานตกไปอยู่ที่ 10% duty = อยู่ใน deadband ตลอดเวลา |
| `wheel_v_min` ที่วัดได้ | V3 | `omega_max` สูงเกิน ล้อข้างในหยุดเงียบ rover เลี้ยวแรงกว่าที่สั่ง |
| FOV กล้องล่าง >= 60° ที่ 220 mm | ก่อนสั่งกล้อง | เห็นร่องไม่ครบ ใบพืชผลไม่แตะขอบเฟรม → นับเป็นวัชพืช |
| `crop_foliage_half_width_mm` = 30 | V1 (asset พืช) | กระทบทั้ง invariant ข้อ 5 และ corridor พร้อมกัน |
| `soil_variation_mm` <= 16 | V4 (แปลงจริง) | ล้อ 65 mm ไม่พอ ต้องเปลี่ยนล้อ |

ความเร็วมอเตอร์ควรอยู่ที่ **2–3 เท่า** ของความเร็วใช้งาน ไม่ใช่มากที่สุดที่หาได้ —
นี่เป็นเรื่องที่ต่างจากงาน stepper ที่เร็วกว่าไม่เสียหาย

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
  track_width_mm: 120          # derived: cad track_width
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
