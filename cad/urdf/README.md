# cad/urdf

Robot description ที่เชื่อม CAD เข้ากับ simulation

```text
cad/urdf/
├── weeding_rover.urdf      links, joints, limits, mesh refs
└── meshes/                 mesh ที่ URDF อ้างถึง (copy/symlink จาก ../exports/meshes)
```

---

## Kinematic Chain

ต้องตรงกับ [../../docs/architecture.md](../../docs/architecture.md) และ
[../../hardware/mechanical/coordinate-frames.md](../../hardware/mechanical/coordinate-frames.md)

```text
base_link                     ตัวถัง rover
  │
  ├── wheel_fl_link           continuous   ล้อหน้าซ้าย
  ├── wheel_fr_link           continuous   ล้อหน้าขวา
  ├── wheel_rl_link           continuous   ล้อหลังซ้าย
  ├── wheel_rr_link           continuous   ล้อหลังขวา
  │
  ├── camera_front_link       fixed        tilt 50°, สูง 850 mm, เลื่อนหน้า 0 mm (กึ่งกลาง mast)
  └── camera_down_link        fixed        tilt 0°,  สูง 850 mm
```

**โครงสร้างแบบดาว ไม่ใช่โซ่** — ต่างจาก gantry ที่เป็น
`base → x_carriage → y_carriage → tool` ซ้อนกันเป็นชั้น

ล้อทั้ง 4 เป็นลูกของ `base_link` โดยตรง และไม่มีล้อไหนเป็นพ่อของอะไร
skid-steer ไม่มีข้อต่อบังคับเลี้ยว — ทิศทางเกิดจากความต่างความเร็ว ไม่ใช่จากมุมข้อต่อ

กล้องทั้งสองตัวต่อกับ `base_link` แบบ **fixed** — ขยับตาม rover ทั้งคู่
(ต่างจาก gantry ที่กล้องอยู่กับที่ไม่ขยับตาม carriage)

---

## Joint Origins

| Joint | Origin (x, y, z) เมตร | ที่มา |
|---|---|---|
| `wheel_fl` | `(+0.200, +0.215, 0.125)` | `+wheelbase/2`, `+track_width/2`, `chassis_clearance` |
| `wheel_fr` | `(+0.200, −0.215, 0.125)` | `+wheelbase/2`, `−track_width/2`, `chassis_clearance` |
| `wheel_rl` | `(−0.200, +0.215, 0.125)` | `−wheelbase/2`, `+track_width/2`, `chassis_clearance` |
| `wheel_rr` | `(−0.200, −0.215, 0.125)` | `−wheelbase/2`, `−track_width/2`, `chassis_clearance` |
| `camera_front` | `(0, 0, 0.850)` | `camera_front_offset_x` (= 0, กึ่งกลาง mast), `camera_front_height` |
| `camera_down` | `(0, 0, 0.850)` | `camera_down_height` — mast เดียวกับ camera_front |

`base_link` origin อยู่กึ่งกลางตัวรถที่ระดับ soil reference plane (rover frame,
`hardware/mechanical/coordinate-frames.md`) ดังนั้น `z` ของเพลาล้อ = `chassis_clearance_mm`
พอดี (125 mm = รัศมีล้อ) — ท้องรถอยู่ที่ระดับเพลา ไม่ใช่สูตรอ้อมผ่าน `body_height`
เหมือนเอกสารรุ่นก่อน scale-up

```text
wheelbase 400 mm    → ±0.200 m
track_width 430 mm  → ±0.215 m

ล้อทั้งสี่อยู่ที่ `(±0.200, ±0.215)` — footprint 430 × 400 mm ใกล้จัตุรัสแต่ไม่เป๊ะ
สัดส่วน track:wheelbase = 430:400 ยังเหมาะกับ skid-steer อยู่ (design spec §3.1)
```

---

## Joint Limits

| Joint | Type | Limit | ที่มา |
|---|---|---|---|
| `wheel_*` | `continuous` | **ไม่มี** position limit | ล้อหมุนได้ไม่จำกัด |
| `wheel_*` | velocity limit | 2.616 rad/s | `wheel_v_max_mm_s / (wheel_diameter_mm/2)` |
| `wheel_*` | effort limit | 10.0 N·m | เพดานแรงบิดต่อเนื่องของเกียร์ (`hardware/bom/poc-v3.md`) |
| `camera_*` | `fixed` | — | — |

### `continuous` ไม่ใช่ `revolute`

```xml
<joint name="wheel_fl" type="continuous">
```

ถ้าใช้ `revolute` ต้องมี `lower`/`upper` แล้ว Isaac จะหยุดล้อที่ขอบ limit —
อาการคือ rover วิ่งไปได้ระยะหนึ่งแล้วหยุดโดยไม่มี error ซึ่งจะถูกเข้าใจผิดว่าเป็น
ปัญหา friction หรือ command timeout

### Velocity limit ตรวจสอบได้

```text
wheel_rad_s_max = wheel_v_max_mm_s / (wheel_diameter_mm / 2)
                = 327 / 125
                = 2.616 rad/s   →  urdf: 2.616
```

ค่านี้ต้องสอดคล้องกับ `config/rover.yaml` `wheel_v_max_mm_s` — ถ้า URDF จำกัดต่ำกว่า
mixing จะสั่งค่าที่ Isaac ปฏิเสธเงียบ ๆ แล้ว `omega` ที่ได้จะไม่ตรงกับที่สั่ง
(อาการเดียวกับ deadband ของมอเตอร์จริง แต่เกิดใน sim)

---

## Unit Convention

⚠️ URDF ใช้ **เมตรและเรเดียน** ส่วน `config/*.yaml` ใช้ **มิลลิเมตรและองศา**

การแปลงหน่วยผิดตรงนี้เป็น bug ที่หายาก เพราะระบบยังวิ่งได้ แค่ระยะผิดพันเท่า

```text
track_width_mm: 430        →  urdf y = ±0.215
wheel_diameter_mm: 250     →  urdf radius = 0.125
omega_max_deg_s: 25        →  0.436 rad/s
camera_front_tilt_deg: 50  →  0.873 rad (ก่อนรวมกับ optical-frame offset ใน rpy จริงของ joint)
```

จุดแปลงหน่วยมีแค่ 2 ที่ และต้องมี unit test ทั้งคู่ —
ดู [../../hardware/mechanical/coordinate-frames.md](../../hardware/mechanical/coordinate-frames.md#unit-convention)

---

## Collision

| Link | Collision geometry |
|---|---|
| `base_link` | **สามชิ้น ไม่ใช่กล่องเดียว**: กล่องโครงช่วงล้อ `0.500 × 0.340 × 0.125` ที่ `z 0.1875` (z 0.125–0.250, ระดับล้อ) · กล่อง body shell `0.500 × 0.380 × 0.155` ที่ `z 0.3275` (z 0.250–0.405, เหนือหลังคาล้อ) · กระบอก mast รัศมี `0.0175 × 0.500` ที่ `z 0.655` (z 0.405–0.905) |
| `wheel_*` | **cylinder** (รัศมี 0.125 × ยาว 0.090) ไม่ใช่ mesh |
| `camera_*` | ไม่มี collision |

**ยังเป็น primitive ล้วน ไม่มี mesh** — แต่ตอนนี้ `base_link` ต้องเป็น**สามชิ้น**
เพราะรูปทรงจริงไม่ใช่กล่องเดียว: โครงช่วงล้อ (340 mm) แคบกว่า body shell (380 mm)
ที่ยื่นคร่อมได้เพราะอยู่เหนือหลังคาล้อ (ดู
[../../hardware/mechanical/dimensions.md](../../hardware/mechanical/dimensions.md))
ส่วน mast เป็นท่อบางแยกต่างหากที่รับกล้องทั้งสองตัว กล่องเดียวแบบเดิมจะครอบคลุม
ทั้งสามช่วงความกว้างไม่ได้โดยไม่ชนล้อหรือเกินตัวถังจริง

**ล้อต้องเป็น cylinder primitive** — mesh ล้อที่มีดอกยางจะสร้าง contact point
จำนวนมากบน heightfield ทำให้ solver ช้าลงมากและไม่เสถียร โดยไม่ได้ความแม่นยำ
ที่มีความหมาย (grip จริงมาจาก friction coefficient ไม่ใช่รูปดอกยาง)

นี่เป็นจุดที่ต่างจาก design gantry ซึ่งยกเว้นให้หัว tool ใช้ mesh ได้
เพราะรูปทรงหัว tool มีผลกับการสัมผัสดินจริง — ล้อไม่ใช่กรณีนั้น

---

## Import เข้า Isaac

```bash
./scripts/import_urdf.sh
```

สร้าง `sim/isaac/robots/rover/rover_base.usd` (generated, gitignored)
แล้ว `rover.usd` เป็น layer ทับ

**อย่าแก้ `rover_base.usd` โดยตรง** — มันจะถูกเขียนทับทุกครั้งที่ import
ดู [../README.md](../README.md#ทำไมต้องแยก-2-usd-layer)

---

## สิ่งที่ URDF เก็บไม่ได้

ต้องไปอยู่ใน `rover.usd` override layer:

```text
drive gain / stiffness / damping ของ wheel joint
friction ของล้อกับ heightfield        ◄── ค่าที่ tune ยากที่สุดของโปรเจกต์
contact offset / rest offset
articulation solver settings + substep
sensor attachment (2 กล้อง)
camera intrinsic / FOV
```

`friction` เป็นตัวที่สำคัญที่สุด: ต้องพอให้ขับไปข้างหน้าได้ แต่ต้องให้ slip ได้
ตอนเลี้ยว skid-steer ถ้าตั้งสูงเกินจะเลี้ยวไม่ออกหรือ solver ระเบิด

เหตุผลที่ต้องแยก layer ไม่ใช่ความสวยงามของ pipeline — มันคือการไม่ทำ tuning
ชุดนี้หายตอน regenerate จาก CAD
