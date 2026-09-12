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
  ├── camera_front_link       fixed        tilt 45°, สูง 180 mm, เลื่อนหน้า 90 mm
  └── camera_down_link        fixed        tilt 0°,  สูง 220 mm
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
| `wheel_fl` | `(+0.060, +0.060, −0.0025)` | `+wheelbase/2`, `+track_width/2` |
| `wheel_fr` | `(+0.060, −0.060, −0.0025)` | `+wheelbase/2`, `−track_width/2` |
| `wheel_rl` | `(−0.060, +0.060, −0.0025)` | `−wheelbase/2`, `+track_width/2` |
| `wheel_rr` | `(−0.060, −0.060, −0.0025)` | `−wheelbase/2`, `−track_width/2` |
| `camera_front` | `(+0.090, 0, +0.180)` | `camera_front_offset_x`, `camera_front_height` |
| `camera_down` | `(0, 0, +0.220)` | `camera_down_height` |

`base_link` origin อยู่กึ่งกลางตัวรถที่ระดับ soil reference plane
ดังนั้น `z` ของเพลาล้อ = `wheel_diameter/2 − chassis_clearance − body_height/2`
ตามที่ assembly จัดวาง — ค่าจริงต้อง export จาก Fusion ไม่ใช่คำนวณมือ

```text
wheelbase 120 mm    → ±0.060 m
track_width 120 mm  → ±0.060 m

ล้อทั้งสี่อยู่ที่ `(±0.060, ±0.060)` — footprint เป็นสี่เหลี่ยมจัตุรัส
ซึ่งทำให้ skid-steer เลี้ยวคาดเดาได้สม่ำเสมอกว่า footprint ที่ยาวกว่ากว้าง
```

---

## Joint Limits

| Joint | Type | Limit | ที่มา |
|---|---|---|---|
| `wheel_*` | `continuous` | **ไม่มี** position limit | ล้อหมุนได้ไม่จำกัด |
| `wheel_*` | velocity limit | 10.5 rad/s | `wheel_v_max / (wheel_diameter/2)` |
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
                = 340 / 32.5
                = 10.46 rad/s   →  urdf: 10.5
```

ค่านี้ต้องสอดคล้องกับ `config/rover.yaml` `wheel_v_max_mm_s` — ถ้า URDF จำกัดต่ำกว่า
mixing จะสั่งค่าที่ Isaac ปฏิเสธเงียบ ๆ แล้ว `omega` ที่ได้จะไม่ตรงกับที่สั่ง
(อาการเดียวกับ deadband ของมอเตอร์จริง แต่เกิดใน sim)

---

## Unit Convention

⚠️ URDF ใช้ **เมตรและเรเดียน** ส่วน `config/*.yaml` ใช้ **มิลลิเมตรและองศา**

การแปลงหน่วยผิดตรงนี้เป็น bug ที่หายาก เพราะระบบยังวิ่งได้ แค่ระยะผิดพันเท่า

```text
track_width_mm: 120        →  urdf y = ±0.060
wheel_diameter_mm: 65      →  urdf radius = 0.0325
omega_max_deg_s: 40        →  0.698 rad/s
camera_front_tilt_deg: 45  →  urdf rpy pitch = 0.785 rad
```

จุดแปลงหน่วยมีแค่ 2 ที่ และต้องมี unit test ทั้งคู่ —
ดู [../../hardware/mechanical/coordinate-frames.md](../../hardware/mechanical/coordinate-frames.md#unit-convention)

---

## Collision

| Link | Collision geometry |
|---|---|
| `base_link` | box — ง่ายและเสถียรที่สุด |
| `wheel_*` | **cylinder** ไม่ใช่ mesh |
| `camera_*` | ไม่มี collision |

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
