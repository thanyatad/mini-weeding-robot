# cad

Mechanical source of truth — **เรขาคณิตทั้งหมดต้นทางที่ `cad/parameters/parameters.csv`**
งานผลิต (STEP / STL) มาจาก Fusion โดยตรง แต่ **ไม่ใช่ต้นทางของ geometry ที่ sim ใช้**
ดู [Pipeline](#pipeline) ด้านล่าง

```text
cad/
├── fusion/
│   ├── rover.f3d               master assembly (Fusion 360)
│   └── components/             ชิ้นส่วนย่อย
│
├── parameters/
│   ├── README.md               นิยาม parameter + ตาราง derived
│   └── parameters.csv          export จาก Fusion (commit ทุกครั้งที่แก้ขนาด)
│
├── exports/
│   ├── step/                   งานผลิต / review / ส่งร้าน
│   ├── stl/                    3D print
│   └── meshes/                 simplified visual + collision สำหรับ sim
│
└── urdf/
    ├── weeding_rover.urdf      links, joints, limits, mesh refs
    └── meshes/                 mesh ที่ URDF อ้างถึง
```

---

## Pipeline

`cad/parameters/parameters.csv` **คือต้นทางของ geometry** — ไม่ใช่ `rover.f3d`
สอง path ไหลออกจากมันแยกกัน และไม่มี path ไหนขึ้นกับอีก path (S11):

```text
cad/parameters/parameters.csv          ◄── source of truth: geometry
   │
   ├── tools/generate_sim_meshes.py ──> cad/exports/meshes/ · cad/urdf/meshes/
   │                                     (generated — ห้ามแก้ด้วยมือ)
   │                                          │
   │                                          ▼
   │                                   cad/urdf/weeding_rover.urdf
   │                                          │
   │                                          │  Isaac URDF importer  →  scripts/import_urdf.sh
   │                                          ▼
   │                    sim/isaac/robots/rover/rover_base.usd    ◄── generated, gitignored
   │                                          │
   │                                          │  + USD layer
   │                                          ▼
   │                    sim/isaac/robots/rover/rover.usd         ◄── override layer, commit
   │
   └── Fusion 360 (งานมือ)  cad/fusion/rover.f3d ──> cad/exports/step/ · cad/exports/stl/
                                                       (สำหรับผลิต / review / ส่งร้าน เท่านั้น)
```

`tools/generate_sim_meshes.py` อ่าน `parameters.csv` แล้วเขียน mesh + inertia
tensor ให้ sim โดยตรง — ไม่ผ่าน Fusion เลย เพราะ Fusion เป็นงานมือ ถ้า sim
ต้องรอ assembly งาน development จะหยุด `cad/exports/meshes/` และ
`cad/urdf/meshes/` จึงเป็นไฟล์ **generated** ห้ามแก้ด้วยมือ — แก้ที่
`parameters.csv` แล้วรัน generator ใหม่เสมอ

`rover.f3d` ยังเป็น**พารามิเตอร์ assembly ของจริง** และเป็นเจ้าของ
`cad/exports/step/` กับ `cad/exports/stl/` สำหรับงานผลิต แต่**ไม่ใช่ต้นทางของ
geometry ที่ sim ใช้** — ตามมาทีหลังได้โดยไม่บล็อกใคร (design §9.1)

### ทำไมต้องแยก 2 USD layer

URDF เก็บ **drive gain, damping, contact offset, friction, sensor attachment**
ของ Isaac ไม่ได้

ถ้าแก้ค่าพวกนี้ลงใน USD ที่ import มาโดยตรง แล้ววันหนึ่ง CAD เปลี่ยนแล้ว regenerate
→ tuning ทั้งหมดหาย

การแยก layer ทำให้ **regenerate `rover_base.usd` ได้ตลอดเวลาโดยไม่แตะ tuning**

| ไฟล์ | สถานะ | เจ้าของ |
|---|---|---|
| `rover_base.usd` | generated, **gitignored** | URDF importer |
| `rover.usd` | override layer, **commit** | คนปรับ physics ใน Isaac |

Friction ของล้อกับ heightfield เป็นค่าที่ tune ยากที่สุดของโปรเจกต์นี้ —
มันอยู่ใน `rover.usd` และเสียไปตอน regenerate ไม่ได้

---

## กฎการแก้ขนาด

เมื่อขนาดทางกายภาพเปลี่ยน ต้องทำครบทั้ง 4 ขั้น:

1. แก้ parameter ใน Fusion (ไม่ใช่แก้ sketch ตรง ๆ)
2. Export `parameters.csv` ทับของเดิม แล้ว commit
3. รัน `tools/generate_sim_meshes.py` เพื่อ regenerate `cad/exports/meshes/`
   และ `cad/urdf/meshes/` — **ไม่ใช่ export mesh มือจาก Fusion**
   ส่วน STEP/STL งานผลิตยัง export จาก Fusion ตามปกติเมื่อจำเป็น
4. Regenerate URDF → base USD (Isaac importer)

แล้วรัน:

```bash
pytest tests/unit/test_cad_config_sync.py
```

ถ้า test fail แปลว่ามีค่าใน `config/` ที่ derived มาจาก CAD แล้วยังไม่ได้อัปเดต
ดูตาราง derived ใน [parameters/README.md](parameters/README.md)

**อย่าแก้ค่าใน `config/rover.yaml` เพื่อให้ test ผ่าน** ถ้าต้นเหตุคือ CAD เปลี่ยน —
ต้องตามให้ค่าใน config สะท้อนของจริง

---

## ⚠️ พารามิเตอร์ที่แก้แล้วกระทบมากกว่าที่คิด

rover มี coupling ระหว่าง CAD กับ software ที่ gantry ไม่มี

| แก้ค่านี้ | กระทบอะไรนอกจากรูปร่าง |
|---|---|
| `camera_front_tilt_deg` | **gain ของ row follower ใช้ไม่ได้** — มุมกำหนด lookahead distance |
| `camera_front_height_mm` | เหมือนกัน — ความสูงก็กำหนด lookahead |
| `wheel_diameter_mm` | `wheel_v_max_mm_s` เปลี่ยน → พิสัยของ `omega_max` เปลี่ยน → **invariant `drive` ทั้งสองข้อ** |
| `track_width_mm` | สูตร mixing เปลี่ยน → `config/drive_mixing_vectors.csv` ต้อง regenerate ทั้งตาราง |
| `body_width_mm` | **invariant ข้อ 5** (`clear_furrow > body_width + 2 × runaway_budget`) |
| `chassis_clearance_mm` | **invariant ข้อ 4** (`> soil_variation`) |
| `camera_down_height_mm` | coverage ของ corridor — ถ้าต่ำเกินจะเห็นไม่ครบร่อง |
| `chassis_plate_width_mm` | ถ้าเกิน track − wheel_width โครงชนล้อ — `test_the_chassis_plate_clears_the_inner_faces_of_the_wheels` |

`camera_front_tilt_deg` เป็นตัวที่เจ็บที่สุด เพราะ **ไม่มี test จับได้** —
`test_cad_config_sync.py` ตรวจว่าตัวเลขตรงกัน แต่ตรวจไม่ได้ว่า gain ยังเหมาะกับมุมใหม่
ต้องกลับไป tune ที่ V1 และ V4 ใหม่ทั้งคู่

(กล้องของ gantry มองตรงลงและอยู่กับที่ มุมไม่มีผลต่อ control เลย)

---

## Git LFS

ไฟล์ binary ในโฟลเดอร์นี้โตเร็วและ diff ไม่ได้ ควรใช้ Git LFS

```bash
git lfs install
git lfs track "*.f3d" "*.step" "*.stp" "*.stl" "*.usd" "*.usdc" "*.obj" "*.dae"
git add .gitattributes
```

ตั้งแต่ commit แรกที่มีไฟล์จริง — ย้ายทีหลังต้องเขียนประวัติใหม่ทั้ง repo

`parameters.csv` และ `.urdf` เป็น text **ไม่ต้องใส่ LFS** — ต้องการ diff ที่อ่านได้
เพราะมันคือที่ที่การเปลี่ยนขนาดจะปรากฏตอน review

---

## Boundary

✅ CAD เป็นเจ้าของ: track width, wheelbase, ขนาดล้อ, ความสูงท้องรถ, ความกว้างตัวถัง,
ตำแหน่งและมุมขายึดกล้อง, ตำแหน่งรูยึดมอเตอร์, mass / inertia

❌ CAD **ไม่** เป็นเจ้าของ: `v_max`, `omega_max`, timeout, gain, threshold,
`soil_variation_mm`, `runaway_budget_mm` — พวกนี้เป็น **operational** อยู่ใน `config/`

เส้นแบ่ง: *ถ้าวัดด้วยเวอร์เนียร์ได้ = CAD · ถ้าเปลี่ยนได้โดยไม่ต้องถอดสกรู = config*

### กรณีชายขอบ: `wheel_v_max_mm_s` และ `wheel_v_min_mm_s`

สองค่านี้ไม่ใช่ทั้ง CAD แท้และ config แท้:

```text
wheel_v_max_mm_s = (motor_rpm / 60) × pi × wheel_diameter_mm
                 = (25 / 60) × pi × 250
                 = 327 mm/s
```

`wheel_diameter_mm` เป็น CAD · `motor_rpm` เป็นสเปกของของที่ซื้อ (BOM)
ค่าผลลัพธ์อยู่ใน `config/rover.yaml` พร้อม comment ชี้ทั้งสองต้นทาง

`wheel_v_min_mm_s` (deadband) **วัดจากมอเตอร์จริงเท่านั้น** คำนวณไม่ได้ —
ค่า 51 ใน config เป็นค่าประมาณจาก 15% duty ต้องวัดจริงที่ V3
