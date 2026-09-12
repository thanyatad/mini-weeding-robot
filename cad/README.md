# cad

> ### ⚠️ `cad/fusion/rover_base_v0.f3d` เป็น snapshot ไม่ใช่ working file
>
> Rover Base V0 (650 × 520, ล้อ Ø250) มี assembly แล้ว `exports/step/` และ
> `exports/stl/` ไม่ว่างอีกต่อไป
>
> ```text
> parameters.csv · urdf/ · exports/meshes/   ✓ generated จาก CSV
> fusion/ · exports/step/ · exports/stl/     ✓ มาจาก CAD assembly
> ```
>
> แต่ **ตัวที่แก้ได้อยู่บน Autodesk cloud ไม่ใช่ไฟล์ใน repo** — Fusion save ลง
> cloud project เสมอ ไม่เคย save กลับมาที่นี่:
>
> ```text
> เปิด .f3d  →  Fusion upload เข้า cloud  →  ได้ document ใหม่ คนละตัวกับไฟล์นี้
> กด Save    →  ลง cloud                  →  ไฟล์ใน repo ไม่ขยับ
> ```
>
> **แก้ assembly แล้วต้อง export ทับทั้ง 3 ที่เสมอ** (`fusion/` · `exports/step/` ·
> `exports/stl/`) ไม่งั้นมัน stale เงียบ ๆ — `7e91e59` ลบ `rover.f3d` ตัวเก่าทิ้ง
> เพราะค้างอยู่ที่รถ 200 × 145 ล้อ Ø70 ทั้งที่โปรเจกต์ย้ายไป 650 × 520 แล้ว
>
> `parameters.csv` ยังเป็นต้นทางของ geometry เหมือนเดิม — `.f3d` เก็บไว้กู้ **งาน
> ประกอบ** (user parameter, sketch, constraint) ไม่ใช่กู้ตัวเลขขนาด
>
> `exports/meshes/` **ไม่เกี่ยวกับงานนี้เลย** — `tools/generate_sim_meshes.py`
> สร้างจาก `parameters.csv` ให้แล้ว

Mechanical source of truth — **เรขาคณิตทั้งหมดต้นทางที่ `cad/parameters/parameters.csv`**
งานผลิต (STEP / STL) มาจาก CAD assembly ใน `fusion/` ดู [Pipeline](#pipeline) ด้านล่าง

```text
cad/
├── parameters/
│   ├── README.md               นิยาม parameter + ตาราง derived
│   └── parameters.csv          ต้นทางของ geometry — แก้ตรงนี้ (commit ทุกครั้งที่แก้ขนาด)
│
├── fusion/
│   └── rover_base_v0.f3d       snapshot ของ assembly — ตัวจริงบน cloud (ดูกล่องบนสุด)
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

`cad/parameters/parameters.csv` **คือต้นทางของ geometry** — ไม่ใช่ CAD assembly
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
   └── CAD assembly (งานมือ, บน cloud) ─> cad/fusion/rover_base_v0.f3d
                                          cad/exports/step/ · cad/exports/stl/
                                             (สำหรับผลิต / review / ส่งร้าน เท่านั้น)
```

`tools/generate_sim_meshes.py` อ่าน `parameters.csv` แล้วเขียน mesh + inertia
tensor ให้ sim โดยตรง — ไม่ผ่าน CAD assembly เลย เพราะงาน assembly เป็นงานมือ
ถ้า sim ต้องรอ งาน development จะหยุด `cad/exports/meshes/` และ
`cad/urdf/meshes/` จึงเป็นไฟล์ **generated** ห้ามแก้ด้วยมือ — แก้ที่
`parameters.csv` แล้วรัน generator ใหม่เสมอ

**path ล่างมีของแล้ว** assembly ขึ้นรูปจาก `parameters.csv` และผูก user parameter
ไว้ทุกตัว แต่มันยังตามมาทีหลังได้โดยไม่บล็อกใคร (design §9.1) — นี่คือเหตุผลที่ sim
ไม่ถูกผูกกับมันตั้งแต่แรก และยังไม่ควรผูก

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

เมื่อขนาดทางกายภาพเปลี่ยน ต้องทำครบทั้ง 3 ขั้นนี้ — ไม่ต้องรอ Fusion:

1. แก้ `cad/parameters/parameters.csv` โดยตรง แล้ว commit — มันคือต้นทางของ
   geometry (ดู [Pipeline](#pipeline) ด้านบน) **ไม่ใช่ export จาก Fusion**
2. รัน `tools/generate_sim_meshes.py` เพื่อ regenerate `cad/exports/meshes/`
   และ `cad/urdf/meshes/` — **ไม่ใช่ export mesh มือจาก Fusion**
3. Regenerate URDF → base USD (Isaac importer)

แล้วรัน:

```bash
pytest tests/unit/test_cad_config_sync.py       # จับ config/ ที่ยังไม่ตามขนาดใหม่
pytest tests/unit/test_generate_sim_meshes.py   # จับ mesh ที่ยังไม่ regenerate
pytest tests/unit/test_urdf_matches_cad.py      # จับ URDF ที่ยังอ้างขนาดเก่า
```

ทั้งสามข้อจับคนละจุดพัง เปลี่ยนขนาดครั้งเดียวอาจทำให้ข้อใดข้อหนึ่งพังโดยที่อีกสองข้อยังผ่าน
ต้องรันครบทั้งสาม — ไม่ใช่แค่ข้อแรก

ถ้า `test_cad_config_sync.py` fail แปลว่ามีค่าใน `config/` ที่ derived มาจาก CAD แล้วยังไม่ได้อัปเดต
ดูตาราง derived ใน [parameters/README.md](parameters/README.md)

**อย่าแก้ค่าใน `config/rover.yaml` เพื่อให้ test ผ่าน** ถ้าต้นเหตุคือ CAD เปลี่ยน —
ต้องตามให้ค่าใน config สะท้อนของจริง

### งานผลิต — แยกต่างหาก ไม่บล็อกงาน sim

STEP/STL งานผลิตเป็นคนละ track จาก 3 ขั้นข้างบน และตามทีหลังได้เสมอโดยไม่บล็อกใคร
(design §9.1) — sim ไม่ต้องรอ assembly

ขั้นตอนเมื่อแก้ assembly (เปิดตัวบน cloud ไม่ใช่ `.f3d` ใน repo):

1. แก้ผ่าน **user parameter** เท่านั้น — อย่าพิมพ์ตัวเลขลง sketch และอย่าใช้
   Move/Copy กับ body ทั้งสองอย่างตัดชิ้นนั้นขาดจาก `parameters.csv`
2. Export `cad/fusion/rover_base_v0.f3d` ทับ — **ขั้นนี้ลืมบ่อยที่สุด**
3. Export `cad/exports/step/` ชิ้นที่กระทบ
4. Export `cad/exports/stl/` เฉพาะชิ้นที่ต้องพิมพ์ใหม่

assembly เป็นเจ้าของ `fusion/`/`step/`/`stl/` เท่านั้น — ไม่ใช่ต้นทางของ
geometry และไม่มีอะไรใน sim ขึ้นกับมัน

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
