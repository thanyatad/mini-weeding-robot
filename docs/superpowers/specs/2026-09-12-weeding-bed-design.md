> **⚠️ SUPERSEDED — 2026-09-12** โดย
> [2026-09-12-rover-mvp-design.md](2026-09-12-rover-mvp-design.md)
>
> เอกสารนี้เก็บไว้เป็นบันทึกการตัดสินใจเดิม **เนื้อหาไม่ถูกแก้**
>
> Platform เปลี่ยนจาก **gantry คร่อมแปลง** เป็น **rover skid-steer 4 ล้อ** ถาวร
> (พลิก D1) — ไม่ใช่การเลื่อน
>
> ### ส่วนที่ตายแล้ว
>
> D1 (gantry ไม่ใช่ rover) · D2 (servo ไม่มีแกน Z) · D3 (workspace 600×400) ·
> D8 (ชื่อ gantry) · §4 Scene Model · §5 Tool Reach · §7 Protocol ·
> §11 Files Touched
>
> ### ส่วนที่ยังถูกต้องและ **M3 จะหยิบกลับมาใช้**
>
> `tool_reach` invariant (`down_reach >= clearance + soil_variation`) ·
> soil contact sensor เป็น required sensor · error code `tool_no_contact` /
> `tool_overreach` และนโยบายจัดการ · scenario `soil_high_spot` /
> `soil_low_spot` / `weed_inside_crop_row` · §6 การแยก crop/weed ด้วย row fitting
> (จำเป็นตอนต้องเห็นวัชพืชในแถว)
>
> ### ส่วนที่ยังใช้อยู่ใน design ปัจจุบัน
>
> D4 (soil heightfield rigid ±15 mm) · D9 (CAD = geometry, config = operational) ·
> D10 (URDF → base USD + override layer) · §8 Environment generator
> + domain randomization

# Design: Weeding Bed แทน Weeding Table

**Date:** 2026-09-12
**Status:** Approved
**Supersedes:** `mini-smart-weeding-table-repository-architecture.md` (sections 7, 8, 9, 25, 27, 33, 48)

---

## 1. Context

Design เดิมจำลองหุ่นยนต์กำจัดวัชพืชบน **โต๊ะ + ถาด 40×60 cm** — พื้นเรียบ, พืชปลูกในถาด,
background เป็นถาดสีเรียบ

Design นี้เปลี่ยนไปจำลองบน **แปลงปลูกขนาดเล็ก** — ดินจริง, ผิวไม่เรียบ, พืชปลูกเป็นแถว,
background เป็นดิน

เหตุผล: ถาดบนโต๊ะไม่ได้พิสูจน์ปัญหาที่ระบบจะเจอจริง — ดินไม่เรียบทำให้ระยะหัว tool
ไม่คงที่ และดินเป็น background ที่ทำให้ color threshold แบบง่ายใช้ไม่ได้
ทั้งสองอย่างเป็นความเสี่ยงหลักของ POC ควรเจอใน simulation ก่อนสร้างของจริง

---

## 2. Decisions

| # | Decision | Rationale |
|---|---|---|
| D1 | **Gantry คร่อมแปลง** (FarmBot style) — ไม่ใช่ rover | ใช้ `Machine` interface และ gantry kinematics เดิมได้ทั้งหมด ไม่ต้องเพิ่ม navigation / odometry / localization |
| D2 | **Servo ขึ้น/ลง — ไม่เพิ่มแกน Z** | คุม POC ให้เล็ก ไม่เพิ่ม stepper ตัวที่ 3 แลกกับข้อจำกัดด้านความลึกที่จัดการด้วย D5 |
| D3 | **Workspace คงเดิม 600 × 400 mm** | `config/machine.yaml`, axis limits, gantry model, homing ไม่ต้องแก้ — โฟกัสอยู่ที่ดินและ CV |
| D4 | **ดินเป็น heightfield rigid mesh ผิวไม่เรียบ ±10–15 mm — ไม่ยุบ** | ได้ collision และ CV background ที่สมจริง โดย physics ยัง stable และรัน headless CI ได้ deformable soil แพงเกินและ tune ยากสำหรับ POC |
| D5 | **`soil_contact` เป็น required sensor** | เมื่อไม่มีแกน Z มันเป็นตัวเดียวที่ยืนยันว่า tool แตะดินจริง ไม่ลอยและไม่จ้วง |
| D6 | **พืชผลปลูกเป็นแถว + วัชพืชสุ่ม** | สมจริงตามแปลงปลูก และได้ row structure เป็นตัวแยก crop/weed ที่ไม่ต้องพึ่ง AI ตั้งแต่ milestone 2 |
| D7 | **ชื่อโครงการ → "Mini Smart Weeding Robot"** | ตรงกับชื่อ repo `mini-weeding-robot` และไม่ผูกกับ form factor แบบโต๊ะ |
| D8 | **CAD/hardware ใช้ชื่อ `gantry` ไม่ใช่ `rover`** | ชื่อไฟล์ที่ขัดกับ D1 จะทำให้เข้าใจผิดว่าหุ่นยนต์เคลื่อนที่ได้ |
| D9 | **แยกความเป็นเจ้าของ: CAD = geometry, config = operational** | ไม่ต้องเขียน generator ที่ผูกกับ Fusion API ตั้งแต่ POC ใช้ตาราง derived + unit test จับ drift แทน |
| D10 | **URDF → base USD (generated) + override layer (commit)** | URDF เก็บ drive gain / damping / contact / sensor ของ Isaac ไม่ได้ การแยก layer ทำให้ regenerate จาก CAD ได้โดยไม่ทับ physics tuning |

---

## 3. What Does NOT Change

จุดสำคัญของ design นี้คือขอบเขตของการเปลี่ยน — architecture หลักไม่ถูกแตะ

```text
Machine interface        additive เท่านั้น (+ get_tool_state) ไม่มี breaking change
Camera interface         ไม่เปลี่ยน
Backend selection        ไม่เปลี่ยน (fake / isaac / esp32)
Gantry kinematics        ไม่เปลี่ยน (X/Y prismatic + tool revolute)
Workspace & axis limits  ไม่เปลี่ยน (600 × 400 mm)
State machine            ไม่เปลี่ยน
Layer boundary           ไม่เปลี่ยน
Transport / bridge       ไม่เปลี่ยน
Development phases       ไม่เปลี่ยน (V0 → V4)
```

### ข้อยกเว้นเดียว: `get_tool_state()`

`Machine` ได้ read-only method เพิ่ม 1 ตัว:

```python
def get_tool_state(self) -> dict:
    """{"state": "up" | "down", "soil_contact": bool}"""
```

จำเป็นเพราะเมื่อไม่มีแกน Z `soil_contact` เป็นตัวเดียวที่ยืนยันว่า tool แตะดิน
ถ้าไม่มี read path นี้ controller ไม่สามารถรู้ว่า `tool_down()` สำเร็จจริงหรือลอยอยู่

เป็นการเพิ่มแบบ additive — method เดิมทุกตัวไม่เปลี่ยน signature หรือ semantics
`FakeMachine` คืนค่า `soil_contact: true` เสมอ (ยกเว้นที่ scenario กำหนด)

**ไม่** ใส่ `soil_contact` ใน `get_endstops()` เพราะมันไม่ใช่ limit ของการเคลื่อนที่
การรวมกันจะทำให้ safety logic ของ endstop และ tool verification ปนกัน

### สรุป

การที่เปลี่ยน environment จากโต๊ะเป็นแปลงดินได้โดยแตะ interface แค่ระดับ additive
คือหลักฐานว่า machine abstraction ใน design เดิมทำงาน

---

## 4. Scene Model

```text
World
│
├── Bed                          แปลงยกขนาดเล็ก
│   ├── SoilSurface              heightfield mesh, rigid, variation ±10–15 mm
│   ├── BedFrame                 ขอบแปลง
│   ├── CropRow_1 .. CropRow_N   พืชผลเรียงแถว ระยะคงที่
│   │   └── Crop_001 ..
│   └── Weed_001 .. Weed_M       สุ่มระหว่างแถว / ในแถว
│
├── Gantry                       คร่อมแปลง
│   ├── X Prismatic Joint
│   ├── Y Prismatic Joint
│   └── Tool Revolute Joint      servo
│
└── Camera
```

### Asset mapping

| เดิม | ใหม่ |
|---|---|
| `scenes/weeding_table.usd` | `scenes/weeding_bed.usd` |
| `scenes/tray.usd` | `scenes/bed_frame.usd` + `scenes/soil_heightfield.usd` |
| `scenes/plants/` | `scenes/crops/` + `scenes/weeds/` |

---

## 5. Tool Reach Constraint

ผลพลอยจาก D2 + D4 ที่ต้องจัดการอย่างชัดเจน

บนถาด ระยะจากหัว tool ถึงพื้นคงที่ → `tool_down()` แตะทุกครั้ง
บนดินผิวไม่เรียบ servo มุมเดียวจะ **ลอยที่จุดต่ำ** และ **จ้วงลึกที่จุดสูง**

### Config contract

```yaml
bed:
  soil_reference_z_mm: 0        # ระดับดินอ้างอิงหลัง homing
  soil_variation_mm: 15         # ความไม่เรียบสูงสุดที่ยอมรับ (±)

tool:
  clearance_mm: 40              # ระยะหัว tool เหนือ soil_reference_z เมื่อ tool up
  down_reach_mm: 60             # ระยะที่ servo กดหัว tool ลงได้
```

**Invariant ที่ต้อง validate ตอน startup:**

```text
down_reach_mm >= clearance_mm + soil_variation_mm
```

ถ้าไม่ผ่าน → ไม่ต้อง start ให้ fail ทันทีพร้อมบอกค่าที่ขัดกัน

### Failure modes ใหม่

| Code | เงื่อนไข | การจัดการ |
|---|---|---|
| `tool_no_contact` | `tool_down()` เสร็จแล้ว `soil_contact == false` | retry ได้ 1 ครั้ง แล้วข้ามวัชพืชจุดนั้น บันทึก log |
| `tool_overreach` | `soil_contact` เป็น true เร็วกว่ามุม servo ที่คาด เกิน budget | ยก tool ทันที เข้า `ERROR` |

---

## 6. Perception

ส่วนที่เปลี่ยนมากที่สุด

| | เดิม (ถาด) | ใหม่ (แปลงดิน) |
|---|---|---|
| Background | ถาดสีเรียบ | ดิน texture สีน้ำตาล/แดง ไม่สม่ำเสมอ |
| Milestone 2 detector | color threshold ตรง ๆ | **ExG** (excess green index) `2G − R − B` แล้ว threshold |
| แยก crop / weed | ขนาด + สี | **row fitting** — fit เส้นแถวปลูกจาก crop blob → green blob ที่อยู่นอก corridor ของแถว = weed |
| Calibration plane | tray เรียบ → homography แม่น | soil ไม่เรียบ → homography บน plane เดียวมี error ต้องประกาศเป็น **error budget (mm)** |

### ไฟล์ใหม่

```text
perception/row_detection.py     fit crop row lines → corridor mask
```

### Error budget

ดินไม่เรียบ ±15 mm ทำให้จุดที่สูง/ต่ำกว่า reference plane ถูก project ผิดตำแหน่ง
ขนาด error ขึ้นกับมุมมองกล้อง — ต้องวัดจริงในขั้น calibration และบันทึกลง
`config/camera.yaml` error ที่ยอมรับได้ต้องเล็กกว่ารัศมีหัว tool

---

## 7. Protocol

เพิ่ม field เดียวใน `machine_state`:

```json
"tool": {
  "state": "down",
  "soil_contact": true
}
```

เพิ่ม error code: `tool_no_contact`, `tool_overreach`

ไม่มี message type ใหม่ ไม่มี breaking change กับ command ที่มีอยู่

---

## 8. Simulation Environment Generator

```python
generate_bed(
    crop_rows=4,
    crop_spacing_mm=80,
    row_spacing_mm=100,
    weed_count=10,
    soil_variation_mm=15,
)
```

Randomize:

- Soil heightfield seed, ความขรุขระ, texture/สีดิน, ความชื้น (สีเข้ม/อ่อน)
- ตำแหน่ง/ขนาด/rotation ของวัชพืช
- Jitter ตำแหน่งพืชผลในแถว (แปลงจริงปลูกไม่ตรงเป๊ะ)
- มุมเอียงของแถวทั้งชุด (row misalignment)
- Lighting, เงา, camera noise

Row jitter และ row misalignment สำคัญ — ถ้าแถวตรงเป๊ะทุก run
`row_detection.py` จะ overfit และใช้กับแปลงจริงไม่ได้

---

## 9. Scenario Tests

### เพิ่ม

| Scenario | ทดสอบ |
|---|---|
| `weed_inside_crop_row.yaml` | วัชพืชชิดพืชผล — ต้องไม่กำจัดพืชผลผิด |
| `soil_high_spot.yaml` | จุดดินสูง → ต้องตรวจจับ `tool_overreach` |
| `soil_low_spot.yaml` | จุดดินต่ำ → ต้องตรวจจับ `tool_no_contact` |
| `crop_row_misaligned.yaml` | แถวเอียง → `row_detection` ต้องยังแยก crop/weed ถูก |

### เปลี่ยนชื่อ

`normal_10_weeds.yaml` → `normal_bed_10_weeds.yaml`

### คงเดิม

`weed_near_x_min` · `weed_near_x_max` · `endstop_failure` · `camera_delay` ·
`motor_timeout` · `estop_during_motion` · `100_random_weeds`

---

## 10. Milestone Impact

| Milestone | เปลี่ยน |
|---|---|
| **1 — Mechanical** | Scene เป็นแปลงดินแทนโต๊ะ เพิ่ม acceptance: `soil_contact` ทำงาน และ tool reach invariant ผ่าน |
| **2 — Simple CV** | จาก color threshold → ExG + row fitting เพิ่ม acceptance: แยก crop/weed ถูกเมื่อแถวเอียง |
| **3 — AI** | ไม่เปลี่ยน — แต่ dataset จาก `tools/dataset/` ได้ background ดินจริงแล้ว มีค่ากับของจริงมากขึ้น |

V0–V4 phases ไม่เปลี่ยน

---

## 11. Files Touched

```text
README.md                    retitle + framing + layout table
docs/architecture.md         scene refs, tool constraint, failure modes
docs/simulation.md           มากสุด — scene, soil, generator, sensor mapping
docs/hardware.md             BOM + soil contact sensor + gantry clearance
docs/calibration.md          soil plane error budget, ExG, row
docs/protocol.md             soil_contact, error codes
protocol/messages.md         soil_contact, error codes
config/README.md             bed block, tool reach, environment keys
controller/README.md         get_tool_state, tool_reach.py, startup validation
perception/README.md         ExG, row_detection
tests/README.md              scenario list + soil_low_spot example
sim/isaac/README.md          layout, asset names, soil_contact sensor

sim/isaac/scenes/crops/      ใหม่ (แทน plants/)
sim/isaac/scenes/weeds/      ใหม่
```

Original design doc ได้รับหมายเหตุ superseded ที่หัวไฟล์ แต่เนื้อหาไม่ถูกแก้ —
เก็บไว้เป็นบันทึกการตัดสินใจเดิม

---

## 12. Known Limitations

1. **ไม่มีการคุมความลึกเป็น mm** — servo มุมคงที่ + soil contact บอกได้แค่ "แตะ/ไม่แตะ"
   ถ้าภายหลังต้องกำจัดวัชพืชที่ต้องขุดลึกแม่นยำ ต้องเพิ่มแกน Z (D2 จะถูก revisit)
2. **ดินไม่ยุบ** — tool ลงดินจริงจะมีแรงต้านและดินเปลี่ยนรูป simulation จะไม่เห็นผลนี้
   ต้องพิสูจน์ในขั้น V4 กับ hardware จริง
3. **Homography plane เดียว** บนผิวไม่เรียบมี error ตกค้างเสมอ ลดได้ด้วยกล้องมองตรงลง
   (top-down) แต่ไม่หมดไป

---

## 13. Addendum — CAD & Hardware Documentation

เพิ่มหลังจาก design หลักได้รับอนุมัติ ครอบคลุม subsystem `cad/` และ `hardware/`

### 13.1 Asset Pipeline

```text
Fusion 360  cad/fusion/gantry.f3d          ◄── source of truth: geometry
   │
   ├── parameters ──> cad/parameters/parameters.csv
   │                     ├─> config/machine.yaml                 (runtime)
   │                     ├─> hardware/mechanical/dimensions.md   (เอกสาร)
   │                     └─> cad/urdf/                           (simulation)
   │
   └── exports ──> step/ · stl/ · meshes/
                      │
                      ▼
              cad/urdf/weeding_gantry.urdf
                      │  scripts/import_urdf.sh
                      ▼
       sim/isaac/robots/gantry/gantry_base.usd   generated, gitignored
                      │  + USD layer
                      ▼
       sim/isaac/robots/gantry/gantry.usd        override, commit
```

### 13.2 Ownership Boundary (D9)

| | เจ้าของ | ตัวอย่าง |
|---|---|---|
| วัดด้วยเวอร์เนียร์ได้ | **CAD** | ความยาวราง, ความสูงโครง, ความยาวแขน servo, ตำแหน่ง mount กล้อง |
| เปลี่ยนได้โดยไม่ต้องถอดสกรู | **config** | soft limit, speed, accel, servo angle, timeout, `soil_variation_mm` |

Enforcement: ค่า derived ทุกตัวมี comment `# derived: cad <parameter>` ใน YAML
+ `tests/unit/test_cad_config_sync.py` อ่าน `parameters.csv` เทียบกับ `machine.yaml`

**ไม่มี generator อัตโนมัติโดยตั้งใจ** — tooling ที่ผูกกับ Fusion API
มีต้นทุนดูแลสูงเกินความจำเป็นของ POC

### 13.3 Documentation Boundary

| | ขอบเขต |
|---|---|
| `hardware/` | เครื่องจริง — BOM, ขนาด, ประกอบ, สายไฟ, coordinate frames |
| `docs/hardware.md` | firmware & integration — ESP32 scope, motion flow, build/flash, HIL |

เส้นแบ่ง: *ต้องใช้ไขควง = `hardware/` · ต้องใช้ compiler = `docs/hardware.md`*

BOM และ machine limits ที่เคยอยู่ใน `docs/hardware.md` ย้ายไป `hardware/` แล้ว

### 13.4 `coordinate-frames.md` — ช่องโหว่ที่ปิด

ก่อนหน้านี้นิยาม frame กระจายอยู่ใน `calibration.md` กับ code
`hardware/mechanical/coordinate-frames.md` รวมไว้ที่เดียว:

```text
world → bed → machine → tool
          └─► camera
```

พร้อมสิ่งที่ไม่เคยเขียนไว้ชัดมาก่อน:

- `machine ↔ bed` ต่างกันด้วย `home_offset_mm` ที่ต้องวัดตอนประกอบ
  ถ้าลืม ระบบจะ "ดูเหมือนทำงาน" แต่พลาดทุกจุดอย่างเป็นระบบ
- **soil reference plane ≠ ผิวดินจริง** — ผิวจริงแกว่งรอบ ๆ ±15 mm
- ตาราง unit convention: config/protocol = mm+deg, URDF/USD = m+rad
  มีจุดแปลงหน่วยแค่ 2 ที่ และทั้งคู่ต้องมี unit test

### 13.5 ไฟล์

```text
สร้าง  cad/README.md · cad/parameters/README.md
       cad/exports/README.md · cad/urdf/README.md
       hardware/README.md · hardware/bom/poc-v1.md
       hardware/mechanical/{dimensions,assembly,coordinate-frames}.md
       hardware/electrical/{wiring,power}.md

แก้    README.md (layout + asset pipeline)
       docs/hardware.md (แยกบทบาท เหลือ firmware/integration)
       config/README.md (derived comments + home_offset_mm)
       scripts/README.md (import_urdf.sh)
       .gitignore (gantry_base.usd, Fusion backup)
```

### 13.6 ค่าที่ตั้งไว้และควรตรวจสอบก่อนสร้างจริง

ตัวเลขใน `cad/parameters/README.md` เป็น **ค่าเริ่มต้นที่สอดคล้องกัน** ไม่ใช่ค่าที่วัดจากของจริง
ต้องยืนยัน 3 ข้อนี้ก่อนสั่งของ:

1. **margin ของ invariant เหลือ 5 mm** — `60 >= 40 + 15` ถ้า `soil_variation_mm`
   ที่วัดจริงเกิน 20 mm ต้องยืดแขน tool หรือเพิ่ม `down_angle`
2. **FOV กล้อง** — ที่ 520 mm ต้องการ ≥ 60° แนวนอน webcam ทั่วไปอาจไม่ถึง
3. **`tool_tip_radius_mm` = 8** เป็นตัวกำหนด error budget ของ perception —
   coordinate transform ต้องแม่นกว่านี้
