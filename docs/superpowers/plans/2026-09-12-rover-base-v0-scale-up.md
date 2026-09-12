# Rover Base V0 Scale-up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the rover's geometry, config, URDF and sim physics for a 650 × 520 mm, Ø250-wheel, ~35 kg machine, replacing the 200 × 145 mm one, with every derived number traceable back to `cad/parameters/parameters.csv`.

**Architecture:** `parameters.csv` is the single source of geometry. Changing it turns `test_cad_config_sync.py` and `test_urdf_matches_cad.py` red, and those failures are the worklist: config, the mixing table, the URDF and the sim physics each follow in turn. Sim meshes stop being hand-exported binaries and become generated from the CSV by `tools/generate_sim_meshes.py`, so Isaac work does not wait on a hand-built Fusion assembly. No file under `controller/`, `perception/`, `bridge/`, `firmware/` (except two constants in `mixing.h`) or `protocol/` changes.

**Tech Stack:** Python 3.12, pytest, ruff, PyYAML, NVIDIA Isaac Sim (USD), PlatformIO/Unity (ESP32 C++), Fusion 360 (manual, out of scope here).

**Spec:** [`docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md`](../specs/2026-09-12-rover-base-v0-scale-up-design.md)

## Global Constraints

- **Units.** `config/` and `protocol/` are **mm and degrees**. `cad/urdf/` and USD are **metres and radians**. These are the only two conversion points and both have unit tests. Never mix.
- **Never edit a value to make a test pass.** If `test_cad_config_sync.py` fails, the config is behind the CAD — fix the config. If `test_drive_mixing.py` fails, the implementation drifted — fix the implementation, not the golden table.
- **Frozen geometry** (spec §4): `track_width_mm 430` · `wheelbase_mm 400` · `wheel_diameter_mm 250` · `wheel_width_mm 90` · `chassis_clearance_mm 125` · `body_width_mm 520` · `chassis_plate_width_mm 340` · `overall_length_mm 650` · `body_length_mm 500` · `body_height_mm 280` · `mast_diameter_mm 35` · `mast_height_mm 500` · `motor_mount_bolt_circle_mm 60`.
- **Frozen camera geometry** (spec §4.3): `camera_front_height_mm 850` · `camera_front_tilt_deg 50` · `camera_front_offset_x_mm 0` · `camera_down_height_mm 850` · `camera_down_tilt_deg 0` · `camera_down_offset_x_mm 0`.
- **Frozen drive limits** (spec §6.1): `v_max_mm_s 160` · `omega_max_deg_s 25` · `wheel_v_max_mm_s 327` · `wheel_v_min_mm_s 51`.
- **Frozen bed geometry** (spec §5): `row_spacing_mm 750` · `crop_foliage_half_width_mm 30` · `soil_variation_mm 15`.
- **Unchanged on purpose:** `runaway_budget_mm 60` · `command_timeout_ms 300` · `row_loss_frames 3` · `loop_hz 10` · `max_lateral_error_mm 40`.
- **Motor reference speed: 25 rpm at the output shaft**, living in `hardware/bom/poc-v3.md`, not in CAD.
- **Repo is intentionally red during Tasks 2–6.** Each task names the exact scoped pytest command that is its own deliverable. Do not chase failures belonging to a later task.
- Run `ruff check .` and `ruff format .` before every commit that touches Python.

---

### Task 1: BOM v3 — drivetrain requirement envelope

The motor speed is not a CAD value. `test_cad_config_sync.py` parses it out of the BOM markdown with a regex, so the BOM has to exist and state it in the exact shape the regex expects before anything downstream can compute `wheel_v_max`.

**Files:**
- Create: `hardware/bom/poc-v3.md`
- Modify: `tests/unit/test_cad_config_sync.py:31` (`BOM_MD`), `:154-168` (`test_the_bom_still_states_the_motor_speed`, `_motor_rpm`)

**Interfaces:**
- Consumes: nothing
- Produces: `hardware/bom/poc-v3.md` containing a line matching `r"DC gear motor 24 V\s*\*\*~(\d+) RPM\*\*"` with the value `25`. Task 3 relies on this for `wheel_v_max_mm_s = 327`.

- [ ] **Step 1: Point the test at the new BOM and the new speed**

In `tests/unit/test_cad_config_sync.py`, change the constant:

```python
BOM_MD = REPO / "hardware" / "bom" / "poc-v3.md"
```

and replace the two motor-speed helpers:

```python
def test_the_bom_still_states_the_motor_speed():
    """wheel_v_max is derived from a number that lives in the BOM, not in CAD.
    If the BOM row is reworded, the derivation below is quietly testing nothing.
    """
    assert _motor_rpm() == 25.0


def _motor_rpm() -> float:
    found = re.search(
        r"DC gear motor 24 V\s*\*\*~(\d+) RPM\*\*", BOM_MD.read_text(encoding="utf-8")
    )
    assert found is not None, f"could not find the motor speed in {BOM_MD}"
    return float(found.group(1))
```

- [ ] **Step 2: Run it to verify it fails**

```bash
pytest tests/unit/test_cad_config_sync.py::test_the_bom_still_states_the_motor_speed -v
```

Expected: FAIL with `FileNotFoundError` — `hardware/bom/poc-v3.md` does not exist.

- [ ] **Step 3: Write the BOM**

Create `hardware/bom/poc-v3.md`:

````markdown
# BOM — POC v3 (Rover Base V0, 650 × 520 mm)

**Status:** Requirement envelope — **ยังไม่เลือก part number**
**Supersedes:** [`poc-v2.md`](poc-v2.md) ทั้งฉบับ (เครื่องคนละขนาด ไม่ใช่รุ่นปรับปรุง)
**Design:** [`docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md`](../../docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md) §8

---

## ทำไมเอกสารนี้ไม่มี part number

Design decision S10: รอบนี้ให้ความสำคัญกับ CAD ที่ปลดล็อกงาน development ก่อน
ตัวเลขทุกตัวด้านล่าง**คำนวณได้จากเรขาคณิตและ invariant** ไม่ได้มาจาก catalogue

ทุกแถวมี gate เดียวกัน: **ยืนยันกับ datasheet ก่อนสั่ง** และเมื่อเลือกของจริงแล้ว
ต้องกลับมาแทนที่ `mass` ใน `tools/generate_sim_meshes.py` ด้วยค่าที่ชั่งได้จริง

---

## Drivetrain

| # | Item | Qty | ข้อกำหนด | Gate |
|---|---|---:|---|---|
| D1 | DC gear motor 24 V **~25 RPM** — brushed planetary, encoder ในตัว | 4 | ดู §หน้าต่างความเร็ว | ⚠️ ยืนยัน datasheet |
| D2 | Motor driver ≥ 10 A/ช่อง ที่ 24 V | 2 (dual) หรือ 4 | ดู §กระแส | ⚠️ ยืนยัน datasheet |
| D3 | ล้อ Ø250 × 90 mm ดอกยางเกษตร + ดุม | 4 | bolt circle 60 mm ตรงกับ D1 | ⚠️ ยืนยัน datasheet |
| D4 | แบตเตอรี่ 24 V LiFePO4 20 Ah (480 Wh) + BMS | 1 | วางต่ำและกึ่งกลาง | ⚠️ ยืนยัน datasheet |
| D5 | E-stop latching relay ตัด rail 24 V ของมอเตอร์ | 1 | ทนกระแส peak 15 A | ⚠️ ยืนยัน datasheet |

## Compute — ไม่เปลี่ยนจาก v2

| # | Item | Qty | หมายเหตุ |
|---|---|---:|---|
| C1 | Raspberry Pi 5 | 1 | Design decision S9 — override source spec §10 ที่ระบุ Jetson |
| C2 | ESP32 DevKit | 1 | ไม่เปลี่ยน `firmware/` ทั้งชุดใช้ต่อได้ |
| C3 | กล้อง USB ×2 | 2 | front ≥ 60° HFOV · down ≥ 45° HFOV |

Jetson เป็นเส้นทางของ M3 ตอนมี CNN weed detector — perception ตอนนี้เป็น ExG + Otsu
ต่อเฟรม ซึ่ง Pi 5 รันสบาย และ `bridge/` + `firmware/` ต่อไว้แล้วทั้งชุด

---

## หน้าต่างความเร็วมอเตอร์ — ทำไมต้อง 25 RPM

Invariant สองข้อบีบจากคนละด้าน ที่ `track_width = 430 mm`, `v_max = 160 mm/s`,
`omega_max = 25 deg/s`:

```text
differential = radians(25) x 430/2 = 93.81 mm/s

ข้อ 6  ล้อนอก 160 + 93.81 = 253.81 <= wheel_v_max   ->  rpm >= 18.6
ข้อ 7  ล้อใน  160 - 93.81 =  66.19 >= deadband
       ที่ 15% duty                                  ->  rpm <= 28.6
```

```text
หน้าต่าง                 18.6 – 28.6 RPM
เลือก                    25 RPM
wheel_v_max = (25/60) x pi x 250 = 327 mm/s
cruise 160 / 327         = 49% duty     ✓ พ้น deadband สบาย
```

49% duty เป็นไปตามกฎเดียวกับ v2: **ความเร็วมอเตอร์ควรอยู่ที่ 2–3 เท่าของความเร็วใช้งาน
ไม่ใช่มากที่สุดที่หาได้** — ที่ v2 ข้อนี้ตัดเกียร์ 125:1 ทิ้งเพราะเร็วไป 4%

---

## แรงบิด

ที่มวลประมาณ **35 kg** และล้อรัศมี 0.125 m:

```text
rolling อย่างเดียว (Crr 0.15 ดินร่วน)     1.61 N·m/ล้อ
ทางชัน 15° + rolling                     4.33 N·m/ล้อ   <- continuous
skid-steer pivot (mu_lat 0.7)            5.5  N·m/ล้อ   <- peak
```

| | ข้อกำหนด |
|---|---|
| Continuous | **≥ 5 N·m ต่อล้อ** |
| Peak | **≥ 10 N·m ต่อล้อ** |

### เพดานมวลที่แรงบิด continuous รับได้

```text
แรงลากรวม = 4 x 5 N·m / 0.125 m = 160 N
ต้านที่ 15° + rolling = m x 9.81 x (sin15 + 0.15 x cos15) = m x 3.960

m_max = 160 / 3.960 = 40.4 kg   ->  เพดาน 40 kg
```

**นี่คือเพดานที่ `test_total_mass_is_under_the_gearbox_ceiling` บังคับ**
แทนเพดาน 2.3 kg ของ v2 ที่มาจากเกียร์ 250:1 ตัวเล็ก

⚠️ ประมาณการมวลคือ 35 kg **±20%** ขอบบนคือ 42 kg ซึ่ง**เกินเพดาน** — ถ้าของจริง
ชั่งได้เกิน 40 kg ต้องขึ้นมอเตอร์แรงบิดสูงขึ้น ไม่ใช่ผ่อนเพดาน

---

## กระแสและพลังงาน

```text
drive continuous  4 x 5 N·m x 2.618 rad/s / 0.6 eff = ~87 W   ->  ~3.6 A ที่ 24 V
drive peak        ~250 W                                      ->  ~10 A
Pi 5 + กล้อง + ESP32                                          ->  ~30 W

รวมเฉลี่ย ~110 W  ->  480 Wh / 110 W = ~4.4 ชม.
```

Driver ต้องรับ **≥ 10 A/ช่อง continuous** เพราะ stall ของมอเตอร์แต่ละตัวอยู่เหนือ
กระแสใช้งานมาก และ skid-steer เข้าใกล้ stall ทุกครั้งที่หมุนอยู่กับที่บนดิน

---

## มวลโดยประมาณ — 35 kg ±20%

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

การกระจายมวลนี้ถูก encode ไว้ใน `tools/generate_sim_meshes.py` (`MASS_ITEMS`)
ซึ่งเป็นที่ที่ URDF ได้ mass และ inertia มา — **แก้ที่นั่นที่เดียว**

---

## ข้อจำกัดเชิงกลที่มาจาก ground clearance

`chassis_clearance = 125 mm` เท่ากับรัศมีล้อพอดี ท้องรถจึงอยู่**ที่ระดับเพลา**

> ห้ามมี hub carrier, bearing block, ตัวมอเตอร์, หัวน็อต หรือสายไฟ
> อยู่ต่ำกว่าแนวศูนย์กลางเพลา

แปลว่า **ขับตรงที่เพลาเท่านั้น** ไม่มีเฟืองทด ไม่มีโซ่ ไม่มีสายพานที่ห้อยลงมา
และหน้าแปลนมอเตอร์ต้องยึดเข้าโครงที่ระดับเพลาพอดี (bolt circle 60 mm)
````

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/unit/test_cad_config_sync.py::test_the_bom_still_states_the_motor_speed -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add hardware/bom/poc-v3.md tests/unit/test_cad_config_sync.py
git commit -m "Let the invariants pick the motor again, at a size they have not seen"
```

---

### Task 2: parameters.csv — the switch

This is the change everything else follows. It also adds two structural tests that encode the two contradictions the design resolved (spec §3.1 and §3.2), so neither can be silently reintroduced.

**Files:**
- Modify: `cad/parameters/parameters.csv` (whole file)
- Modify: `tests/unit/test_cad_config_sync.py:34-52` (`EXPECTED_PARAMETERS`), plus two new tests

**Interfaces:**
- Consumes: nothing
- Produces: `cad/parameters/parameters.csv` with the 19 parameters listed below. Every later task reads it.

- [ ] **Step 1: Write the failing structural tests**

Add to `tests/unit/test_cad_config_sync.py`, directly after `test_body_width_is_the_widest_point_not_the_chassis_plate`:

```python
def test_overall_length_is_measured_across_the_wheels():
    """Design decision S2.  The source spec said 620 mm with a 400 mm wheelbase
    and a 250 mm wheel, which is 15 mm short per side of where the wheel
    actually ends.  Width already used the across-the-wheels convention and
    closed exactly (430 + 90 = 520); length now uses the same one.

    A future edit that "restores" 620 without moving the wheelbase puts the
    envelope inside the tyres, and nothing else in the repo would notice.
    """
    parameters = _cad_parameters()
    expected = parameters["wheelbase_mm"] + parameters["wheel_diameter_mm"]
    assert parameters["overall_length_mm"] == expected, (
        f"CAD overall_length_mm = {parameters['overall_length_mm']} but "
        f"wheelbase {parameters['wheelbase_mm']} + "
        f"wheel_diameter {parameters['wheel_diameter_mm']} = {expected} — "
        f"overall_length is measured across the wheels, like body_width"
    )


def test_the_chassis_plate_clears_the_inner_faces_of_the_wheels():
    """Design decision S3.  The frame lives between z = 125 (belly) and z = 250
    (wheel top), which is exactly where the wheels are.  At the source spec's
    380 mm it overlaps each wheel by 20 mm.

    The plate may be narrower than the inner faces, never wider.  The body
    above z = 250 is free to overhang, and does at 380 mm.
    """
    parameters = _cad_parameters()
    inner_faces = parameters["track_width_mm"] - parameters["wheel_width_mm"]
    assert parameters["chassis_plate_width_mm"] <= inner_faces, (
        f"CAD chassis_plate_width_mm = {parameters['chassis_plate_width_mm']} but the "
        f"wheel inner faces are {inner_faces} mm apart "
        f"(track {parameters['track_width_mm']} - wheel_width {parameters['wheel_width_mm']}) — "
        f"the frame would hit the wheels"
    )
```

Update `EXPECTED_PARAMETERS` in the same file:

```python
EXPECTED_PARAMETERS = (
    # Chassis
    "overall_length_mm",
    "body_length_mm",
    "chassis_plate_width_mm",
    "body_width_mm",
    "body_height_mm",
    "chassis_clearance_mm",
    # Drive
    "track_width_mm",
    "wheelbase_mm",
    "wheel_diameter_mm",
    "wheel_width_mm",
    "motor_mount_bolt_circle_mm",
    # Mast
    "mast_diameter_mm",
    "mast_height_mm",
    # Camera
    "camera_front_height_mm",
    "camera_front_tilt_deg",
    "camera_front_offset_x_mm",
    "camera_down_height_mm",
    "camera_down_tilt_deg",
    "camera_down_offset_x_mm",
)
```

- [ ] **Step 2: Run them to verify they fail**

```bash
pytest tests/unit/test_cad_config_sync.py -k "overall_length or chassis_plate or parameter_table" -v
```

Expected: 3 FAIL — `KeyError: 'overall_length_mm'`, `KeyError: 'overall_length_mm'`, and the parameter-table set mismatch.

- [ ] **Step 3: Write the new parameters.csv**

Replace `cad/parameters/parameters.csv` entirely:

```text
# Fusion 360 user parameters — ต้นทางของตัวเลขขนาดทุกตัว
#
# ถอดมาจาก cad/parameters/README.md#parameters ตรง ๆ — ไฟล์นี้คือสิ่งที่ export
# ออกจาก Fusion แล้ว commit ทุกครั้งที่แก้ขนาด ห้ามแก้เพื่อให้ test ผ่าน
#
# Rover Base V0 — 650 x 520 mm, ล้อ O250, 4WD skid steer, ~35 kg
# Design: docs/superpowers/specs/2026-09-12-rover-base-v0-scale-up-design.md
#
# ค่าที่ไหลจาก CAD ไป config อยู่ในตาราง Derived Values ของ README
# บังคับด้วย:
#
#   pytest tests/unit/test_cad_config_sync.py
#
# สามค่าต่อไปนี้เป็น "ผลลัพธ์" ไม่ใช่ตัวเลือก — test บังคับความสัมพันธ์ไว้:
#
#   overall_length     = wheelbase + wheel_diameter   = 400 + 250 = 650
#   body_width         = max(chassis_plate_width, track_width + wheel_width)
#                      = max(340, 430 + 90) = 520
#   chassis_plate_width <= track_width - wheel_width  = 430 - 90 = 340
#
# body_width ไม่ใช่ความกว้างแชสซี — เป็นความกว้างรวมล้อ ส่วนที่ชนใบพืชคือล้อ
# chassis_plate_width ถูกบังคับให้ <= 340 เพราะโครงอยู่ในช่วง z 125-250 ซึ่งเป็น
# ที่ที่ล้ออยู่ ตัวถังเหนือ z 250 ยื่นได้ถึง 380 mm
#
# wheel_v_max_mm_s ใน config ไม่ได้เท่ากับพารามิเตอร์ตัวใดตัวหนึ่งที่นี่ แต่เป็นสูตร:
#   (motor_rpm / 60) x pi x wheel_diameter_mm = (25/60) x pi x 250 = 327.2 -> 327
# motor_rpm มาจาก hardware/bom/poc-v3.md ไม่ใช่จาก CAD
#
# chassis_clearance_mm = 125 เท่ากับรัศมีล้อพอดี ท้องรถอยู่ที่ระดับเพลา —
# ห้ามมีอะไรต่ำกว่าแนวศูนย์กลางเพลา ดู hardware/bom/poc-v3.md
#
# ⚠️ camera_front_tilt_deg และ camera_front_height_mm กำหนด lookahead distance
#    ซึ่งคือสิ่งที่ gain ของ row follower ถูก tune กับมัน
#    **ไม่มีสูตรจากเรขาคณิตไป gain และไม่มี test จับได้** — lookahead ขยับจาก
#    180 mm เป็น 713 mm ในรอบนี้ ต้อง tune ใหม่ที่ V1 และ V4 ด้วยมือ
#    ดู config/control.yaml
#
# Reader ต้องข้ามบรรทัดที่เริ่มด้วย '#' และบรรทัดว่าง
#
name,value,unit,comment
overall_length_mm,650,mm,ความยาวรวมวัดคร่อมล้อ = wheelbase + wheel_diameter
body_length_mm,500,mm,ความยาวตัวถัง (ไม่รวมล้อ)
chassis_plate_width_mm,340,mm,ความกว้างโครงช่วงล้อ = ระยะหน้าในล้อ
body_width_mm,520,mm,ความกว้างรวมล้อ = จุดกว้างสุดของรถ
body_height_mm,280,mm,ความสูงตัวถัง (ไม่รวม mast)
chassis_clearance_mm,125,mm,จากพื้นถึงท้องรถ = รัศมีล้อพอดี
track_width_mm,430,mm,ระยะกึ่งกลางล้อซ้ายถึงกึ่งกลางล้อขวา
wheelbase_mm,400,mm,ระยะกึ่งกลางเพลาหน้าถึงกึ่งกลางเพลาหลัง
wheel_diameter_mm,250,mm,เส้นผ่านศูนย์กลางล้อรวมยาง
wheel_width_mm,90,mm,ความกว้างหน้ายาง
motor_mount_bolt_circle_mm,60,mm,bolt circle หน้าแปลนมอเตอร์ 4 x M5
mast_diameter_mm,35,mm,ท่อ mast
mast_height_mm,500,mm,ความสูง mast เหนือหลังคาตัวถัง
camera_front_height_mm,850,mm,ความสูงเหนือ soil reference (ใน mast head)
camera_front_tilt_deg,50,deg,มุมก้มจากแนวนอน
camera_front_offset_x_mm,0,mm,mast อยู่กึ่งกลางตัวรถ
camera_down_height_mm,850,mm,ความสูงเหนือ soil reference (ใน mast head)
camera_down_tilt_deg,0,deg,0 = มองตรงลง
camera_down_offset_x_mm,0,mm,อยู่กึ่งกลางตัวรถ
```

- [ ] **Step 4: Run the task's own tests to verify they pass**

```bash
pytest tests/unit/test_cad_config_sync.py -k "overall_length or chassis_plate or parameter_table or body_width or marker or bom" -v
```

Expected: all PASS.

Now confirm the failures this task deliberately leaves for Task 3:

```bash
pytest tests/unit/test_cad_config_sync.py -v
```

Expected: the five `test_config_value_equals_the_cad_parameter` cases and `test_wheel_v_max_follows_from_wheel_diameter_and_motor_rpm` FAIL. **This is correct.** Task 3 fixes them.

- [ ] **Step 5: Commit**

```bash
git add cad/parameters/parameters.csv tests/unit/test_cad_config_sync.py
git commit -m "Move the switch, and let the tests say what has not caught up"
```

---

### Task 3: config/rover.yaml + the nine invariants

**Files:**
- Modify: `config/rover.yaml` (whole file)
- Modify: `tests/unit/test_startup_invariants.py:94, 107, 125-128, 133-147, 158, 204-225`

**Interfaces:**
- Consumes: `parameters.csv` (Task 2), `poc-v3.md` motor RPM (Task 1)
- Produces: `rover.track_width_mm 430`, `rover.drive.wheel_v_max_mm_s 327`, `bed.row_spacing_mm 750`. Task 5 reads track and `wheel_v_max` to regenerate the mixing table.

- [ ] **Step 1: Write the new rover.yaml**

Replace `config/rover.yaml` entirely:

```yaml
# rover.yaml — เรขาคณิตของ rover, ขีดจำกัดการขับ, และเรขาคณิตของแปลง
#
# ค่าที่มี comment `# derived: cad ...` มีต้นทางอยู่ที่ cad/parameters/parameters.csv
# ห้ามแก้เพื่อให้ test ผ่าน — ถ้า test fail แปลว่า CAD เปลี่ยนแล้ว config ตามไม่ทัน
#
# ดู config/README.md#roveryaml สำหรับที่มาของทุกตัวเลขในไฟล์นี้

rover:
  track_width_mm: 430          # derived: cad track_width
  wheelbase_mm: 400            # derived: cad wheelbase
  body_width_mm: 520           # derived: cad body_width  (รวมล้อ = track + wheel_width)
  wheel_diameter_mm: 250       # derived: cad wheel_diameter
  chassis_clearance_mm: 125    # derived: cad chassis_clearance

  drive:
    v_max_mm_s: 160
    omega_max_deg_s: 25
    wheel_v_max_mm_s: 327      # derived: cad wheel_diameter + มอเตอร์ 25 RPM
    wheel_v_min_mm_s: 51       # deadband — ต้องวัดจริงที่ V3 ดูหมายเหตุท้ายไฟล์

# `omega_max_deg_s: 25` ไม่ใช่ค่าที่เลือกเพราะชอบ — differential ของล้อเป็นสัดส่วนกับ
# track ซึ่งโตจาก 120 เป็น 430 (3.6 เท่า) ที่ 40 deg/s เดิม ล้อข้างในจะได้
# 160 - 150 = 10 mm/s ซึ่งอยู่ใต้ deadband: ล้อหยุดนิ่งขณะที่ ESP32 ยังคิดว่ากำลังขับ
# และรถเลี้ยวแรงกว่าที่สั่งโดยไม่มีอะไรส่งสัญญาณ
#
# `v_max_mm_s: 160` เพดานจาก runaway budget คือ 200 (60 mm / 0.3 s) เลือก 160
# เพราะมันดัน margin ของ invariant ข้อ 7 จาก 5.2 เป็น 15.2 mm/s ซึ่งเป็นข้อเดียว
# ที่พึ่งค่าที่ยังไม่ได้วัด
#
# `wheel_v_min_mm_s: 51` ถูกเก็บไว้เท่าเดิมโดยเจตนาแม้เปลี่ยนมอเตอร์ทั้งชั้น
#
# 15% duty ของ 327 mm/s ให้ 49 — แต่ค่าที่เข้มกว่าคือค่าที่เก็บ เหมือนที่ poc-v2
# ทำไว้ เกียร์ planetary มีชั้นเฟืองมาก stiction สูง deadband จริงอาจสูงกว่า 15%
# ไม่ใช่ต่ำกว่า เก็บ 51 ไว้จนกว่าจะวัดจริงที่ V3
#
# invariant ตัวล่างผ่านที่ 51: ล้อข้างในที่ omega_max = 66.2 >= 51  margin 15.2 mm/s

bed:
  soil_variation_mm: 15
  row_spacing_mm: 750
  crop_foliage_half_width_mm: 30    # ประมาณ — ต้องวัดจาก asset พืชที่ V1

# `row_spacing_mm: 750` ไม่ใช่ค่าที่เลือก แต่เป็นผลของ invariant ข้อ 5:
#
#   clear_furrow > body_width + 2 x runaway_budget = 520 + 120 = 640
#   clear_furrow = row_spacing - 2 x crop_foliage_half_width
#   -> row_spacing > 700
#
# 750 คือแถวมาตรฐาน 30 นิ้ว ให้ clear_furrow = 690 margin 50 mm
#
# ⚠️ margin นั้นถูกใช้จ่ายไปกับ `crop_foliage_half_width_mm` ที่ยังไม่ได้วัด:
#
#   row_spacing 750 ผ่านตราบใดที่ crop_foliage_half_width < 55 mm
#   ถ้า V1 วัดได้ 55+ -> row_spacing ต้องเป็น 800+
#
# นี่คือข้อจำกัดต่อ**การปลูก** ไม่ใช่แค่ค่าใน config — รถกว้าง 520 mm บังคับว่า
# แถวต้องห่างเท่าไร แถว 750 mm ที่ใบพืชกว้างข้างละ 30 mm แปลว่าพืชระยะต้นอ่อน
# ซึ่งเป็นช่วงที่กำจัดวัชพืชอยู่แล้ว
```

- [ ] **Step 2: Run the sync test to verify it now passes**

```bash
pytest tests/unit/test_cad_config_sync.py -v
```

Expected: all PASS.

- [ ] **Step 3: Run the invariants and read the new margins**

```bash
pytest tests/unit/test_startup_invariants.py -v
```

Expected: FAIL. The tests assert the old numbers in their messages and margins. Print the real margins to confirm they match the design before editing anything:

```bash
python -c "
from controller.config import load_config
from controller import startup_checks
startup_checks.run(load_config())
print(startup_checks.margins(load_config()))
"
```

Expected: no exception (all nine pass), and `clear_furrow` ≈ 50.0, `wheel_v_min` ≈ 15.19.

- [ ] **Step 4: Update the invariant tests to the new numbers**

In `tests/unit/test_startup_invariants.py`, make these replacements:

| Line | Old | New |
|---|---|---|
| docstring at `:94` | `35 > 15` | `125 > 15` |
| docstring at `:107` | `290 > 265` | `690 > 640` |
| docstring/assert at `:125-128` | `145 mm, not the 140 mm`, `assert "145" in message` | `520 mm, not the 340 mm`, `assert "520" in message` |
| docstring at `:133` | `141.9 <= 202` | `253.8 <= 327` |
| assert at `:136` | `assert "141.9" in message and "100" in message` | `assert "253.8" in message and "160" in message` |
| docstring at `:139` | `58.1 >= 51` | `66.2 >= 51` |
| assert at `:147` | `assert "37.2" in message and "51" in message` | recompute — see step below |
| assert at `:158` | `assert "120" in message` | `assert "430" in message` |

For `:147`, the test builds a deliberately-violating config. Run the single test with `-v` to read the actual message the new geometry produces, and assert on the number that appears:

```bash
pytest tests/unit/test_startup_invariants.py -k "deadband" -v
```

Replace `test_track_width_140_passes_but_with_almost_no_margin` and `test_the_shipped_track_width_keeps_a_real_margin` (`:204-225`) with:

```python
    def test_the_shipped_geometry_keeps_a_real_margin(self):
        """The two margins that matter, at the shipped numbers.

        clear_furrow is now the binding constraint of the whole machine — the
        520 mm body spends 640 of the 690 mm the 750 mm row leaves.  Ø250
        wheels retired wheel_clears_soil, which used to be the tight one.

        wheel_v_min's 15.19 mm/s is better than the 7.11 the small rover had,
        and it is better on purpose: v_max was set to 160 rather than 150 to
        buy it, because wheel_v_min is still an unmeasured 15%-duty estimate.
        """
        margins = startup_checks.margins(load_config())
        assert margins["clear_furrow"] == pytest.approx(50.0)
        assert margins["wheel_v_min"] == pytest.approx(15.19, abs=0.01)

    def test_raising_foliage_width_past_55_breaks_the_furrow(self):
        """The margin above is spent on a number nobody has measured.

        crop_foliage_half_width_mm is 30 by assumption, to be measured against
        the plant asset at V1.  At 55 the furrow no longer fits the rover, and
        row_spacing has to go to 800+ — which is a decision about what the
        customer plants, not a config edit.
        """
        config = load_config()
        config["bed"]["crop_foliage_half_width_mm"] = 55
        with pytest.raises(startup_checks.ConfigInvalid) as failure:
            startup_checks.run(config)
        assert "clear_furrow" in str(failure.value) or "640" in str(failure.value)
```

- [ ] **Step 5: Run the invariant tests to verify they pass**

```bash
pytest tests/unit/test_startup_invariants.py -v
```

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add config/rover.yaml tests/unit/test_startup_invariants.py
git commit -m "Let the furrow become the constraint the wheels used to be"
```

---

### Task 4: the remaining config files

**Files:**
- Modify: `config/control.yaml`, `config/perception.yaml`, `config/simulation.yaml`

**Interfaces:**
- Consumes: `rover.yaml` drive limits (Task 3)
- Produces: `row_follower.v_mm_s 160`, `row_follower.omega_max_deg_s 25`, `simulation.environment.row_spacing_mm 750`

`config/camera.yaml` needs **no change** — it holds calibration state (`intrinsic: null`, `homography: null`), not geometry. Extrinsics live in the URDF.

- [ ] **Step 1: Update control.yaml**

Change the two numeric values and add the lookahead note. In `config/control.yaml`:

```yaml
row_follower:
  v_mm_s: 160
  omega_max_deg_s: 25

  gains:
    fake:  {k_lat: 45.0, k_head: 25.0}
    isaac: {k_lat: 45.0, k_head: 25.0}
    esp32: {k_lat: null, k_head: null}    # ยังไม่ tune บนของจริง
```

Append to the header comment block, after the existing `⚠️` paragraph:

```yaml
# ⚠️ 2026-09-12 — lookahead ขยับจาก 180 mm เป็น 713 mm
#
#   เดิม  camera_front 180 mm ก้ม 45°  ->  lookahead 180 mm  = 0.9 เท่าของความยาวรถ
#   ใหม่  camera_front 850 mm ก้ม 50°  ->  lookahead 713 mm  = 1.1 เท่าของความยาวรถ
#
# gain สองคู่ด้านล่างเป็น**ค่าเดิมที่ยังไม่ได้ tune กับเรขาคณิตใหม่** — เก็บไว้เป็น
# จุดตั้งต้นให้ sim สตาร์ตได้เท่านั้น ไม่ใช่คำตอบ ต้อง tune ใหม่ที่ V1 (isaac)
# และ V4 (esp32) ไม่มี test จับข้อนี้ได้
```

- [ ] **Step 2: Update perception.yaml**

The only change is the corridor derivation in the header comment — `max_lateral_error_mm` itself stays at 40. Replace the corridor comment block:

```yaml
# `max_lateral_error_mm` กำหนดความกว้าง corridor:
#   corridor = clear_furrow − 2 × max_lateral_error_mm = 690 − 80 = 610 mm
#
# กล้องล่างต้องครอบ corridor + ขอบข้างละ 20 = 650 mm ที่ความสูง 850 mm
#   HFOV ที่ต้องการ = 2 × atan(325 / 850) = 41.9°  ->  เลนส์ >= 45°
```

- [ ] **Step 3: Update simulation.yaml**

Change one value — the bed generator's row spacing must match `bed.row_spacing_mm`:

```yaml
    row_spacing_mm: 750
```

Leave `soil.variation_mm: 15` alone (invariant 9 requires it to equal `bed.soil_variation_mm`, which did not change).

Add below the `environment` block:

```yaml
  # row_spacing_mm ต้องตรงกับ bed.row_spacing_mm ใน rover.yaml — ไม่มี invariant
  # บังคับข้อนี้ (ข้อ 9 บังคับแค่ soil) แต่ถ้าไม่ตรง sim จะสร้างแปลงที่รถลงไม่ได้
  # แล้ว row follower จะดูเหมือนพัง ทั้งที่เป็นปัญหาของแปลง
```

- [ ] **Step 4: Run config loading and invariants to verify nothing regressed**

```bash
pytest tests/unit/test_config_loading.py tests/unit/test_startup_invariants.py tests/unit/test_cad_config_sync.py -v
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add config/control.yaml config/perception.yaml config/simulation.yaml
git commit -m "Widen the simulated bed to the one the rover can actually enter"
```

---

### Task 5: regenerate the drive mixing table

`track_width` is in the mixing formula, so every row of the golden table is wrong. `cad/parameters/README.md` calls this out as a change no test catches — which is only true until the constants at the top of `test_drive_mixing.py` are updated, at which point it fails loudly.

**Files:**
- Modify: `config/drive_mixing_vectors.csv` (whole file)
- Modify: `tests/unit/test_drive_mixing.py:23-24`
- Modify: `firmware/esp32/include/mixing.h:22-23`

**Interfaces:**
- Consumes: `rover.track_width_mm 430`, `rover.drive.wheel_v_max_mm_s 327` (Task 3)
- Produces: a 12-row golden table. Row count stays 12, so `EXPECTED_VECTORS` in `firmware/esp32/test/test_mixing.cpp` needs no change.

- [ ] **Step 1: Update the test constants so the table fails loudly**

In `tests/unit/test_drive_mixing.py`:

```python
TRACK_WIDTH_MM = 430.0
WHEEL_V_MAX_MM_S = 327.0
```

- [ ] **Step 2: Run to verify the golden rows now fail**

```bash
pytest tests/unit/test_drive_mixing.py -v
```

Expected: the 12 `test_golden_vector` cases FAIL (the table still holds track-120 numbers), `test_the_table_still_matches_the_config_it_was_generated_against` PASSes.

- [ ] **Step 3: Regenerate the table**

Verify the arithmetic first:

```bash
python -c "
import math
T=430.0; VMAX=327.0
rows=[(0,0),(160,0),(160,10),(160,-10),(160,15),(160,-15),(160,25),(160,-25),(80,25),(-160,0),(-160,15),(500,25)]
for v,w in rows:
    d=math.radians(w)*T/2
    l,r=v-d,v+d
    m=max(abs(l),abs(r))
    if m>VMAX:
        s=VMAX/m; l*=s; r*=s
    print(f'{v:g},{w:g},{l:.3f},{r:.3f}')
"
```

Replace `config/drive_mixing_vectors.csv` entirely:

```text
# Golden test vectors — skid-steer mixing
#
# SOURCE OF TRUTH สำหรับสูตร mixing ที่ถูก implement 2 ที่ที่แชร์โค้ดกันไม่ได้:
#   tests/unit/test_drive_mixing.py         Python / IsaacRover
#   firmware/esp32/test/test_mixing.cpp     C++ / PlatformIO native env
#
# ทั้งสองต้องผ่านตารางนี้ ถ้าตัวใดตัวหนึ่ง fail แปลว่า implementation เพี้ยนจากกัน
# ห้ามแก้ค่าในตารางเพื่อให้ test ผ่าน — แก้ implementation
#
# สูตร:
#   omega_rad = omega_deg_s * pi / 180
#   v_left    = v_mm_s - omega_rad * track_width_mm / 2
#   v_right   = v_mm_s + omega_rad * track_width_mm / 2
#
# Saturation: ถ้าล้อข้างใดเกิน wheel_v_max_mm_s ให้ scale ทั้งสองข้างตามอัตราส่วนเดิม
#             ห้าม clip ข้างเดียว เพราะจะเปลี่ยน omega ที่ได้จริงโดยเงียบ
#             แถวสุดท้าย (500, 25) เป็นเคสที่ทดสอบข้อนี้
#
# พารามิเตอร์ที่ตารางนี้อิงอยู่ (จาก config/rover.yaml):
#   track_width_mm    = 430
#   wheel_v_max_mm_s  = 327
# ถ้า track_width หรือ wheel_v_max เปลี่ยน ต้อง regenerate ตารางนี้ทั้งชุด
#
# ตารางนี้ถูก regenerate ทั้งชุดเมื่อ 2026-09-12 ตอนที่ track ขยับ 120 -> 430
# omega ที่ใช้ทดสอบเปลี่ยนจาก 10/20/40 เป็น 10/15/25 ตาม omega_max ใหม่
#
# Reader ต้องข้ามบรรทัดที่เริ่มด้วย '#' และบรรทัดว่าง
# Tolerance ในการเทียบ: +/- 0.01 mm/s
#
v_mm_s,omega_deg_s,v_left_mm_s,v_right_mm_s
0,0,0.000,0.000
160,0,160.000,160.000
160,10,122.475,197.525
160,-10,197.525,122.475
160,15,103.713,216.287
160,-15,216.287,103.713
160,25,66.189,253.811
160,-25,253.811,66.189
80,25,-13.811,173.811
-160,0,-160.000,-160.000
-160,15,-216.287,-103.713
500,25,223.680,327.000
```

- [ ] **Step 4: Run the Python side to verify it passes**

```bash
pytest tests/unit/test_drive_mixing.py -v
```

Expected: all PASS, including `test_the_table_is_actually_read` (still 12 rows).

- [ ] **Step 5: Update the firmware constants**

In `firmware/esp32/include/mixing.h`:

```c
#define TRACK_WIDTH_MM 430.0f   /* config/rover.yaml rover.track_width_mm */
#define WHEEL_V_MAX_MM_S 327.0f /* config/rover.yaml rover.drive.wheel_v_max_mm_s */
```

`firmware/esp32/test/test_mixing.cpp` reads the CSV at runtime and needs no edit — confirm `EXPECTED_VECTORS` is still 12:

```bash
grep -n "EXPECTED_VECTORS" firmware/esp32/test/test_mixing.cpp
```

If PlatformIO is available, run the native suite; if not, note it in the commit body as unverified:

```bash
pio test -e native -f test_mixing
```

- [ ] **Step 6: Commit**

```bash
git add config/drive_mixing_vectors.csv tests/unit/test_drive_mixing.py firmware/esp32/include/mixing.h
git commit -m "Regenerate every mixing row, because track width is in the formula"
```

---

### Task 6: generate the sim meshes from parameters.csv

The existing `.obj` files were exported by hand from Fusion. Fusion is a manual tool and the assembly does not exist at the new size, so the sim would wait on it. This task makes the simplified sim geometry *derived* instead — `exports/step|stl` for manufacturing stays Fusion's job and follows later without blocking anything.

**Files:**
- Create: `tools/generate_sim_meshes.py`
- Create: `tests/unit/test_generate_sim_meshes.py`
- Overwrite (generated): `cad/urdf/meshes/base_link.obj`, `cad/urdf/meshes/wheel.obj`, `cad/exports/meshes/base_link.obj`, `cad/exports/meshes/wheel.obj`

**Interfaces:**
- Consumes: `cad/parameters/parameters.csv` (Task 2), mass figures from `hardware/bom/poc-v3.md` §มวล (Task 1)
- Produces:
  - `generate_sim_meshes.main(repo_root: Path) -> None` — writes the four `.obj` files
  - `generate_sim_meshes.mass_properties() -> dict[str, LinkInertia]` keyed `"base_link"`, `"wheel"`, where `LinkInertia` is a frozen dataclass with fields `mass: float`, `com: tuple[float, float, float]`, `ixx iyy izz ixy ixz iyz: float`, all in kg and metres
  - CLI: `python tools/generate_sim_meshes.py --print-inertia` prints a URDF-ready `<inertial>` block per link

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_generate_sim_meshes.py`:

```python
"""The sim meshes are generated, not exported by hand.

cad/README.md separates exports/meshes/ (simplified visual + collision, sim
only) from exports/step|stl (manufacturing).  Only the first is generated here.
Manufacturing geometry stays Fusion's, and this script never touches it.

The point is that a dimension change cannot leave a stale mesh behind: run the
script, and the mesh is the CSV.
"""

import math
from pathlib import Path

import pytest

from tools import generate_sim_meshes as gen

REPO = Path(__file__).resolve().parents[2]
MM_PER_M = 1000.0


@pytest.fixture(scope="module")
def cad() -> dict[str, float]:
    return gen.read_parameters(REPO / "cad" / "parameters" / "parameters.csv")


def test_the_wheel_mesh_is_the_cad_wheel(tmp_path, cad):
    gen.main(REPO, out_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "wheel.obj")

    radius = max(math.hypot(x, z) for x, _, z in vertices)
    half_width = max(abs(y) for _, y, _ in vertices)

    assert radius == pytest.approx(cad["wheel_diameter_mm"] / 2 / MM_PER_M, abs=1e-6)
    assert half_width == pytest.approx(cad["wheel_width_mm"] / 2 / MM_PER_M, abs=1e-9)


def test_the_body_mesh_spans_belly_to_mast_head(tmp_path, cad):
    gen.main(REPO, out_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")

    lowest = min(z for _, _, z in vertices)
    highest = max(z for _, _, z in vertices)
    expected_top = (
        cad["chassis_clearance_mm"] + cad["body_height_mm"] + cad["mast_height_mm"]
    ) / MM_PER_M

    assert lowest == pytest.approx(cad["chassis_clearance_mm"] / MM_PER_M, abs=1e-9)
    assert highest == pytest.approx(expected_top, abs=1e-9)


def test_nothing_in_the_body_mesh_hangs_below_the_axle(tmp_path, cad):
    """chassis_clearance equals the wheel radius, so the belly is the axle
    plane.  Anything below it is a part that would drag — this is the layout
    rule hardware/bom/poc-v3.md states, made checkable."""
    gen.main(REPO, out_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")
    axle_z = cad["wheel_diameter_mm"] / 2 / MM_PER_M
    assert min(z for _, _, z in vertices) >= axle_z - 1e-9


def test_the_frame_never_reaches_the_wheels(tmp_path, cad):
    """Below the wheel tops the body must stay inside the wheel inner faces."""
    gen.main(REPO, out_dirs=[tmp_path])
    vertices = gen.read_obj_vertices(tmp_path / "base_link.obj")
    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    inner_face = (cad["track_width_mm"] - cad["wheel_width_mm"]) / 2 / MM_PER_M

    widest_in_wheel_zone = max(
        (abs(y) for _, y, z in vertices if z < wheel_top - 1e-9), default=0.0
    )
    assert widest_in_wheel_zone <= inner_face + 1e-9


def test_total_mass_matches_the_bom():
    properties = gen.mass_properties()
    total = properties["base_link"].mass + 4 * properties["wheel"].mass
    assert total == pytest.approx(35.0, abs=0.01)


def test_the_mass_is_under_the_continuous_torque_ceiling():
    """hardware/bom/poc-v3.md: 4 x 5 N.m at r = 0.125 m gives 160 N, and a 15
    degree grade plus rolling costs m x 3.960 N/kg.  40 kg is the ceiling."""
    properties = gen.mass_properties()
    total = properties["base_link"].mass + 4 * properties["wheel"].mass
    assert total <= 40.0


def test_the_wheel_inertia_is_a_cylinder_about_y():
    """The wheel joint axis is +Y, so Iyy is the spin axis: 0.5 m r^2."""
    wheel = gen.mass_properties()["wheel"]
    radius = 0.125
    assert wheel.iyy == pytest.approx(0.5 * wheel.mass * radius**2, rel=1e-9)
    assert wheel.ixx == pytest.approx(wheel.izz, rel=1e-9)


def test_the_centre_of_mass_is_low_enough_to_be_credible(tmp_path):
    """Battery low and central is the whole point of source spec section 10.
    If the CoM lands above half the body height, the mass model has the battery
    in the wrong place and Isaac will roll the rover on the first slope."""
    base = gen.mass_properties()["base_link"]
    assert base.com[2] < 0.265
    assert base.com[0] == pytest.approx(0.0, abs=1e-9)
    assert base.com[1] == pytest.approx(0.0, abs=1e-9)
```

- [ ] **Step 2: Run to verify it fails**

```bash
pytest tests/unit/test_generate_sim_meshes.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tools.generate_sim_meshes'`.

- [ ] **Step 3: Write the generator**

Create an empty `tools/__init__.py` — `tools/` is currently a plain directory whose three subdirectories hold only `.gitkeep`, so nothing breaks. The test imports it as `from tools import generate_sim_meshes`, which works because `pyproject.toml` sets `pythonpath = ["."]`. Do **not** add `tools` to `[tool.setuptools] packages`: that list is the installable runtime package set, and this is build-time tooling.

Then create `tools/generate_sim_meshes.py`:

```python
"""Generate the simplified sim meshes and mass properties from parameters.csv.

cad/README.md splits the exports: `exports/meshes/` is simplified visual and
collision geometry for the simulator, `exports/step|stl` is for manufacturing.
Only the first is generated here.  Fusion still owns everything a shop would
cut, and that work can follow later without blocking Isaac.

This reads a CSV.  It is NOT Fusion-API tooling, which cad/parameters/README.md
warns against on maintenance-cost grounds -- there is nothing here that breaks
when Autodesk ships a new version.

The mass model is an ESTIMATE (hardware/bom/poc-v3.md, +/-20%).  When real
parts are weighed, MASS_ITEMS below is the single place to change.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

MM_PER_M = 1000.0

#: How finely the wheel cylinder is tessellated.  The wheel's COLLISION is a
#: primitive cylinder in the URDF -- this mesh is visual only, so the segment
#: count buys appearance, not physics.  Isaac pays for every triangle.
WHEEL_SEGMENTS = 32


@dataclass(frozen=True)
class LinkInertia:
    """Mass properties in kg and metres, about the link's own centre of mass."""

    mass: float
    com: tuple[float, float, float]
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


@dataclass(frozen=True)
class Box:
    """An axis-aligned box, in metres, positioned in the base_link frame."""

    size: tuple[float, float, float]
    centre: tuple[float, float, float]


def read_parameters(path: Path) -> dict[str, float]:
    lines = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    return {row["name"]: float(row["value"]) for row in csv.DictReader(lines)}


def read_obj_vertices(path: Path) -> list[tuple[float, float, float]]:
    """Every `v` line of an OBJ, as (x, y, z) in metres."""
    vertices = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            x, y, z = (float(v) for v in line.split()[1:4])
            vertices.append((x, y, z))
    return vertices


# -- geometry ---------------------------------------------------------------


def body_boxes(cad: dict[str, float]) -> list[Box]:
    """The three boxes base_link is built from, in metres.

    The frame necks in below the wheel tops: chassis_plate_width is the inner
    face spacing, and the body above the wheels is free to overhang at
    body_width minus the wheels.  See the design doc section 3.2.
    """
    belly = cad["chassis_clearance_mm"] / MM_PER_M
    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    body_top = (cad["chassis_clearance_mm"] + cad["body_height_mm"]) / MM_PER_M
    length = cad["body_length_mm"] / MM_PER_M
    frame_width = cad["chassis_plate_width_mm"] / MM_PER_M
    shell_width = (cad["body_width_mm"] - cad["wheel_width_mm"]) / MM_PER_M

    frame_height = wheel_top - belly
    shell_height = body_top - wheel_top

    return [
        Box((length, frame_width, frame_height), (0.0, 0.0, belly + frame_height / 2)),
        Box((length, shell_width, shell_height), (0.0, 0.0, wheel_top + shell_height / 2)),
    ]


def mast_dimensions(cad: dict[str, float]) -> tuple[float, float, float]:
    """(radius, height, centre_z) of the mast tube, in metres."""
    radius = cad["mast_diameter_mm"] / 2 / MM_PER_M
    height = cad["mast_height_mm"] / MM_PER_M
    body_top = (cad["chassis_clearance_mm"] + cad["body_height_mm"]) / MM_PER_M
    return radius, height, body_top + height / 2


# -- mass model -------------------------------------------------------------

#: (name, mass_kg, Box in millimetres) -- hardware/bom/poc-v3.md section มวล.
#: Millimetres here so the table reads like the BOM; converted below.
MASS_ITEMS: tuple[tuple[str, float, tuple[float, float, float], tuple[float, float, float]], ...] = (
    ("frame", 8.5, (500, 340, 125), (0, 0, 187.5)),
    ("battery", 5.0, (300, 200, 120), (0, 0, 190)),
    ("power_box", 1.5, (200, 150, 80), (-150, 0, 300)),
    ("compute", 1.5, (200, 150, 80), (150, 0, 300)),
    ("shell", 3.0, (500, 380, 155), (0, 0, 327.5)),
    ("motors", 6.0, (400, 300, 100), (0, 0, 125)),
    ("fasteners", 1.1, (500, 340, 125), (0, 0, 187.5)),
)

#: One wheel: tyre, rim and hub together.  Modelled as a solid cylinder, which
#: overstates the inertia of a tyre (mass sits at the rim), and is the right
#: direction to be wrong in for a stability estimate.
WHEEL_MASS_KG = 1.8

#: The mast tube plus the sensor head it carries.
MAST_MASS_KG = 1.2


def _box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    a, b, c = size
    return (
        mass * (b * b + c * c) / 12.0,
        mass * (a * a + c * c) / 12.0,
        mass * (a * a + b * b) / 12.0,
    )


def mass_properties(cad: dict[str, float] | None = None) -> dict[str, LinkInertia]:
    """base_link and one wheel, about each link's own centre of mass."""
    if cad is None:
        cad = read_parameters(
            Path(__file__).resolve().parents[1] / "cad" / "parameters" / "parameters.csv"
        )

    radius, height, mast_centre_z = mast_dimensions(cad)

    parts: list[tuple[float, tuple[float, float, float], tuple[float, float, float]]] = []
    for _name, mass, size_mm, centre_mm in MASS_ITEMS:
        size = tuple(v / MM_PER_M for v in size_mm)
        centre = tuple(v / MM_PER_M for v in centre_mm)
        parts.append((mass, centre, _box_inertia(mass, size)))

    mast_own = (
        MAST_MASS_KG * (3 * radius * radius + height * height) / 12.0,
        MAST_MASS_KG * (3 * radius * radius + height * height) / 12.0,
        MAST_MASS_KG * radius * radius / 2.0,
    )
    parts.append((MAST_MASS_KG, (0.0, 0.0, mast_centre_z), mast_own))

    total = sum(mass for mass, _, _ in parts)
    com = tuple(
        sum(mass * centre[axis] for mass, centre, _ in parts) / total for axis in range(3)
    )

    ixx = iyy = izz = 0.0
    for mass, centre, own in parts:
        dx, dy, dz = (centre[axis] - com[axis] for axis in range(3))
        ixx += own[0] + mass * (dy * dy + dz * dz)
        iyy += own[1] + mass * (dx * dx + dz * dz)
        izz += own[2] + mass * (dx * dx + dy * dy)

    wheel_radius = cad["wheel_diameter_mm"] / 2 / MM_PER_M
    wheel_width = cad["wheel_width_mm"] / MM_PER_M
    wheel_transverse = (
        WHEEL_MASS_KG * (3 * wheel_radius**2 + wheel_width**2) / 12.0
    )

    return {
        "base_link": LinkInertia(total, com, ixx, iyy, izz),
        "wheel": LinkInertia(
            WHEEL_MASS_KG,
            (0.0, 0.0, 0.0),
            wheel_transverse,
            0.5 * WHEEL_MASS_KG * wheel_radius**2,
            wheel_transverse,
        ),
    }


# -- OBJ writing ------------------------------------------------------------


def _box_obj(box: Box, offset: int) -> tuple[list[str], list[str]]:
    (a, b, c), (cx, cy, cz) = box.size, box.centre
    corners = [
        (cx + sx * a / 2, cy + sy * b / 2, cz + sz * c / 2)
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    vertices = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in corners]
    quads = [
        (0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
        (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3),
    ]
    faces = [
        "f " + " ".join(str(offset + index + 1) for index in quad) for quad in quads
    ]
    return vertices, faces


def _cylinder_obj(
    radius: float, length: float, axis: str, centre: tuple[float, float, float], offset: int
) -> tuple[list[str], list[str]]:
    """A closed cylinder tessellated into WHEEL_SEGMENTS, about `axis`."""
    half = length / 2
    rings: list[tuple[float, float, float]] = []
    for end in (-half, half):
        for segment in range(WHEEL_SEGMENTS):
            angle = 2 * math.pi * segment / WHEEL_SEGMENTS
            u, v = radius * math.cos(angle), radius * math.sin(angle)
            if axis == "y":
                point = (u, end, v)
            else:
                point = (u, v, end)
            rings.append(tuple(point[i] + centre[i] for i in range(3)))

    vertices = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in rings]
    faces = []
    for segment in range(WHEEL_SEGMENTS):
        nxt = (segment + 1) % WHEEL_SEGMENTS
        a = offset + segment + 1
        b = offset + nxt + 1
        c = offset + WHEEL_SEGMENTS + nxt + 1
        d = offset + WHEEL_SEGMENTS + segment + 1
        faces.append(f"f {a} {b} {c} {d}")
    for base in (offset + 1, offset + WHEEL_SEGMENTS + 1):
        faces.append("f " + " ".join(str(base + s) for s in range(WHEEL_SEGMENTS)))
    return vertices, faces


def _write_obj(path: Path, name: str, blocks: list[tuple[list[str], list[str]]]) -> None:
    lines = [
        f"# {name} - generated by tools/generate_sim_meshes.py from",
        "# cad/parameters/parameters.csv.  Do not edit by hand: run the script.",
        "# units: metres (URDF reads this mesh with no scale factor)",
        f"o {name}",
    ]
    for vertices, _ in blocks:
        lines.extend(vertices)
    for _, faces in blocks:
        lines.extend(faces)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(repo_root: Path, out_dirs: list[Path] | None = None) -> None:
    cad = read_parameters(repo_root / "cad" / "parameters" / "parameters.csv")
    if out_dirs is None:
        out_dirs = [
            repo_root / "cad" / "urdf" / "meshes",
            repo_root / "cad" / "exports" / "meshes",
        ]

    blocks: list[tuple[list[str], list[str]]] = []
    offset = 0
    for box in body_boxes(cad):
        vertices, faces = _box_obj(box, offset)
        blocks.append((vertices, faces))
        offset += len(vertices)

    radius, height, centre_z = mast_dimensions(cad)
    vertices, faces = _cylinder_obj(radius, height, "z", (0.0, 0.0, centre_z), offset)
    blocks.append((vertices, faces))

    wheel_radius = cad["wheel_diameter_mm"] / 2 / MM_PER_M
    wheel_width = cad["wheel_width_mm"] / MM_PER_M
    wheel_block = _cylinder_obj(wheel_radius, wheel_width, "y", (0.0, 0.0, 0.0), 0)

    for directory in out_dirs:
        _write_obj(directory / "base_link.obj", "base_link", blocks)
        _write_obj(directory / "wheel.obj", "wheel", [wheel_block])


def _print_inertia() -> None:
    for name, properties in mass_properties().items():
        x, y, z = properties.com
        print(f"<!-- {name} -->")
        print("<inertial>")
        print(f'  <origin xyz="{x:.5f} {y:.5f} {z:.5f}" rpy="0 0 0"/>')
        print(f'  <mass value="{properties.mass:.6f}"/>')
        print(
            f'  <inertia ixx="{properties.ixx:.9f}" ixy="0" ixz="0"'
            f' iyy="{properties.iyy:.9f}" iyz="0" izz="{properties.izz:.9f}"/>'
        )
        print("</inertial>")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-inertia", action="store_true")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if arguments.print_inertia:
        _print_inertia()
    else:
        main(root)
        print(f"wrote sim meshes from {root / 'cad' / 'parameters' / 'parameters.csv'}")
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
pytest tests/unit/test_generate_sim_meshes.py -v
ruff check tools/ tests/unit/test_generate_sim_meshes.py
ruff format tools/ tests/unit/test_generate_sim_meshes.py
```

Expected: all PASS. If `test_the_centre_of_mass_is_low_enough_to_be_credible` fails, the mass model is wrong — fix `MASS_ITEMS` placement, do not relax the assertion.

- [ ] **Step 5: Generate the meshes and record the inertia**

```bash
python tools/generate_sim_meshes.py
python tools/generate_sim_meshes.py --print-inertia
```

Keep the printed `<inertial>` blocks — Task 7 pastes them into the URDF verbatim.

- [ ] **Step 6: Commit**

```bash
git add tools/ tests/unit/test_generate_sim_meshes.py cad/urdf/meshes/ cad/exports/meshes/
git commit -m "Generate the sim meshes, so Isaac stops waiting on a hand-built assembly"
```

---

### Task 7: the URDF

**Files:**
- Modify: `cad/urdf/weeding_rover.urdf` (whole file)
- Modify: `tests/unit/test_urdf_matches_cad.py:189-193` (`test_base_link_collision_is_a_box`), `:257-268` (`test_total_mass_is_under_the_gearbox_ceiling`)

**Interfaces:**
- Consumes: `parameters.csv` (Task 2), the meshes and `--print-inertia` output (Task 6), `wheel_v_max_mm_s 327` (Task 3)
- Produces: a URDF whose wheel joints sit at (±0.200, ±0.215, 0.125) with velocity 2.616 rad/s and effort 10.0 N·m

- [ ] **Step 1: Update the two tests whose assumptions changed**

These are the only two tests in the file that encode something no longer true. Both change **with the reason written in**, never by loosening an assertion.

Replace `test_base_link_collision_is_a_box`:

```python
def test_base_link_collision_is_boxes_that_clear_the_wheels():
    """Two boxes, not one, and neither may be a mesh.

    The frame necks in below the wheel tops -- chassis_plate_width is the wheel
    inner face spacing -- and the body above them overhangs.  One box cannot
    describe that shape without either clipping the wheels or throwing away the
    overhang, so the collision is the two boxes the shape actually has.

    cad/urdf/README.md#collision still holds: primitives only.
    """
    cad = _cad()
    collisions = _link("base_link").findall("collision")
    assert len(collisions) == 2

    boxes = []
    for collision in collisions:
        geometry = collision.find("geometry")
        assert geometry.find("box") is not None
        assert geometry.find("mesh") is None
        _, _, z = _xyz(collision.find("origin"))
        _, width, height = (float(v) for v in geometry.find("box").get("size").split())
        boxes.append((z - height / 2, z + height / 2, width))

    wheel_top = cad["wheel_diameter_mm"] / MM_PER_M
    inner_faces = (cad["track_width_mm"] - cad["wheel_width_mm"]) / MM_PER_M
    for bottom, top, width in boxes:
        if bottom < wheel_top - 1e-9:
            assert width <= inner_faces + 1e-9, (
                f"a collision box {width * MM_PER_M:.0f} mm wide reaches below the "
                f"wheel tops, where only {inner_faces * MM_PER_M:.0f} mm fits"
            )
        assert bottom >= cad["chassis_clearance_mm"] / MM_PER_M - 1e-9
```

Replace `test_total_mass_is_under_the_gearbox_ceiling`:

```python
def test_total_mass_is_under_the_gearbox_ceiling():
    """40 kg is what 4 x 5 N.m continuous moves up a 15 degree grade on a
    0.125 m wheel radius, against rolling resistance - hardware/bom/poc-v3.md.

    The old ceiling was 2.3 kg, from a 250:1 micro gearbox that is not on this
    machine.  The ceiling is a property of the drivetrain, so it moved with it;
    it is not a budget anyone may relax.  The estimate is 35 kg +/-20%, and the
    upper end of that band is 42 kg -- so this test is expected to fail if the
    real parts come in heavy, and the answer then is a stronger motor.
    """
    total = sum(
        float(link.find("inertial").find("mass").get("value"))
        for link in _robot().findall("link")
        if link.find("inertial") is not None
    )
    assert total <= 40.0, f"URDF total mass {total:.3f} kg exceeds the 40 kg ceiling"
```

- [ ] **Step 2: Run to verify the URDF tests fail**

```bash
pytest tests/unit/test_urdf_matches_cad.py -v
```

Expected: many FAIL — joint origins, wheel collision size, velocity limit, camera origins, collision box count, mass ceiling.

- [ ] **Step 3: Rewrite the URDF**

Replace `cad/urdf/weeding_rover.urdf`. Use the `<inertial>` blocks printed in Task 6 Step 5 verbatim for `base_link` and each wheel — do not retype the numbers.

Header comment:

```xml
<?xml version="1.0"?>
<!--
  weeding_rover - Rover Base V0, 650 x 520 mm, wheel O250, ~35 kg

  Frames follow hardware/mechanical/coordinate-frames.md: base_link sits at the
  centre of the rover on the soil reference plane, X forward, Y left, Z up.
  A positive wheel joint velocity about +Y drives that wheel forward, so
  omega > 0 (v_right > v_left) is a left turn, CCW with Z up.

  Units are metres and radians.  config/*.yaml is millimetres and degrees -
  this file is one of the only two places that conversion happens.

  Meshes under meshes/ are GENERATED by tools/generate_sim_meshes.py from
  cad/parameters/parameters.csv, authored in metres, so no scale factor is
  applied.  Do not hand-edit them; run the script.

  Collision is primitives only, per cad/urdf/README.md#collision.  base_link is
  TWO boxes, not one: the frame necks in to 340 mm between z 0.125 and 0.250
  where the wheels are, and the body above overhangs to 430 mm.  The wheels
  must NOT use a mesh - a tread mesh on a heightfield makes a very large number
  of contact points and buys no accuracy, because grip comes from the friction
  coefficient set in sim/isaac/robots/rover/rover.usd.

  Mass properties are an ESTIMATE from hardware/bom/poc-v3.md (+/-20%),
  computed analytically by tools/generate_sim_meshes.py --print-inertia.  Total
  35.0 kg against the 40 kg continuous-torque ceiling.  Replace with weighed
  values when real parts are chosen.
-->
<robot name="weeding_rover">
```

`base_link` — paste the generated `<inertial>`, then:

```xml
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/base_link.obj"/>
      </geometry>
    </visual>
    <!-- frame: necks in to the wheel inner faces, z 0.125 -> 0.250 -->
    <collision>
      <origin xyz="0 0 0.1875" rpy="0 0 0"/>
      <geometry>
        <box size="0.500 0.340 0.125"/>
      </geometry>
    </collision>
    <!-- body: overhangs the wheels, z 0.250 -> 0.405 -->
    <collision>
      <origin xyz="0 0 0.3275" rpy="0 0 0"/>
      <geometry>
        <box size="0.500 0.430 0.155"/>
      </geometry>
    </collision>
    <!-- mast: 35 mm tube, z 0.405 -> 0.905, carrying both cameras -->
    <collision>
      <origin xyz="0 0 0.655" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.0175" length="0.500"/>
      </geometry>
    </collision>
```

Each of the four wheels — paste the generated wheel `<inertial>` into all four, then:

```xml
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/wheel.obj"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963268 0 0"/>
      <geometry>
        <cylinder radius="0.125" length="0.090"/>
      </geometry>
    </collision>
```

Wheel joints, with the block comment updated:

```xml
  <!-- ================================================================== -->
  <!-- wheels - four continuous joints, star topology off base_link       -->
  <!--                                                                    -->
  <!-- origin  x = +/- wheelbase/2      = +/- 0.200                       -->
  <!--         y = +/- track_width/2    = +/- 0.215                       -->
  <!--         z =     wheel_diameter/2 =     0.125  (the belly plane is  -->
  <!--                 the axle plane: ground clearance equals the radius)-->
  <!-- velocity 327 mm/s / 125 mm       = 2.616 rad/s                     -->
  <!-- effort   gearbox continuous limit = 10.0 N.m                       -->
  <!-- ================================================================== -->
```

with, per wheel (`wheel_fl` shown; signs per the `WHEELS` table in the test):

```xml
  <joint name="wheel_fl" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fl_link"/>
    <origin xyz="0.200 0.215 0.125" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10.0" velocity="2.616"/>
  </joint>
```

Cameras — both in the mast head at z = 0.850. Keep the existing tiny sensor-box links and masses; only the joint origins change:

```xml
  <joint name="camera_front" type="fixed">
    <parent link="base_link"/>
    <child link="camera_front_link"/>
    <!-- tilt 50 deg below horizontal.  roll = -(90 + tilt) in radians, NOT
         -(180 - tilt): the two agree at exactly 45 deg, which is the angle
         this file used to carry, so the wrong formula reproduces the old
         value and then drifts.  -(90 + 50) = -140 deg = -2.4434609528. -->
    <origin xyz="0 0 0.850" rpy="-2.4434609528 0 -1.5707963268"/>
  </joint>
```

```xml
  <joint name="camera_down" type="fixed">
    <parent link="base_link"/>
    <child link="camera_down_link"/>
    <origin xyz="0 0 0.850" rpy="3.1415926536 0 -1.5707963268"/>
  </joint>
```

- [ ] **Step 4: Verify the front camera tilt arithmetic before trusting it**

The roll value encodes the tilt and is the easiest thing in the file to get wrong — and it has a trap. The correct relation is `roll = -(90 + tilt)`. The plausible-looking `-(180 - tilt)` gives the **same answer at 45°**, which is exactly the angle the old URDF carried, so a wrong formula reproduces the old file perfectly and then drifts at every other angle. Do not derive this by pattern-matching the old value; check it:

```bash
python -c "
import math, sys
sys.path.insert(0, 'tests/unit')
from test_urdf_matches_cad import _optical_axis
axis = _optical_axis(-2.4434609528, 0.0, -1.5707963268)
below = math.degrees(math.atan2(-axis[2], math.hypot(axis[0], axis[1])))
print('optical axis', [round(v, 6) for v in axis])
print('degrees below horizontal', round(below, 6))
print('lookahead mm', round(850 / math.tan(math.radians(below)), 1))
"
```

Expected exactly:

```text
optical axis [0.642788, -0.0, -0.766044]
degrees below horizontal 50.0
lookahead mm 713.2
```

If it prints 40.0, the `-(180 - tilt)` form was used. If it prints 130 or −50, the sign is wrong.

- [ ] **Step 5: Run the URDF tests to verify they pass**

```bash
pytest tests/unit/test_urdf_matches_cad.py -v
```

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add cad/urdf/weeding_rover.urdf tests/unit/test_urdf_matches_cad.py
git commit -m "Describe a rover three times the size, and let the old tests object first"
```

---

### Task 8: Isaac physics — retune, do not edit

Every physics value in `sim/isaac/robots/rover/config.yaml` was measured against a 1.038 kg rover with a 25 mm wheel contact. None of it transfers to 35 kg on a 90 mm contact. The file's own comments say the yaw problem may not be a friction problem at all — `V1-ISAAC.md#risks` §3 names *revisiting the wheel and running gear* as the lever, and this task is the first time that lever has actually been pulled.

**Files:**
- Modify: `sim/isaac/robots/rover/config.yaml`, `sim/isaac/robots/rover/rover.usd`
- Read only: `tests/unit/test_rover_usd_override.py` (it parses both; no edit expected)

**Interfaces:**
- Consumes: the URDF (Task 7)
- Produces: physics values in `config.yaml` that `rover.usd` restates verbatim

- [ ] **Step 1: Regenerate the base USD and confirm it loads**

```bash
./scripts/import_urdf.sh
```

Expected: `sim/isaac/robots/rover/rover_base.usd` regenerated (gitignored). If the importer errors on the second `<collision>` element, that is a real finding — record it and stop; the two-box collision may need to become one box plus a separate collision link.

- [ ] **Step 2: Update the contact offset, which is the one value that is arithmetic not tuning**

In `sim/isaac/robots/rover/config.yaml`, the contact offset was set as a fraction of the wheel radius. The radius went 0.035 → 0.125, so the same reasoning gives a different number:

```yaml
collision:
  # Isaac's default contact offset is 0.02 m, which was 57% of the old 0.035 m
  # wheel radius.  At 0.125 m the default is 16%, which is still enough to
  # generate contacts before the wheel touches.  Held at the same ~3% of radius
  # the small rover used.
  contact_offset_m: 0.004
  rest_offset_m: 0.0
```

Keep `0.004` — it is 3.2% of the new radius, where it was 11% of the old. Update the comment to say so, as above.

- [ ] **Step 3: Rewrite the friction comment block to state what is now unknown**

The measurement table in `config.yaml` is evidence about a machine that no longer exists. Replace the `friction:` comment block with:

```yaml
friction:
  # ⚠️ THESE VALUES ARE INHERITED FROM A 1.038 kg ROVER AND ARE NOT VALID.
  #
  # The table that used to be here measured a 1.038 kg rover on 0.035 m wheels
  # with a 25 mm line contact.  This machine is 35 kg on 0.125 m wheels with a
  # 90 mm contact and 3.6x the track.  Every row of that table is about a
  # different vehicle, so it has been removed rather than left to be misread.
  #
  # What the old table did establish, and what carries over as a QUESTION
  # rather than an answer: straight-line driving was right at every friction
  # setting, turning was not, and lowering friction by a factor of 45 moved the
  # yaw from 1.10 to 1.57 deg/s against a commanded 40.  The conclusion was
  # that friction is not the knob -- the contact model is -- and that the real
  # lever is the wheel and running gear (V1-ISAAC.md#risks section 3).
  #
  # This change pulls that lever for the first time: the contact is now 90 mm
  # wide instead of 25, on a wheel 3.6x the radius, at 34x the mass.  Whether
  # that fixes the yaw is the first thing Stage 3 must measure, BEFORE touching
  # any number in this file.
  #
  # Re-measure on flat ground, 3 s per case, and put the new table here:
  #   drive(160, 0)  -> expect 160 mm/s
  #   drive(0, 25)   -> expect 25 deg/s     <-- the one that was broken
  #   drive(160, 25) -> expect the mixing table's 66.2 / 253.8 mm/s
  #
  # Do NOT keep lowering `static` to chase the yaw.  That was the trap last
  # time and the file said so.
  static: 0.15
  dynamic: 0.12
  restitution: 0.0
```

Also update the `wheel_drive` `max_force` comment, which cites the old gearbox:

```yaml
  # max_force is NOT authored here on purpose.  It is the gearbox's continuous
  # torque limit (10 N·m), it comes from the URDF's <limit effort=...>, and
  # rover_base.usd already carries it.  Restating it here would put a BOM
  # number in two places, which is exactly the drift the two-layer split exists
  # to prevent.
```

- [ ] **Step 4: Mirror every change into rover.usd and verify the two agree**

`rover.usd` must restate each value in `config.yaml`. Update the `maxForce` comment on all four wheel drive blocks from `0.4903 N.m` to `10 N.m`, then:

```bash
pytest tests/unit/test_rover_usd_override.py -v
```

Expected: all PASS. The test parses both files and asserts every `config.yaml` value appears in `rover.usd`; a mismatch here means one file was edited and the other was not.

- [ ] **Step 5: Run the full unit suite**

```bash
pytest tests/unit -v
```

Expected: all PASS. This is the first point in the plan where the whole suite should be green.

- [ ] **Step 6: Commit**

```bash
git add sim/isaac/robots/rover/config.yaml sim/isaac/robots/rover/rover.usd
git commit -m "Retire a friction table that measured a different vehicle"
```

> **Stage 3 note for whoever runs Isaac:** the empirical re-tuning (measuring the three drive cases above, then adjusting damping and friction) is deliberately NOT in this plan. It needs a running Isaac session and it is the longest job in V1. This task only puts the file into a state where the old numbers cannot be mistaken for measurements.

---

### Task 9: documentation

The numbers are now right everywhere they are executed. This task makes them right everywhere they are read.

**Files:**
- Modify: `cad/parameters/README.md`, `cad/README.md`, `config/README.md`, `hardware/mechanical/dimensions.md`, `hardware/mechanical/assembly.md`, `hardware/electrical/power.md`, `hardware/electrical/wiring.md`, `README.md`, `docs/architecture.md`

**Interfaces:**
- Consumes: everything above
- Produces: no code interface

- [ ] **Step 1: Rewrite cad/parameters/README.md**

This is the largest doc change. It must carry:

- the parameter tables from Task 2 (Chassis / Drive / Mast / Camera), replacing the old values
- the three derivations now enforced by tests: `overall_length = wheelbase + wheel_diameter`, `body_width = max(chassis_plate_width, track + wheel_width)`, `chassis_plate_width ≤ track − wheel_width`
- the replacement of the `body_width` section's reasoning: the `track 140 → 120` story is history about a different machine; keep it as a one-line pointer to git history and replace the live text with the 340/380 neck-in (design §3.2)
- the **Coverage Checks** section recomputed:

```text
กล้องล่างต้องครอบ corridor + ขอบ
  corridor              610 mm   (clear_furrow 690 − 2 × max_lateral_error 40)
  ต้องครอบ + ขอบข้างละ 20  650 mm
  HFOV ที่ต้องการ >= 2 × atan(325 / 850) = 41.9°   →  ต้องการ >= 45°

ต้องไม่มีช่องว่างระหว่างเฟรม
  เซนเซอร์ down คือ 1280 × 720 (16:9) จาก config/simulation.yaml
  VFOV ที่ HFOV 45° = 2 × atan(tan(22.5°) × 720/1280) = 26.2°
  coverage ตามแนววิ่ง = 2 × 850 × tan(13.1°) ≈ 396 mm
  ระยะที่รถวิ่งต่อเฟรม = 160 / 2 = 80 mm
  396 > 80  ✓

กล้องหน้า — lookahead
  lookahead = 850 / tan(50°) = 713 mm  = 1.1 เท่าของความยาวรถ
  เวลาที่ได้ = 713 / 160 = 4.5 s
```

- the **Derived Values** table, with `motor_mount_pitch_mm` removed, `mast_*` and `overall_length_mm` added, and the `wheel_v_max` row citing `poc-v3.md`
- the **Startup invariants** block at the new numbers (design §7)
- the **ค่าที่ตั้งไว้และควรตรวจสอบก่อนสร้างจริง** table updated: `motor_rpm ~25`, `wheel_v_min` at V3, down-camera FOV ≥ 45° at 850 mm, `crop_foliage_half_width` **< 55 mm**, `soil_variation ≤ 62` (no longer the binding one)

- [ ] **Step 2: Update cad/README.md**

In **⚠️ พารามิเตอร์ที่แก้แล้วกระทบมากกว่าที่คิด**, the mechanism table is still correct — only the例 values change. Add one row:

```text
| `chassis_plate_width_mm` | ถ้าเกิน track − wheel_width โครงชนล้อ — `test_the_chassis_plate_clears_the_inner_faces_of_the_wheels` |
```

In **Pipeline**, add the generator:

```text
cad/parameters/parameters.csv
   │
   ├── tools/generate_sim_meshes.py ──> exports/meshes/ · urdf/meshes/
   │                                     (generated — ห้ามแก้ด้วยมือ)
   └── Fusion 360 (งานมือ) ──────────> exports/step/ · exports/stl/
                                         (สำหรับผลิต ตามมาทีหลังได้)
```

- [ ] **Step 3: Update config/README.md**

Recompute every derivation the file states: `clear_furrow = 750 − 60 = 690`, the nine invariants at the new numbers, and the `row_spacing_mm` history row (`250 → 350 → 750`). Add the foliage budget as its own subsection — it is the machine's binding constraint now:

```text
### `crop_foliage_half_width_mm` คือ margin ของ invariant ข้อ 5 ทั้งก้อน

row_spacing 750 ผ่านตราบใดที่ crop_foliage_half_width < 55 mm
ถ้า V1 วัดได้ 55+ → row_spacing ต้องเป็น 800+

นี่ไม่ใช่การแก้ config — เป็นข้อจำกัดว่าแปลงต้องปลูกห่างเท่าไร
```

- [ ] **Step 4: Rewrite hardware/mechanical/dimensions.md**

Replace all geometry with the Task 2 values. Add the top/side views at the new scale, the wheel-centre table from design §4, and the two layout rules the design established:

```text
ห้ามมีอะไรต่ำกว่าแนวศูนย์กลางเพลา (ground clearance = wheel radius)
โครงช่วง z 125–250 กว้างได้ไม่เกิน 340 mm
```

- [ ] **Step 5: Update the remaining docs**

| File | Change |
|---|---|
| `hardware/mechanical/assembly.md` | 35 kg is not a one-person lift — two people or a ramp. Wheel/motor removal via the 60 mm bolt circle. |
| `hardware/electrical/power.md` | 12 V → **24 V** rail, 480 Wh pack, ~110 W average, ~4.4 h, driver ≥ 10 A/channel |
| `hardware/electrical/wiring.md` | 24 V motor rail, E-stop relay current rating, separation of the 10 A drive loom from signal |
| `README.md` | machine class and `row_spacing 750` in the MVP description; link the new spec |
| `docs/architecture.md` | line 8's form-factor sentence: still "วิ่งในร่องระหว่างแถวปลูก", but the furrow is now 750 mm because the rover is 520 |

`hardware/mechanical/coordinate-frames.md` needs **no change** — the conventions (X forward, Y left, Z up, origin on the soil plane) are size-independent. Confirm with:

```bash
pytest tests/unit/test_coordinate_frames.py -v
```

- [ ] **Step 6: Verify the whole repo**

```bash
pytest tests/unit -v
ruff check .
ruff format --check .
grep -rn "202\b\|track_width.*120\|wheel_diameter.*70\|row_spacing.*350" --include=*.md --include=*.yaml --include=*.csv . | grep -v "\.git/"
```

The grep should return only intentional historical references (git-history pointers, the superseded `poc-v2.md`, and the two older spec documents). Anything else is a doc that still describes the old machine.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "Tell the documentation which rover it is describing"
```

---

## Self-Review

**Spec coverage.** Every design section maps to a task: §3 contradictions → Task 2 tests · §4 parameters → Task 2 · §5 bed → Task 3 · §6 drive and perception → Tasks 3, 4 · §7 invariants → Task 3 · §8 BOM → Task 1 · §9.1 mesh generator → Task 6 · §9.2 URDF → Task 7 · §10 file list → all · §11 verification → each task's own pytest command · §12 risks → carried into config comments (Tasks 3, 4), `config.yaml` (Task 8) and docs (Task 9) · §13 P0–P7 → Tasks 1–9 · §14 out of scope → nothing here touches tool, Fusion, STEP/STL, part numbers or Jetson.

**Deviation from the spec, recorded deliberately:** spec §6.3 computed the down camera's along-track coverage as 527 mm assuming a 4:3 sensor. `config/simulation.yaml` specifies the down camera as 1280 × 720, which is 16:9, giving **396 mm**. Task 9 Step 1 uses the corrected figure. It changes no decision — 396 mm is still ~5× the 80 mm travelled per frame.

**Spec §12 risk 6** (35 kg is not a one-person lift) appears only in documentation, which is correct: it has no executable consequence.

**One error caught during self-review and fixed inline:** the front camera's roll was first written as `-2.2689280276`, derived by pattern-matching the old URDF's `-2.3561944902`. Running it through the test's own `_optical_axis` helper showed it encodes **40°**, not 50°. The correct relation is `roll = -(90 + tilt)`, and `-(180 - tilt)` — the shape the old value also fits — agrees with it at exactly 45°, the angle the old file used. Task 7 Step 4 now states the trap and asserts the exact expected output.

**Placeholders:** none. Every code step carries the code; every config step carries the file content; the one place a number is not written out (the URDF `<inertial>` blocks) is generated by a script written in the preceding task, with the exact command to produce it.

**Type consistency:** `read_parameters`, `read_obj_vertices`, `mass_properties`, `main(repo_root, out_dirs)`, `LinkInertia` and `Box` are defined in Task 6 and used with those exact signatures in its tests. `_cad()`, `_link()`, `_joint()`, `_xyz()`, `_optical_axis()` in Task 7 are the helpers already present in `test_urdf_matches_cad.py`. `_cad_parameters()` in Task 2 is the existing helper in `test_cad_config_sync.py`. `startup_checks.margins()` and `startup_checks.ConfigInvalid` in Task 3 exist in `controller/startup_checks.py`.
