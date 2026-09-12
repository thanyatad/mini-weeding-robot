# Coordinate Frames

นิยาม frame ทุกตัวในระบบและความสัมพันธ์ระหว่างกัน

เอกสารนี้เป็นจุดอ้างอิงเดียวสำหรับ `perception/`, `cad/urdf/`, `sim/isaac/`
และการ calibrate — ถ้าเอกสารนี้กับ code ไม่ตรงกัน **เอกสารนี้ผิด**
ให้แก้เอกสาร ไม่ใช่แก้ code ให้เข้ากับเอกสารที่ผิด

---

## โซ่ที่ขาดโดยเจตนา — อ่านข้อนี้ก่อน

```text
เดิม (gantry)   world → bed → machine → tool
                             └─► camera         โซ่ต่อเนื่อง machine อยู่นิ่ง

ใหม่ (rover)    world → bed  ┄┄ ไม่มี ┄┄  rover ──┬─► camera_front
                                                  └─► camera_down
```

**`bed → rover` ไม่มีใครรู้ค่า runtime** เพราะ MVP ไม่มี localization
ไม่มี encoder ไม่มี odometry

ทุกอย่างใต้ `rover` ทำงานใน **rover frame เท่านั้น**

นี่คือ **ช่องว่างที่ประกาศไว้ ไม่ใช่ของที่ลืมเขียน** — ใครเขียน code ที่ต้องการ
พิกัดใน `bed` frame ต้องหยุดแล้วไปทำ **M2 (wheel encoder)** ไม่ใช่เดาค่ามาเติม

ผลที่ตามมาที่ต้องรู้:

```text
weed_log มีแต่พิกัดใน rover frame        ไม่มีพิกัดในแปลง
ไม่มี weed id                            ไม่รู้ว่าเฟรม 123 กับ 124 เป็นต้นเดียวกันไหม
log นับ "การตรวจจับ" ไม่ใช่ "จำนวนต้น"    ตัวเลขรวมจาก log ไม่มีความหมาย
```

---

## Frames

| Frame | Origin | แกน | ผูกกับ |
|---|---|---|---|
| `world` | Isaac world origin | Z ขึ้น | — (simulation เท่านั้น) |
| `bed` | มุมแปลง X min / Y min บน **soil reference plane** | X ตามความยาวร่อง, Y ตามความกว้างแปลง, Z ขึ้น | โครงสร้างจริง |
| `rover` | กึ่งกลางตัวรถ ที่ระดับ soil reference plane | X เดินหน้า, Y ซ้าย, Z ขึ้น | **ไม่ผูกกับอะไร — ไม่มี localization** |
| `camera_front` | optical center | ตาม OpenCV (Z ออกจากเลนส์) | `rover` แบบ fixed |
| `camera_down` | optical center | ตาม OpenCV | `rover` แบบ fixed |

```text
       world  (sim เท่านั้น)
         │
         ▼
       bed  ◄────────────── soil reference plane, Z = 0
         ┊
         ┊  ไม่มี transform ที่ runtime
         ┊
       rover  ──┬──► camera_front     fixed, tilt 50°, สูง 850 mm
                └──► camera_down      fixed, tilt 0°,  สูง 850 mm
```

`bed` ยังมีอยู่ในเอกสารและใน sim (Isaac รู้ ground truth) แต่ **controller ไม่รู้** —
ห้ามเอา ground-truth pose จาก Isaac มาใช้ใน `controller/` หรือ `perception/`
ไม่งั้น code จะทำงานใน sim แล้วพังบน hardware

---

## `rover` — frame หลักของ MVP

```text
X > 0    เดินหน้า
Y > 0    ซ้าย
Z > 0    ขึ้น
Z = 0    soil reference plane
```

Right-hand rule, Z ขึ้น → **`omega > 0` คือเลี้ยวซ้าย (CCW)**

ค่านี้ต้องตรงกันทั้ง 3 ที่:

```text
Rover.drive(v, omega)                 controller/rover/base.py
skid-steer mixing                     v_right = v + omega_rad × track/2
wheel joint velocity ใน Isaac         cad/urdf + sim/isaac/adapters/
```

ถ้าที่ใดที่หนึ่งกลับเครื่องหมาย rover จะเลี้ยวสวนทางที่ row follower ต้องการ
แล้วมันจะ**ออกนอกร่องเร็วขึ้น** ไม่ใช่แค่ช้าลง — เพราะ feedback กลายเป็น positive

มี unit test บังคับ: `lateral_err > 0` (ร่องอยู่ขวา) ต้องได้ `omega < 0`

---

## `bed` — ยังนิยามไว้ เพราะ sim และเอกสารต้องใช้

```text
X = 0 … 2000 mm     ความยาวร่อง
Y = 0 … 1000 mm     ความกว้างแปลง  ⚠️ ไม่มีที่มาที่ตรวจสอบได้ — ดู docs/simulation.md
Z = 0               soil reference plane
```

**Soil reference plane ไม่ใช่ผิวดินจริง** — เป็นระนาบอ้างอิงที่ผิวดินจริง
แกว่งรอบ ๆ ±`soil_variation_mm` (15 mm)

```text
        ┌─── ยอดดิน          Z = +15
────────┼───────────────────  Z =  0   soil reference plane
        └─── ก้นร่อง          Z = −15
```

การสับสนระหว่าง "reference plane" กับ "ผิวดินจริง" เป็นต้นเหตุของ error
ใน homography ของ `camera_down` — ต้องแยกให้ชัดเสมอ

### ความกว้างที่ rover ใช้จริงไม่ใช่ `row_spacing`

```text
row_spacing               750 mm   กึ่งกลางแถวถึงกึ่งกลางแถว
crop_foliage_half_width    30 mm   ใบยื่นออกจากกึ่งกลางแถวข้างละเท่านี้
clear_furrow              690 mm   750 − 2 × 30   ◄── ค่าที่ invariant ใช้
```

ช่องว่างข้างละ `(690 − 520) / 2 = 85 mm` สำหรับ rover กว้างรวมล้อ 520 mm

`body_width` = **จุดกว้างสุด** = `track_width 430 + wheel_width 90` ไม่ใช่ความกว้างโครงช่วงล้อ (340 mm) — ส่วนที่ชนใบพืชคือล้อ ไม่ใช่ตัวถัง

---

## `camera_front` — ไม่มี extrinsic ที่ต้องแม่น

```text
ตำแหน่งใน rover frame:  x = 0,  z = +850 mm,  tilt 50° ก้มลง
lookahead = 850 / tan(50°) = 713 mm ข้างหน้า
```

**ไม่ calibrate** — `RowEstimate` เป็นค่า image-space ล้วน:

```python
lateral_err = (valley_near - u_center) / (width / 2)      # [-1, 1]
heading_err = (valley_far  - valley_near) / (width / 2)   # [-1, 1]
```

`heading_err` คำนวณจากผลต่างของ valley สองระยะ **ในภาพเดียวกัน** ซึ่งไม่ต้องรู้
focal length ความสูง หรือมุมก้มเลย

### ⚠️ แต่ extrinsic ผูกกับ gain

มุมก้มและความสูงกำหนด **lookahead distance** ซึ่งเป็นตัวกำหนดว่า gain เท่าไรจึงลู่เข้า

```text
ขยับขายึดกล้อง → gain ที่ tune ไว้ใช้ไม่ได้ → ต้อง tune ใหม่ที่ V1 และ V4
```

**ไม่มี test จับข้อนี้ได้** — `test_cad_config_sync.py` ตรวจว่าตัวเลขตรงกัน
แต่ตรวจไม่ได้ว่า gain ยังเหมาะกับมุมใหม่

นี่เป็น coupling ระหว่าง mechanical กับ control ที่ gantry ไม่มี
(กล้อง gantry มองตรงลงและอยู่กับที่ มุมไม่มีผลต่อ control)

---

## `camera_down` — ตัวเดียวที่ calibrate

```text
ตำแหน่งใน rover frame:  x = 0,  z = +850 mm,  tilt 0° (มองตรงลง)
```

กล้องทั้งสองตัวอยู่บน mast เดียวกันที่ z = 850 mm — กล้องล่างต้องขึ้นมาจากตัวถัง
เพราะล้อ Ø250 บังพื้นที่มองตรงลงจนเหลือน้อยเกินไป (ดู design spec §6.3)

ใช้ convention ของ OpenCV:

```text
X → ขวาในภาพ
Y → ลงในภาพ
Z → ออกจากเลนส์ (ทิศที่มอง)
```

ความสัมพันธ์ `camera_down → rover` คือสิ่งที่ **calibration** หา และเป็น homography
ที่ `perception/weed_detector.py` ใช้แปลง pixel → mm ใน rover frame

**ไม่ใช่ `camera → bed` เหมือน design gantry** — ปลายทางคือ rover frame เท่านั้น

ค่าเริ่มต้นมาจาก CAD (`camera_down_*`) แต่ **ต้อง calibrate ทับเสมอ** —
ตำแหน่งติดตั้งจริงคลาดจาก CAD ได้หลายมิลลิเมตร

`tilt = 0` ตั้งใจ: ยิ่งเอียงยิ่งทำให้ error จากดินไม่เรียบใหญ่ขึ้น
Error budget ของ MVP คือ **±20 mm** (ผ่อนจาก ±8 mm ของ gantry เพราะไม่มี tool ที่ต้องเล็ง)
ดู [../../docs/calibration.md](../../docs/calibration.md#soil-plane-error-budget)

---

## สิ่งที่หายไปจาก design gantry

| หายไป | เพราะ |
|---|---|
| `machine` frame | ไม่มีโครงที่อยู่นิ่งให้เป็น frame |
| `tool` frame | ไม่มี tool ใน MVP (กลับมาที่ M3) |
| `home_offset_mm` | ไม่มี homing ไม่มี endstop จึงไม่มี offset ให้วัด |
| `bed → machine` transform | แทนที่ด้วย **ช่องว่างที่ประกาศไว้** |

`home_offset_mm` เคยเป็น "กับดักที่ลืมวัดแล้วพลาดทุกจุดอย่างเป็นระบบ"
ของ design gantry — MVP นี้ไม่มีมัน แต่มีกับดักที่แย่กว่าแทน:
**สัญลักษณ์ของ `omega` ที่กลับด้าน** ซึ่งไม่ได้ทำให้พลาดเท่า ๆ กัน
แต่ทำให้ระบบไม่เสถียร

---

## Unit Convention

ความผิดพลาดเรื่องหน่วยตรงรอยต่อเป็น bug ที่หายากที่สุดในระบบนี้

| ที่ | ความยาว | มุม / ความเร็วเชิงมุม |
|---|---|---|
| `config/*.yaml` | **mm** | **degree** / deg/s |
| `protocol/` JSON | **mm** | degree / deg/s |
| `cad/parameters/` | **mm** | degree |
| `cad/urdf/` | **m** | **radian** / rad/s |
| Isaac Sim / USD | **m** | radian / rad/s |
| OpenCV | pixel → mm | radian |

จุดแปลงหน่วยมีแค่ 2 ที่ และต้องมี unit test ทั้งคู่:

```text
cad/urdf              ↔  config     mm ↔ m,  deg ↔ rad
sim/isaac/adapters/   ↔  Rover      mm ↔ m,  deg/s ↔ rad/s
```

ทุก field ที่เป็นความยาวใน config และ protocol ลงท้ายด้วย `_mm` และความเร็ว
เชิงมุมลงท้ายด้วย `_deg_s` โดยบังคับ เพื่อให้ความผิดพลาดเห็นได้ตอนอ่าน code
ไม่ใช่ตอน debug

### `RowEstimate` ไม่มีหน่วย — โดยเจตนา

```python
lateral_err: float      # [-1, 1]   unitless
heading_err: float      # [-1, 1]   unitless
```

กล้องหน้าไม่ calibrate ค่าที่วัดได้จากภาพที่ไม่ calibrate คือ **pixel ratio**
ถ้าประกาศเป็น mm หรือ deg จะเป็นการอ้างหน่วยที่ไม่มีใครยืนยันได้

ผลคือ gain มีหน่วย `(deg/s) / unitless` และ **ต้อง tune** ไม่มีสูตรคำนวณ
จากเรขาคณิต — ยอมรับตรง ๆ ดีกว่าซ่อนใต้หน่วยปลอม

---

## Verification

```text
tests/unit/test_coordinate_frames.py
```

ต้องครอบคลุม:

```text
[ ] sign convention: lateral_err > 0  →  omega < 0
[ ] sign convention: omega > 0  →  v_right > v_left  (เลี้ยวซ้าย)
[ ] mm ↔ m conversion ที่ขอบ urdf และ adapter
[ ] deg ↔ rad conversion ที่ขอบ urdf (omega_max 25° = 0.436 rad/s)
[ ] pixel → rover-frame → pixel round-trip (camera_down)
[ ] ไม่มี code ใน controller/ หรือ perception/ ที่อ่าน ground-truth pose จาก Isaac
```

ข้อสุดท้ายเป็น test ระดับ static check (grep / import graph) ไม่ใช่ runtime —
มันกันความผิดพลาดที่ทำให้ระบบ **ทำงานได้ใน sim แล้วพังบน hardware**
ซึ่งเป็นความผิดพลาดที่แพงที่สุดในโครงสร้างแบบนี้
