# perception

สอง pipeline แยกกันสมบูรณ์ แชร์เฉพาะ ExG

```text
                  ┌── exg.py (shared) ──┐
                  ▼                     ▼
camera.front ─> row_estimator.py    weed_detector.py <─ camera.down
                  │                     │
                  ▼                     ▼
             RowEstimate            Detection[]  ──> weed_log.py
             (control path)         (observation path)
```

**control path กับ observation path ต้องไม่ปนกัน** — detection ช้าหรือพลาด
ต้องไม่ทำให้ rover เลี้ยวผิด front ทำงานที่ 10 Hz · down ที่ 1-2 Hz

---

## Responsibility

- Camera input (2 ตัว)
- ExG segmentation
- Row estimate (navigation)
- Weed detection (observation)
- Weed log
- Calibration (กล้องล่างเท่านั้น)

**ไม่** รับผิดชอบ: motion, rover control, row following policy

---

## Layout

```text
perception/
├── cameras.py              # CameraSet interface + Fake / Isaac / Usb
├── exg.py                  # ExG + Otsu + absolute floor       shared
├── row_estimator.py        # front  → RowEstimate
├── weed_detector.py        # down   → Detection[]
├── weed_log.py             # JSONL writer
├── calibration.py          # intrinsic + homography (กล้องล่าง)
└── models/                 # trained weights — ว่างใน MVP (ใช้ที่ M5)
```

ไม่มี `row_detection.py` · ไม่มี `coordinate_transform.py` สำหรับ navigation —
เหตุผลอยู่ที่ [§weed_detector](#weed_detectorpy--เรขาคณิตของ-rover-ทำงานแทน-row-fitting)
และ [§Calibration](#calibration--เหลือแค่กล้องล่าง)

---

## CameraSet Abstraction

```python
class CameraSet(Protocol):

    def front(self) -> Frame:
        """สำหรับ row following"""
        ...

    def down(self) -> Frame:
        """สำหรับ weed detection"""
        ...
```

```text
CameraSet
│
├── FakeCameraSet
├── IsaacCameraSet
└── UsbCameraSet
```

เลือก **named accessor** ไม่ใช่ `capture(camera_id)` เพราะสองกล้องนี้ต่างกันที่
contract ไม่ใช่ต่างกันที่ index:

| | `front()` | `down()` |
|---|---|---|
| ใช้ทำ | row following | weed detection |
| Calibration | **ไม่ต้อง** | homography → mm |
| Rate | 10 Hz (อยู่ใน control loop) | 1-2 Hz |
| ความละเอียด | 640×480 → ย่อ 320×240 | 1280×720 |

`capture(id)` จะซ่อนความต่างนี้ แล้วมีคนเผลอเอาภาพ `front` ไปทำ coordinate transform

---

## `exg.py` — ExG + adaptive threshold + absolute floor

```python
exg = 2*G - R - B
if exg.max() < exg_floor:
    return Mask.empty()        # ไม่มีเขียวจริง — ไม่เรียก Otsu
mask = exg > otsu(exg)
```

### ทำไม ExG ไม่ใช่ color threshold ตรง ๆ

**ExG (Excess Green Index)** = `2G − R − B` วัด *ความเขียวเชิงสัมพัทธ์*
ไม่ใช่ค่าสีดิบ จึงทนต่อการเปลี่ยนแปลงของแสงและสีดินมากกว่า
เป็น index มาตรฐานในงาน agricultural vision

ดินแห้งกับดินเปียกสีต่างกันมาก — threshold คงที่บนค่าสีดิบต้อง tune ใหม่ทุกครั้งที่รดน้ำ

### ทำไม Otsu ต่อเฟรม ไม่ใช่ค่าคงที่

สีดินเปลี่ยนตามความชื้นและแสง Otsu หา threshold จากฮิสโทแกรมของเฟรมนั้นเอง
ไม่ต้องมีใครมาตั้งค่า

### ⚠️ จุดตายของ Otsu — และเหตุผลของ `exg_floor`

**เมื่อไม่มีสีเขียวในเฟรมเลย Otsu จะแบ่ง noise ออกเป็นสองกอง** แล้วคืน mask
ที่ดูเหมือนมีพืช ทำให้ heuristic `row_end` ตรวจไม่เจอว่าสุดร่องแล้ว

`exg_floor` เป็นพื้นกันข้อนี้ และเป็น **ค่าเดียวใน pipeline ที่ต้อง tune ด้วยมือ**

ต้องมี unit test ป้อนภาพดินเปล่ายืนยันว่าได้ mask ว่าง — อยู่ใน acceptance criteria ของ V0

---

## `row_estimator.py` — valley histogram ไม่ใช่ line fitting

สิ่งที่ rover ต้องตามคือ **ร่องดิน** ซึ่งคือหุบเขาระหว่างแถบเขียวสองแถบ

```text
column-wise green sum ของแถบภาพหนึ่งแถบ:

  ███                       ███        ← แถวพืชซ้าย / ขวา
  ███▄                     ▄███
  █████▄        ▼         ▄█████
  ───────────────────────────────────  u
              valley = กลางร่อง
```

สองแถบ สองหน้าที่:

```python
valley_near = find_valley(mask[bottom_third])    # ใกล้ล้อ
valley_far  = find_valley(mask[middle_third])    # lookahead

lateral_err = (valley_near - u_center) / (width / 2)
heading_err = (valley_far - valley_near) / (width / 2)
confidence  = valley_prominence                  # ความลึกของหุบเทียบยอดสองข้าง
valid       = confidence > conf_min  (ทั้งสองแถบ)
```

### `RowEstimate`

```python
@dataclass(frozen=True)
class RowEstimate:
    valid: bool
    lateral_err: float      # [-1, 1]  ร่องเบี่ยงจากกลางภาพ  (+ = ร่องอยู่ขวา)
    heading_err: float      # [-1, 1]  ความเอียงของเส้นร่องในภาพ  (+ = เอียงขวา)
    confidence: float       # [0, 1]
    green_fraction: float   # [0, 1]  สัดส่วน pixel ที่ ExG ผ่าน threshold
```

**ไม่มีหน่วย mm หรือ deg ทั้ง struct โดยเจตนา** — กล้องหน้าไม่ calibrate
ค่าที่วัดได้จากภาพที่ไม่ calibrate คือ pixel ratio ถ้าประกาศเป็น mm หรือ deg
จะเป็นการอ้างหน่วยที่ไม่มีใครยืนยันได้

gain ของ controller จึงมีหน่วย `(deg/s) / unitless` และ **ต้อง tune** —
ยอมรับตรง ๆ ดีกว่าซ่อนใต้หน่วยปลอม

### ทำไม valley histogram แทน line fitting / Hough

| | valley histogram | line fitting |
|---|---|---|
| ต้นหายกลางแถว | ทนได้ — histogram ยังเห็นแถบ | เสี่ยง fit เพี้ยนเพราะ blob หาย |
| ร่องโค้ง | ทนได้ — วัดสองระยะแยกกัน ไม่บังคับเป็นเส้นตรง | ต้อง fit เส้นโค้ง ซับซ้อนขึ้น |
| ต้นทุน | numpy sum + argmin ทัน 10 Hz บน Pi สบาย | Hough แพงกว่ามาก |
| Debug | plot histogram เห็นทันทีว่าพลาดที่ไหน | ต้องดู blob + parameter หลายตัว |

`heading_err` จากผลต่างของ valley สองระยะ **ไม่ต้องรู้ geometry กล้องเลย**

`valley_prominence` เป็น confidence ที่มีความหมายทางกายภาพ: ถ้าแถบเขียวสองข้าง
แยกจากหุบไม่ชัด แปลว่าไม่เห็นร่อง ซึ่งควรทำให้ `valid = False` จริง ๆ
ไม่ใช่ค่าที่ตั้งเอาใจให้ผ่าน

### ⚠️ ผูกกับ `camera_front_tilt_deg`

มุมก้มกำหนดว่า `valley_far` อยู่ห่างไปข้างหน้าเท่าไร = **lookahead distance**
ขยับขายึดกล้องแล้ว gain ที่ tune ไว้ใช้ไม่ได้ ดู [../config/README.md](../config/README.md#controlyaml)

---

## `weed_detector.py` — เรขาคณิตของ rover ทำงานแทน row fitting

design gantry ต้อง fit เส้นแถวเพื่อแยก crop จาก weed เพราะมองทั้งแปลงจากด้านบน
พืชผลกับวัชพืชอยู่ในเฟรมเดียวกัน

rover กลับด้านปัญหานี้: **กล้องล่างเล็งที่ร่อง และพืชผลไม่ได้ปลูกในร่อง**

```text
สีเขียวในร่อง = วัชพืช  โดยนิยาม
```

ไม่ต้อง classifier ไม่ต้อง row fitting ไม่ต้องเทียบขนาดกับพืชผล:

```python
mask = exg_mask(down_frame)
for blob in blobs(mask):
    if blob.touches_side_edge:                     continue   # ใบพืชผลที่ล้ำเข้ามา
    if blob.centroid_x_mm not in furrow_corridor:  continue
    if blob.area_mm2 < min_weed_area_mm2:          continue
    yield Detection(...)
```

### `furrow_corridor` มาจากเรขาคณิต

```text
clear furrow   row_spacing − 2 × crop_foliage_half_width   350 − 60 = 290 mm
corridor       clear_furrow − 2 × max_lateral_error_mm     290 − 80 = 210 mm
```

`corridor` คือแถบที่ **รับประกันว่าอยู่ในร่อง** แม้ rover เบี่ยงเต็มพิสัย —
สีเขียวในแถบนี้จึงเป็นวัชพืชได้โดยไม่ต้องรู้ว่าแถวอยู่ไหน

กฎ `touches_side_edge` จัดการใบพืชผลที่ยื่นเข้าร่องโดยไม่ต้องรู้ว่ามันคือพืชผล —
ใบที่ต่อเนื่องออกไปนอกเฟรมย่อมไม่ใช่ต้นเล็กที่อยู่กลางร่อง

### สองค่าที่ประกอบกันเป็น corridor ต้องวัดที่ V1 ทั้งคู่

| ค่า | ตั้งไว้ | ถ้าวัดจริงแล้วมากกว่า |
|---|---|---|
| `max_lateral_error_mm` | 40 | corridor แคบลง recall ตก |
| `crop_foliage_half_width_mm` | 30 | clear furrow แคบลง **กระทบ startup invariant ข้อ 5 ด้วย** |

ถ้าค่าที่วัดได้ทำให้ `corridor <= 0` แปลว่า **แปลงแคบเกินไปสำหรับ rover ขนาดนี้**
ไม่ใช่ว่า detector ต้อง tune ใหม่

### ข้อจำกัด: วัชพืชในแถวมองไม่เห็น

in-row weed เป็นกรณี**ยากที่สุด**ของปัญหาจริง และอยู่นอก MVP โดยเจตนา —
เพราะการกำจัดมันต้องมี tool ซึ่งไม่มีใน MVP อยู่แล้ว

**ไม่ควรทำให้ perception ยากกว่าที่ actuator ทำได้** กลับมาที่ M3 พร้อม tool

---

## Calibration — เหลือแค่กล้องล่าง

| | กล้องหน้า | กล้องล่าง |
|---|---|---|
| Calibration | **ไม่มี** | homography → soil reference plane |
| Accuracy target | – | **±20 mm** (ผ่อนจาก ±8 mm ของ design gantry) |

ผ่อนได้เพราะ **MVP ไม่มีอะไรกระทำต่อตำแหน่ง** — ไม่มี tool ที่ต้องเล็ง
ตัวเลข mm ใช้แค่บันทึกขนาด/ตำแหน่งคร่าว ๆ ใน log

แต่ยัง **ต้องทำ** homography ตอนนี้ ไม่เลื่อนทั้งหมด — เพื่อรู้ตัวเลข error budget จริง
บนดิน ±15 mm **ก่อน** ออกแบบ tool ที่ M3 ไม่ใช่หลังจากนั้น

ขั้นตอน: [../docs/calibration.md](../docs/calibration.md)

---

## `weed_log.py` — และข้อจำกัดที่ต้องไม่ให้ใครเข้าใจผิด

```jsonl
{"t":12.34,"frame":123,"lateral_err":0.12,"u":160,"v":200,
 "x_mm":-25,"y_mm":120,"area_mm2":480}
```

`x_mm` / `y_mm` เป็นพิกัด **ใน rover frame** ไม่ใช่ใน bed frame

### ไม่มี weed id และไม่มีพิกัดในแปลง

ไม่มี odometry จึงไม่มีทางรู้ว่าวัชพืชที่เห็นในเฟรม 123 กับ 124 เป็นต้นเดียวกัน
หรือคนละต้น

**log นี้นับ "การตรวจจับ" ไม่ใช่ "จำนวนวัชพืช"** — ตัวเลขรวมจาก log ไม่มีความหมาย
**ห้ามใช้เป็น metric**

### วัดด้วย per-frame precision / recall

```text
per-frame precision >= 0.9
per-frame recall    >= 0.8   (วัดเฉพาะวัชพืชในร่อง)
```

เทียบ ground truth ที่ Isaac รู้อยู่แล้ว — ได้ label ฟรีและวัดได้ทุกเฟรม

recall ตั้งต่ำกว่า precision **โดยเจตนา**: MVP ไม่กำจัดอะไร การพลาดต้นหนึ่ง
ไม่เสียหาย แต่การรายงานดินเป็นวัชพืชจะทำให้ threshold ถูก tune ผิดทางตั้งแต่ต้น

ต้องวัดกับ randomization เปิดครบ ไม่ใช่ seed เดียว

---

## Detection Roadmap

| Stage | Method |
|---|---|
| **M1 (MVP)** | OpenCV — ExG + Otsu + corridor geometry |
| M3 | + in-row detection (ต้องแยก crop/weed จริง ๆ ตอนนั้น) |
| M5 | AI — YOLO / segmentation model |

อย่าเริ่มด้วย Deep Learning — MVP นี้แยก crop/weed ได้ด้วย**เรขาคณิต**
ซึ่งอธิบายได้ debug ได้ และไม่ต้องมี dataset

Dataset สำหรับ M5 generate ได้จาก `tools/dataset/` โดยใช้ ground truth จาก Isaac Sim
