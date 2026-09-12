# Calibration & Coordinate Transform

Module: `perception/calibration.py`

📐 นิยาม frame ทั้งหมด (`world` / `bed` / `rover` / `camera_front` / `camera_down`) และ
unit convention อยู่ที่ [../hardware/mechanical/coordinate-frames.md](../hardware/mechanical/coordinate-frames.md)
— อ่านก่อนแตะ code ในไฟล์นี้

---

## MVP calibrate กล้องเดียว

| | `camera_front` | `camera_down` |
|---|---|---|
| ใช้ทำ | row following | weed detection |
| Calibration | **ไม่มี** | intrinsic + homography |
| Accuracy target | – | **±20 mm** |

นี่คือความต่างที่ใหญ่ที่สุดจาก design gantry ซึ่งต้อง calibrate ให้แม่นกว่า
รัศมีหัว tool (±8 mm) เพราะมีอะไรต้องเล็ง

---

## ทำไมกล้องหน้าไม่ต้อง calibrate

`RowEstimate` เป็นค่า **image-space ล้วน**:

```python
lateral_err = (valley_near - u_center) / (width / 2)      # [-1, 1]
heading_err = (valley_far  - valley_near) / (width / 2)   # [-1, 1]
```

`heading_err` คำนวณจาก **ผลต่างของ valley สองระยะในภาพเดียวกัน** ซึ่งไม่ต้องรู้
focal length, ความสูงกล้อง หรือมุมก้มเลย

row follower ปิดลูปด้วยค่านี้โดยตรง — ไม่มีขั้นตอนไหนต้องแปลงเป็น mm

**ราคาที่จ่าย**: gain มีหน่วย `(deg/s) / unitless` จึงต้อง tune ไม่มีสูตรคำนวณจาก
เรขาคณิต และ gain **ผูกกับ `camera_front_tilt_deg`** เพราะมุมก้มกำหนด lookahead
ขยับขายึดกล้องแล้ว gain ใช้ไม่ได้

แลกกันแล้วคุ้ม: MVP ตัดงาน calibration ที่แม่นยำออกได้หนึ่งกล้อง และตัด
coordinate transform ออกจาก control path ทั้งหมด

---

## ทำไมกล้องล่างยังต้อง calibrate

MVP **ไม่มีอะไรกระทำต่อตำแหน่งวัชพืช** — ไม่มี tool ที่ต้องเล็ง
ถ้าจะเลื่อนงาน calibration ไป M3 ก็ดูมีเหตุผล

แต่ **ต้องทำตอนนี้** เพราะ:

```text
ต้องรู้ตัวเลข error budget จริงบนดิน ±15 mm ก่อนออกแบบ tool
ไม่ใช่หลังจากสร้าง tool แล้วพบว่าเล็งไม่เข้า
```

M3 ต้องเลือกรัศมีหัว tool จากตัวเลขนี้ — ถ้าไม่มีตัวเลข จะเลือกจากการเดา

---

## Pipeline (กล้องล่าง)

```text
camera_down
  │
  ▼
Image
  │
  ▼
ExG + Otsu + floor
  │
  ▼
Blobs
  │
  ▼
furrow_corridor filter + touches_side_edge filter
  │
  ▼
Pixel Coordinate (u, v)
  │
  ▼
Homography (soil reference plane)
  │
  ▼
Rover-frame Coordinate (x_mm, y_mm)
  │
  ▼
weed_log.jsonl
```

**สังเกตว่าไม่มี Motion Planner ที่ปลายทาง** — ปลายทางคือ log
นี่คือ observation path ไม่ใช่ control path

---

## Example

```text
u = 160, v = 200   →   x_mm = −25, y_mm = 120   (rover frame)
```

`x_mm = −25` แปลว่าอยู่ซ้ายของกึ่งกลาง rover 25 mm

**ไม่มีพิกัดในแปลง** — ไม่มี odometry จึงแปลง `rover → bed` ไม่ได้

---

## Calibration Steps

1. วาง calibration target (checkerboard) **บนแผ่นแข็งวางราบบนดิน**
2. Capture จาก `camera_down` ที่ตำแหน่งติดตั้งจริงบน rover
3. คำนวณ intrinsic (focal length, principal point, distortion)
4. คำนวณ homography ระหว่าง image plane และ **soil reference plane**
5. Verify ด้วยจุดที่รู้ตำแหน่งจริง — วัด error เป็น mm ทั้งจุดดินสูงและจุดดินต่ำ
6. บันทึกผลและ error budget ลง `config/camera.yaml`

ขั้นที่ 1 ต้องใช้แผ่นแข็ง เพราะวาง target ลงบนดินตรง ๆ จะเอียงตามความไม่เรียบ
ทำให้ได้ homography ที่ผิดตั้งแต่ต้น

ขั้นที่ 2 ต้องทำ **บน rover ที่ประกอบเสร็จ** ไม่ใช่ถือกล้องวางไว้ —
ความสูงและมุมของขายึดจริงคือสิ่งที่ homography อธิบาย

---

## Soil Plane Error Budget

ดินไม่เรียบ ±`soil_variation_mm` (15 mm) แต่ homography สมมติว่าทุกจุดอยู่บน
plane เดียว → จุดที่สูงหรือต่ำกว่า reference plane จะถูก project ผิดตำแหน่ง

```text
     กล้อง
       │ ╲
       │  ╲  แนวสายตา
       │   ╲
───────┼────╳──────  reference plane  → ตำแหน่งที่คำนวณได้
       │   ╱ ╲
       │  ╱   ╲
       │ ╱     ● จุดจริงบนยอดดิน
              └─ error
```

ขนาด error ขึ้นกับมุมมองกล้อง:

| การติดตั้ง | Error จากความไม่เรียบ |
|---|---|
| `camera_down` top-down | เล็กสุด แต่ไม่เป็นศูนย์ที่ขอบเฟรม |
| เอียงทำมุม | ใหญ่ขึ้นตามมุม |

`camera_down` ตั้งให้มองตรงลง **โดยเจตนา** เหตุผลเดียวกับ `camera_tilt_deg = 0`
ของ design gantry — ยิ่งเอียงยิ่งแย่

**ต้องวัดจริง ไม่ใช่ประมาณ** — เอา marker ที่รู้ตำแหน่งไปวางทั้งจุดสูงและจุดต่ำ
แล้ววัดว่าค่าที่คำนวณได้เพี้ยนไปกี่ mm

### Error ที่ยอมรับได้ใน MVP: ±20 mm

ผ่อนจาก ±8 mm ของ design gantry เพราะ MVP ไม่มี tool ที่ต้องเล็ง
ตัวเลข mm ใช้แค่บันทึกขนาด/ตำแหน่งคร่าว ๆ ใน `weed_log`

ถ้าวัดจริงแล้วเกิน 20 mm:

1. ยืนยันว่ากล้องตั้งตรงลงจริง (±1°)
2. ปรับหน้าดินให้เรียบขึ้น → ลด `soil_variation_mm`
3. ลดความสูงกล้อง (แลกกับ FOV ที่แคบลง — ต้องยังครอบ corridor 210 mm)

**ห้ามแก้ `error_budget_mm` ในไฟล์ config เพื่อให้ "ผ่าน"** — ค่านั้นคือ target
ที่ M3 จะใช้ตัดสินขนาดหัว tool ค่าที่วัดได้ต้องบันทึกแยกเป็นค่าที่วัดได้

---

## Verification

Round-trip test ใน `tests/unit/`:

```text
rover-frame (x, y) → expected pixel → transform back → rover-frame (x, y)
```

Scenario test บนดินไม่เรียบ:

```text
tests/scenarios/soil_rough.yaml
tests/scenarios/weed_in_furrow.yaml
```

ยืนยันว่า error ที่เกิดจริงยังอยู่ใน budget และ `weed_detector` ยังได้
per-frame precision/recall ตามเกณฑ์

---

## สิ่งที่ MVP ไม่มี: `bed` frame

```text
world → bed  ┄┄ ไม่มี ┄┄  rover ──┬─► camera_front
                                   └─► camera_down
```

`bed → rover` **ไม่มีใครรู้ค่า runtime** เพราะไม่มี localization

ผลคือ calibration ของ MVP หา `camera_down → rover` เท่านั้น — **ไม่ใช่**
`camera → bed` เหมือน design gantry

ใครเขียน code ที่ต้องการพิกัดใน `bed` frame ต้องหยุดแล้วไปทำ **M2 (encoder)**
ไม่ใช่เดาค่ามาเติม ดู [../hardware/mechanical/coordinate-frames.md](../hardware/mechanical/coordinate-frames.md)

---

## Sim vs Real

Isaac Sim camera ให้ ground-truth intrinsic, pose และ **ความสูงจริงของทุกจุดบนดิน** —
ใช้วัด projection error ได้แม่นยำโดยไม่ต้องวัดมือ เป็นวิธีที่ถูกที่สุดในการหา
error budget ก่อนมี hardware

เมื่อเปลี่ยนไปใช้กล้องจริง ให้ calibrate ใหม่แล้วเปลี่ยนแค่ `config/camera.yaml`
โดยไม่แก้ code

⚠️ **gain ของ row follower ไม่โอนจาก sim ไป hardware** แม้ calibration จะโอนได้ —
slip ของ skid-steer ต่างกัน ดู [../config/README.md](../config/README.md#controlyaml)

---

## Tools

`tools/calibration/` — script ช่วย capture, solve, visualize
และ **plot error map ทั่วเฟรม** เพื่อหาโซนที่ error เกิน budget
