# V1 — Isaac Simulation

แผนงานจาก V0 ที่ปิดแล้ว ไปถึง **rover วิ่งตามร่องใน Isaac Sim ได้จริง**

เกณฑ์จบอยู่ที่ [README.md#mvp-acceptance-criteria](README.md#mvp-acceptance-criteria) หมวด V1
รายละเอียด scene / physics อยู่ที่ [docs/simulation.md](docs/simulation.md) และ [sim/isaac/README.md](sim/isaac/README.md)

---

## สถานะตอนเขียน

V0 ปิดครบ — 572 tests, `pytest -q` ~1.3 s, ไม่ต้องมี GPU · `pio test -e native` ผ่าน 4 เคส

| มีแล้ว ใช้ได้ทันที | อยู่ที่ |
|---|---|
| state machine + transition ครบตาราง §5.3 | `controller/workflow/` |
| safety 3 detector + fault router | `controller/safety/` |
| skid-steer mixing ผ่าน golden vectors ทั้ง Python และ C++ | `controller/rover/mixing.py` |
| startup invariants 9 ข้อ | `controller/startup_checks.py` |
| protocol codec + seq tracker + ESP32 emulator | `bridge/` |
| ExG + Otsu + absolute floor | `perception/exg.py` |
| scenario loader / runner + vocabulary | `tests/harness/scenario.py` |
| `RowEstimate` | `controller/motion/row_estimate.py` |

| ยังว่าง | สภาพ |
|---|---|
| `sim/isaac/` | **README.md ไฟล์เดียว** — 6 โฟลเดอร์ย่อยว่างหมด ไม่มี `app.py` / `world.py` |
| `perception/` | มีแค่ `exg.py` — ขาด `cameras.py` · `row_estimator.py` · `weed_detector.py` · `weed_log.py` · `calibration.py` |
| `cad/urdf/` | ไม่มี `weeding_rover.urdf` · `cad/exports/{meshes,step,stl}` ว่างทั้งหมด |
| `controller/rover/isaac_rover.py` | skeleton — ทุก method `raise NotImplementedError` |
| `scripts/` | README เอกสาร 6 script ไม่มีสักตัว |
| `controller/main.py` | ไม่มี |
| `tests/simulation/` | `.gitkeep` |

`config/development.yaml` ตั้ง `backend: isaac` ไว้แล้ว และ invariant 8 ผ่าน (`gains.isaac` ไม่ใช่ null) —
แปลว่าตอนนี้ startup ผ่านแล้วไปตายที่ `drive()` ไม่ใช่ตายที่ config

---

## สามเรื่องที่ต้องเคลียร์ก่อนเขียนโค้ดบรรทัดแรก

### 1. Isaac Sim เวอร์ชันไหน และ controller รันใต้ Python ตัวไหน

[sim/isaac/README.md#boundary](sim/isaac/README.md) กำหนดว่า `adapters/` เป็นจุดเดียวที่
`controller/rover/isaac_rover.py` คุยด้วย — เป็นการเรียกในโปรเซสเดียวกัน แปลว่า
**controller ต้องรันใต้ Python ที่มากับ Isaac Sim**

`pyproject.toml` เขียน `requires-python = ">=3.11"` ถ้าเวอร์ชันที่ลงให้ Python คนละตัว
ต้องเลือกทางใดทางหนึ่ง:

| ทางเลือก | ผลที่ตามมา |
|---|---|
| ขยับ `requires-python` ให้ตรงกับ Isaac | ง่ายที่สุด แต่ต้องเช็คว่า syntax ที่ V0 ใช้อยู่ยังผ่าน |
| แยกโปรเซส controller ↔ sim | ขัดกฎ "`adapters/` เป็นจุดเดียวที่คุยด้วย" และเพิ่ม transport ที่ V0 ไม่ได้ออกแบบไว้ |

**ข้อนี้เปลี่ยนรูปงานทั้งหมด ตอบก่อน**

### 2. asset พืชผล / วัชพืช มาจากไหน

`sim/isaac/scenes/crops/` และ `scenes/weeds/` ว่างทั้งคู่

V1 acceptance บังคับว่าต้อง **วัด** `crop_foliage_half_width_mm` จาก asset จริงแล้วบันทึกลง
`config/rover.yaml` — ค่า `30` ที่ใช้อยู่เป็นการประมาณ และมันไหลต่อไปที่
`clear_furrow = row_spacing − 2 × 30 = 290` ซึ่งเป็นฐานของ invariant ข้อ 5

ถ้าวัดแล้วได้มากกว่า 30 → `clear_furrow` แคบลง → **invariant ข้อ 5 อาจ fail**
นั่นคือพฤติกรรมที่ถูกต้อง ไม่ใช่ bug

### 3. mesh ของ rover — primitive ก่อน หรือรอ Fusion

`cad/urdf/weeding_rover.urdf` ไม่มี และ `cad/exports/` ว่างทั้งสามโฟลเดอร์

`cad/parameters/parameters.csv` มีขนาดครบทุกตัวแล้ว (มี test บังคับ sync กับ config อยู่) —
เขียน URDF ด้วย `<box>` + `<cylinder>` จากตัวเลขชุดนี้ได้เลยโดยไม่ต้องรอ mesh
collision ใช้ primitive อยู่แล้วดีกว่า mesh ในแง่ physics stability

**แนะนำ: เริ่มด้วย primitive** แล้วค่อยเปลี่ยน visual mesh ทีหลัง — physics tuning
(Stage 3) ไม่ควรต้องรอ CAD

---

## ลำดับงาน

```text
Stage 0  ติดตั้ง + พิสูจน์ว่าเปิดได้          ◄── blocker
Stage 1  perception ที่ไม่ต้องใช้ Isaac        ◄── ขนานได้ทันที ไม่ต้องรอ Stage 0
Stage 2  เอา rover เข้า Isaac ให้ขยับได้      (thin slice — พื้นเรียบ ไม่มีพืช)
Stage 3  physics tuning                       ◄── งานหนักสุดของ V1
Stage 4  แปลงดิน + พืช
Stage 5  กล้อง + virtual I/O
Stage 6  ปิดลูป — main.py + scripts
Stage 7  scenario ใน Isaac
Stage 8  tune + วัดค่าจริง → ปิด V1
```

---

### Stage 0 — ติดตั้งและพิสูจน์ว่าเปิดได้

ไม่มีโค้ด งานคือได้คำตอบของคำถามข้อ 1

```text
[ ] ติดตั้ง Isaac Sim + ยืนยัน GPU
[ ] เปิด empty stage ได้
[ ] รู้เวอร์ชัน Python ที่ Isaac ใช้ และตัดสินใจเรื่อง requires-python
[ ] import numpy / yaml จาก Python ตัวนั้นได้
```

---

### Stage 1 — perception ที่ไม่ต้องใช้ Isaac

**เริ่มได้เดี๋ยวนี้ ไม่ต้องรอ Stage 0** และนี่คือก้อนที่บล็อก `main.py`

```text
perception/cameras.py         CameraSet protocol + FakeCameraSet
perception/row_estimator.py   valley histogram → RowEstimate
tests/unit/                   ภาพสังเคราะห์ ไม่ต้องมี GPU
```

สูตรและเหตุผลทั้งหมดอยู่ที่ [perception/README.md](perception/README.md#row_estimatorpy--valley-histogram-ไม่ใช่-line-fitting) แล้ว:

```python
valley_near = find_valley(mask[bottom_third])    # ใกล้ล้อ
valley_far  = find_valley(mask[middle_third])    # lookahead

lateral_err = (valley_near - u_center) / (width / 2)
heading_err = (valley_far - valley_near) / (width / 2)
confidence  = valley_prominence
valid       = confidence > conf_min   (ทั้งสองแถบ)
```

ของที่มีให้ใช้แล้ว: `exg.green_mask(rgb, exg_floor)` · `RowEstimate` (มี `__post_init__`
บังคับช่วงค่าอยู่แล้ว) · `perception.row_estimator.conf_min` และ `green_fraction_row_end` ใน config

**เกณฑ์จบ**

```text
[ ] ภาพสังเคราะห์ร่องตรง        → lateral_err ≈ 0
[ ] ร่องเบี่ยงซ้าย/ขวา           → เครื่องหมายถูกต้อง (+ = ร่องอยู่ขวา)
[ ] ร่องเอียง                    → heading_err มีเครื่องหมายถูกต้อง
[ ] ภาพดินเปล่า                  → valid = False, green_fraction ต่ำ
[ ] ต้นหายกลางแถว                → ยัง valid (นี่คือจุดที่ valley ชนะ line fitting)
[ ] ไม่มี pose ใน perception/    (test_layering ยังไม่คุม dir นี้ — พิจารณาเพิ่ม)
```

---

### Stage 2 — เอา rover เข้า Isaac ให้ขยับได้ (thin slice)

พื้นเรียบ ไม่มี heightfield ไม่มีพืช ไม่มีกล้อง — เป้าหมายเดียวคือ**ล้อหมุนตามคำสั่ง**

```text
cad/urdf/weeding_rover.urdf        links / joints / limits จาก parameters.csv
scripts/import_urdf.sh             → robots/rover/rover_base.usd  (gitignored)
robots/rover/rover.usd             override layer: drive gain, damping, friction
robots/rover/wheels.py             joint handles fl · fr · rl · rr
robots/rover/config.yaml           joint drive gains, friction
sim/isaac/world.py                 stage setup
sim/isaac/app.py                   entry point
adapters/wheel_adapter.py          (v, omega) → mixing → wheel joint velocity
controller/rover/isaac_rover.py    ของจริง แทน skeleton
```

`wheel_adapter.py` **ต้องเรียก `controller.rover.mixing`** ไม่ใช่เขียนสูตรใหม่ —
สูตรนี้ถูก implement ซ้ำที่ firmware แล้วหนึ่งที่ และทั้งคู่ผูกกับ
`config/drive_mixing_vectors.csv` ตารางเดียวกัน ที่นี่แปลงหน่วยอย่างเดียว:

```text
mm → m        deg/s → rad/s        v_side → wheel_rad_s
```

ล้อหน้าและหลังข้างเดียวกันได้ค่าเดียวกันเสมอ (ส่งค่าเดียวไป 2 joint) ตรงกับของจริงที่ต่อมอเตอร์ขนานเป็นข้าง

**เกณฑ์จบ**

```text
[ ] drive(100, 0)   → วิ่งตรง ความเร็วใกล้ 100 mm/s
[ ] drive(0, 40)    → หมุนอยู่กับที่
[ ] drive(320, 40)  → saturation scale ทั้งสองข้าง ไม่ clip ข้างเดียว
[ ] stop()          → หยุดโดยยังเบรกค้าง
[ ] get_drive_state() key set ตรงกับ fake / esp32   (test เดิมจับอยู่แล้ว)
[ ] IsaacRover ไม่มี pose รั่วออกมา
```

---

### Stage 3 — physics tuning

[sim/isaac/README.md](sim/isaac/README.md#ความเสี่ยง-physics-ที่-gantry-ไม่มี) เขียนไว้ตรง ๆ ว่า
**นี่คืองานที่ใช้เวลามากที่สุดของ V1 มากกว่า row follower**

```text
1. friction     พอให้ขับไปข้างหน้าได้ แต่ต้อง slip ได้ตอนเลี้ยว skid-steer
                สูงเกิน → เลี้ยวไม่ออก หรือ solver ระเบิด
2. ล้อลอย       บน heightfield ±15 mm แรงขับหายเป็นช่วง — ไม่ใช่ bug ของ sim
3. timestep     contact ที่ความเร็วต่ำบนผิวขรุขระอาจต้อง substep ถี่ขึ้น
                กระทบเวลารัน CI
```

ค่าที่ tune ได้ทั้งหมดอยู่ใน `robots/rover/rover.usd` (override layer) **ไม่ใช่ใน URDF** —
เพราะ `rover_base.usd` ถูกเขียนทับทุกครั้งที่ regenerate

**ถ้า Stage 2 ยังขยับไม่สวย อย่าไปต่อ Stage 4**

---

### Stage 4 — แปลงดิน + พืช

```text
environments/soil_heightfield.py    ±15 mm · resolution 10 mm · generate จาก seed
environments/crop_rows.py           3 แถว + jitter + row angle + curvature + gap
environments/random_weeds.py        สุ่มในร่อง / ในแถว
scenes/weeding_bed.usd              scene หลัก 2000 × 1000 mm
scenes/bed_frame.usd                ขอบแปลง ยกสูงกว่า wheel_diameter/2
```

`bed_frame.usd` **ไม่ใช่ของประดับ** — MVP ไม่มี bumper switch ขอบยกเป็น compensating
control ทางกายภาพ ต้องมีทั้งใน sim และของจริง

สอง parameter ที่ห้ามเป็น 0 ในการทดสอบปกติ:

| Parameter | ถ้าเป็น 0 |
|---|---|
| `row_curvature_mm` | ร่องตรงเป๊ะ → `RowFollower` ผ่านได้ด้วย gain = 0 จะไม่รู้ว่ามัน track ได้จริงไหมจนลงแปลง |
| `crop_gap_probability` | ไม่มีต้นหาย → heuristic แยก `row_end` / `row_lost` ไม่ถูกทดสอบที่จุดที่มันพังจริง |

`soil.variation_mm` ใน `simulation.yaml` ต้องตรงกับ `bed.soil_variation_mm` — **invariant ข้อ 9**
บังคับอยู่แล้ว ไม่ใช่แค่คำแนะนำ

**เกณฑ์จบ**

```text
[ ] วัด crop_foliage_half_width_mm จาก asset จริง → บันทึกลง config/rover.yaml
[ ] re-validate invariant ข้อ 5 ด้วยค่าที่วัดได้
[ ] ถ้า invariant fail → แปลงแคบเกินไปสำหรับ rover ขนาดนี้ ไม่ใช่ตัวเลขที่ต้องแก้ให้ผ่าน
```

---

### Stage 5 — กล้อง + virtual I/O

```text
sensors/camera_front.py       640×480  tilt ~45°   ผูกกับ base_link
sensors/camera_down.py        1280×720 top-down
sensors/sensor_manager.py
sensors/estop.py              virtual digital input
adapters/io_adapter.py        virtual digital I/O + command timeout
perception/cameras.py         + IsaacCameraSet
```

**`io_adapter.py` ต้องจำลอง command timeout ด้วย — ห้ามข้าม**

`IsaacRover` ต้องดับล้อเมื่อไม่ได้ `drive` ใหม่เกิน `command_timeout_ms` เหมือน firmware จริง
ไม่งั้น `link_lost.yaml` จะผ่านใน sim แต่พฤติกรรมไม่ตรงกับของจริง — และ bug ประเภทนี้
จะโผล่ที่ V3 ซึ่ง debug แพงกว่ามาก

`bridge/simulator.py` ทำเรื่องนี้ไว้แล้วฝั่ง ESP32 อ่านเป็นแบบได้

---

### Stage 6 — ปิดลูป

```text
controller/main.py        startup_checks → เลือก backend → CameraSet → RowRun + detectors
scripts/run_sim.sh        เปิด Isaac Sim + โหลด scene
scripts/run_controller.sh รัน controller ตาม backend ใน development.yaml
scripts/run_all.sh        sim + bridge + controller พร้อมกัน (SIL)
scripts/run_fake_loop.sh  closed loop ด้วย FakeRover — ไม่ต้องมี Isaac
```

`main.py` เขียนได้แล้วตรงนี้ เพราะ `perception/row_estimator.py` มีจริงตั้งแต่ Stage 1 —
เหตุผลที่รอบก่อนไม่เขียนหมดไปแล้ว

ลูปที่ต้องประกอบ ([controller/README.md](controller/README.md)):

```python
while state is DRIVING_ROW:
    est = row_estimator.estimate(cameras.front())    # 10 Hz
    if not est.valid:
        watchdog.tick(est)
        continue
    watchdog.reset()
    rover.drive(*follower.step(est))
```

บวก detector ที่มีอยู่แล้ว: `EmergencyStop.tick()` (นอก guard — ปุ่มถูกกฎจากทุก state),
`FaultManager.handle()`, `LinkMonitor.tick()` ลำดับการเรียกดูได้จาก `tests/harness/bench.py`
ซึ่งเป็นลูปที่ใกล้เคียงที่สุดที่มีอยู่

---

### Stage 7 — scenario ใน Isaac

runner มีโครงอยู่แล้ว เหลือขยาย 2 อย่าง:

```text
1. vocabulary ใน tests/harness/scenario.py
     distance_travelled_mm · min_clearance_mm · max_lateral_error_mm · stopped_before_mm
2. backend ของ runner — Isaac แทน Bench
```

แล้วเขียน 7 ไฟล์ที่เหลือ (`weed_in_furrow.yaml` เป็น V2 ไม่ใช่รอบนี้):

```text
row_straight.yaml · row_curved.yaml · row_tilted_start.yaml
crop_gap_midrow.yaml · row_end.yaml · row_lost.yaml · soil_rough.yaml
```

`crop_gap_midrow` กับ `row_end` เป็นคู่ตรงข้าม **ต้องผ่านทั้งคู่** — ผ่านข้างเดียว
แปลว่า threshold ตั้งเอาใจข้างเดียว และจะพังในแปลงจริงที่มีทั้งสองอย่าง

⚠️ ตัวเลขใน `tests/README.md` เขียนไว้สำหรับ V1 perception ไม่ใช่สำหรับ harness
(`FakeRowSensor` จบร่องเร็วไป lookahead หนึ่งช่วงโดยโครงสร้าง) — ตอนเขียนไฟล์จริง
ให้วัดกับ estimator จริงแล้วตัดสินใจใหม่ **ห้ามแก้ตัวเลขในไฟล์ scenario เพื่อให้ผ่าน**

---

### Stage 8 — tune + วัดค่าจริง

```text
[ ] tune row_follower.gains.isaac ใน config/control.yaml
[ ] วัด max_lateral_error_mm จริง → บันทึกลง config/perception.yaml
[ ] ยืนยัน front loop >= 10 Hz
[ ] ตรวจ V1 acceptance ครบทุกข้อ
```

---

## V1 Acceptance Criteria

จาก [README.md](README.md#mvp-acceptance-criteria) — ทุกข้อวัดได้ ไม่มีข้อไหนเขียนว่า "ทำงานได้"

```text
[ ] rover วิ่งร่องตรง 2000 mm จบ ไม่เบียดต้นพืช
[ ] ร่องโค้ง 40 mm วิ่งจบ
[ ] เริ่มเอียง 15° เข้าร่องได้
[ ] ผิวดิน ±15 mm ยังตามร่องได้ (ล้อลอยเป็นช่วง)
[ ] ช่องว่างกลางแถว → ไม่หยุด
[ ] สุดร่อง → STOPPED(row_end_suspected)
[ ] ถอดแถวแต่ยังมีเขียว → ERROR(row_lost)
[ ] front loop >= 10 Hz
[ ] วัด max_lateral_error จริง บันทึกลง perception.yaml
[ ] วัด crop_foliage_half_width จริงจาก asset พืช บันทึกลง rover.yaml
[ ] re-validate invariant ข้อ 5 ด้วยค่าที่วัดได้ทั้งสองตัว
```

---

## นอกขอบเขต V1

| ของ | ไปอยู่ที่ |
|---|---|
| `weed_detector.py` · `weed_log.py` · evaluation harness | V2 |
| `weed_in_furrow.yaml` | V2 |
| `calibration.py` + homography กล้องล่าง | V2 |
| `domain_randomization.py` | V2 (randomization พื้นฐานอยู่ใน `crop_rows.py` แล้ว) |
| `tools/dataset/` | M5 |
| `bridge/serial_link.py` · `bridge/main.py` · firmware จริง | V3 |
| เลี้ยวเข้าร่องถัดไป | M4 |

---

## ความเสี่ยง

**1. Stage 3 กินเวลามากกว่าที่คิด**
docs เขียนไว้ตรง ๆ ว่า physics ของล้อบน heightfield หนักกว่า row follower
วางแผนเวลาตามนี้ ไม่ใช่ตามสัญชาตญาณที่ว่า controller คืองานหลัก

**2. `camera_front_tilt_deg` ผูกกับ gain และไม่มี test จับ**
มุมก้มกำหนด lookahead distance ซึ่งคือสิ่งที่ gain ถูก tune กับมัน
ขยับขายึดกล้องเมื่อไหร่ gain ที่ tune ไว้ใช้ไม่ได้ — **ตัดสินใจมุมกล้องให้จบก่อน Stage 8**

**3. ถ้า V1 แสดงว่าตามร่องไม่ได้ ให้กลับไปทบทวนล้อ/ช่วงล่าง ไม่ใช่ทบทวน gain**
spec §11 เขียนไว้แบบนั้น — gain ที่ tune จนผ่านบนล้อที่ผิดคือการเลื่อนปัญหาไปที่ V4

**4. gain ไม่โอนจาก sim ไป hardware**
slip ของ skid-steer ต่างกัน `control.yaml` จึงเก็บ gain แยกตาม backend
`gains.esp32` เป็น `null` โดยเจตนา และ invariant ข้อ 8 จะ fail ถ้ามีใครพยายามรันของจริงด้วยค่า sim

---

## เส้นทางที่แนะนำ

**Stage 1 ขนานไปกับ Stage 0 + 2**

Stage 1 ทำได้ทันทีโดยไม่ต้องติดตั้งอะไรเลย เป็น Python ล้วน ทดสอบด้วยภาพสังเคราะห์
สไตล์เดียวกับ V0 และถ้าติดปัญหาเรื่องเวอร์ชัน Python ที่ Stage 0 งานฝั่ง perception
ก็ยังเดินหน้าต่อได้โดยไม่เสียเวลา
