# Selected Parts — ของที่เลือกแล้ว พร้อมที่มาของ CAD

ส่งต่อให้ session ที่ต่อ Fusion MCP เข้าไปประกอบ

BOM เต็มและเหตุผลของแต่ละหมวด: [poc-v2.md](poc-v2.md)
ข้อจำกัดเชิงขนาดทั้งหมด: [../../cad/parameters/README.md](../../cad/parameters/README.md)

> ⚠️ **เอกสารนี้เลือกชิ้นส่วนสำหรับเครื่องล้อ Ø70 มม. (145 mm) — ไม่ใช่ Rover Base V0**
> Rover Base V0 (650 × 520 mm, ล้อ Ø250, ~35 kg) มีข้อกำหนดที่ปัจจุบันคือ
> [`poc-v3.md`](poc-v3.md) ซึ่งยังเป็น **requirement envelope ไม่มี part number**
> (design decision S10) ไม่ใช่การเลือกของแบบไฟล์นี้
>
> พารามิเตอร์ที่ตารางด้านล่างอ้างอิง (เช่น `motor_mount_pitch_mm`) **ถูกลบออกจาก
> `cad/parameters/parameters.csv` แล้ว** แทนที่ด้วย `motor_mount_bolt_circle_mm: 60`
> — ของที่เลือกไว้ในตารางนี้ยึดด้วยรูคู่ ไม่ใช่ bolt circle จึงใช้กับโครงปัจจุบันไม่ได้
>
> เก็บไว้เป็นประวัติของเครื่อง 145 mm ไม่ใช่ของที่ต้องสั่งซื้อสำหรับเครื่องนี้

---

## เลือกแล้ว

| # | ของ | รุ่น / เลขสินค้า | จำนวน | CAD |
|---|---|---|---|---|
| D1 | มอเตอร์เกียร์ | Pololu **250:1 Metal Gearmotor 20Dx46L mm 12V CB** [#3482](https://www.pololu.com/product/3482) | 4 | **STEP** — `20d-metal-gearmotors-3d-models.zip` ที่แท็บ Resources |
| D2 | ล้อ | Pololu **Scooter/Skate Wheel 70×25 mm** Black [#3272](https://www.pololu.com/product/3272) | 4 | มีที่แท็บ Resources |
| D4 | ดุมล้อ | Pololu **Aluminum Scooter Wheel Adapter for 4 mm Shaft** [#2672](https://www.pololu.com/product/2672) | 4 | มีที่แท็บ Resources |
| D5 | ขายึดมอเตอร์ | Pololu **20D mm Metal Gearmotor Bracket Pair** [#1138](https://www.pololu.com/product/1138) | 2 คู่ | มีที่แท็บ Resources |
| C1 | คอมพิวเตอร์ | **Raspberry Pi 5 (4 GB)** | 1 | [STEP ทางการ](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-step.zip) |
| S1 | กล้องหน้า | **Raspberry Pi Camera Module 3** (ตัวมาตรฐาน) | 1 | [กลไก](https://datasheets.raspberrypi.com/) |
| S2 | กล้องล่าง | **Raspberry Pi Camera Module 3** (ตัวมาตรฐาน — **ไม่ใช่ Wide**) | 1 | เหมือน S1 |

ยังไม่ได้เลือก: driver (BTS7960/IBT-2 ตามสเปกเดิม) · แบต 3S · buck ×2 · E-stop latching NC · แชสซี

⚠️ **ยังไม่ได้เช็คราคาจริงสักตัว** ของ Pololu เป็นของนำเข้า ต้องบวกค่าส่ง
และน่าจะสูงกว่าช่วงราคาที่ร่างไว้ใน BOM เดิมหลายเท่า

---

## ทำไมต้องเป็นตัวนี้ — ข้อจำกัดที่บังคับ ไม่ใช่รสนิยม

### D1 — `track_width 120` จำกัดความยาวมอเตอร์ที่ 47 mm

หน้าในของล้ออยู่ที่ ±47.5 mm จากแกนกลาง มอเตอร์ซ้าย/ขวายื่นเข้าหากัน

```text
N20                     ~30 mm   ผ่าน แต่แรงบิดไม่พอไต่ก้อนดิน 15 mm
Pololu 20D            41–46 mm   ผ่าน            ◄── ที่เลือก (46 mm)
Pololu 25D            48–60 mm   ชน
GA25-370 / JGA25-370    ~65 mm   ชน
JGB37-520 (37D)         ~72 mm   ชน
```

มอเตอร์ที่หาง่ายที่สุดในไทยใช้ไม่ได้ทั้งหมด ถ้าจะใช้ต้องขยาย `track_width`
ซึ่งกิน margin ของ invariant ข้อ 5

### D1 — เพดานรอบมาจาก invariant

```text
ล้อข้างในตอนเลี้ยวเต็มพิสัย = 58.1 mm/s   ต้อง >= 15% ของ wheel_v_max
                            →  wheel_v_max <= 387 mm/s  →  ล้อ 70 mm คือ <= 106 RPM

20D 125:1   110 RPM   403 mm/s   ล้อข้างใน 14.4% duty   ✗ ใต้ deadband
20D 250:1    55 RPM   202 mm/s   ล้อข้างใน 28.8% duty   ✓
```

125:1 ดูตรงสเปก "~100 RPM" ที่สุด แต่เร็วไป 4% แล้วตก invariant ทันที

ที่ 202 mm/s = **2.0×** ของ `v_max` ซึ่งอยู่ขอบล่างพอดีของ 2–3× ที่ควรเป็น
→ **ห้ามเพิ่ม `v_max` โดยไม่เปลี่ยนมอเตอร์**

### D2 — 70 mm ดีกว่า 65 ที่ร่างไว้

`wheel_diameter >= 4 × soil_variation` เป็น invariant ที่บางที่สุดในระบบ
ล้อสำเร็จรูปที่หาได้จริงเป็น 70 mm ทำให้ margin ขยับจาก 5 เป็น **10 mm**

### S2 — Camera Module 3 ตัวมาตรฐาน ไม่ใช่ Wide

```text
HFOV 66° ที่สูง 220 mm  →  ครอบ 286 mm   (ต้องการ 250)         ✓ margin 36 mm
VFOV 40° ที่สูง 220 mm  →  161 mm/เฟรม   (ต้องการ > 50)        ✓
```

Wide (102°) ครอบ 543 mm — เกินความจำเป็นและได้ distortion เปล่า ๆ

⚠️ IMX708 **crop FOV เมื่อใช้โหมดความละเอียดต่ำ** — ต้องจับภาพที่โหมดเต็มเซนเซอร์
แล้วย่อลงมา ไม่ใช่สั่ง 1280×720 ตรง ๆ ไม่งั้น FOV จริงจะแคบกว่าที่คำนวณไว้

### C1 — Pi 4 มี CSI ช่องเดียว

Pi 5 มี CAM0 + CAM1 ใช้พร้อมกันจริง multiplexer บน Pi 4 สลับทีละตัว
ซึ่งทำให้ตก V1 acceptance ข้อ `front loop >= 10 Hz` โดยตรง

---

## ค่าที่เปลี่ยนแล้วใน CAD + config

แก้ครบทั้ง `cad/parameters/parameters.csv` และ `config/` แล้ว · `pytest` 572 ผ่าน · `pio test -e native` ผ่าน

| ค่า | เดิม | ใหม่ | ที่มา |
|---|---|---|---|
| `wheel_diameter_mm` | 65 | **70** | ล้อ Pololu #3272 |
| `wheel_width_mm` | 26 | **25** | ล้อ Pololu #3272 |
| `body_width_mm` | 146 | **145** | `max(140, 120 + 25)` |
| `wheel_v_max_mm_s` | 340 | **202** | `(55/60) × pi × 70` |
| BOM motor | ~100 RPM | **~55 RPM** | Pololu #3482 |
| `drive_mixing_vectors.csv` | — | **regenerate ทั้งตาราง** | saturation ขึ้นกับ `wheel_v_max` |
| `firmware/.../mixing.h` | 340.0f | **202.0f** | ต้องตรงกับ config |

`wheel_v_min_mm_s` **คงไว้ที่ 51** โดยเจตนา — โมเดล 15% duty กับมอเตอร์ตัวใหม่
ให้ 30 แต่เกียร์ 250:1 มีชั้นเฟืองมากกว่า stiction สูงกว่า deadband จริงจึงอาจ
**สูงกว่า** 15% ไม่ใช่ต่ำกว่า เก็บค่าที่เข้มกว่าไว้จนกว่าจะวัดจริงที่ V3

### margin ของ invariant หลังเปลี่ยน — ดีขึ้นทุกข้อ

```text
wheel_diameter >= 4 × soil_variation      70 >= 60      margin 10 mm   (เดิม 5)
clear_furrow   >  body_width + 2×runaway 290 > 265      margin 25 mm   (เดิม 24)
outer wheel    <= wheel_v_max           141.9 <= 202    ok
inner wheel    >= wheel_v_min            58.1 >= 51     margin 7.1 mm/s (เท่าเดิม)
```

---

## ⚠️ ค่าที่ยังไม่ได้ยืนยัน — ต้องวัดจาก STEP ใน Fusion

| ค่า | ตั้งไว้ | ต้องทำ |
|---|---|---|
| `motor_mount_pitch_mm` | **18** | **เดา** — วัดระยะรู M2.5 จริงจาก STEP ของ #3482 แล้วแก้ `parameters.csv` ถ้าไม่ตรง |

ค่านี้อยู่ใน `EXPECTED_PARAMETERS` ของ `tests/unit/test_cad_config_sync.py` แต่
ไม่มี config ตัวไหน derive จากมัน — test จึงไม่จับว่ามันผิด **ต้องวัดด้วยมือ**

ถ้าใช้ขายึด Pololu #1138 ระยะรูไม่สำคัญต่อการออกแบบเท่าไร (ขายึดจัดการให้)
แต่ตัวเลขใน `parameters.csv` ควรเป็นของจริงอยู่ดี

---

## งบมวล — เพดาน 2.3 kg

```text
ไต่ก้อนดิน h=15 mm ด้วยล้อ r=35 mm   torque = W × 4.33 kg·cm ต่อล้อ
Pololu แนะนำโหลดต่อเนื่องไม่เกิน 5 kg·cm  (stall 14 แต่ห้ามออกแบบไปที่ 14)

รถ 2.0 kg  →  4.33 kg·cm   ✓
รถ 2.5 kg  →  5.41 kg·cm   ✗
```

**ต้องคุมมวลตั้งแต่ออกแบบแชสซี ไม่ใช่ชั่งทีหลัง** และเสากล้อง 220 mm บนรถกว้าง
145 mm ทำให้จุดศูนย์ถ่วงสูงอยู่แล้ว — แบตกับมอเตอร์ต้องอยู่ต่ำที่สุด

---

## ส่งต่อให้ session ที่ทำ Fusion

### ลำดับ

```text
1. โหลด STEP ทั้ง 4 ตัวจาก Pololu (D1 D2 D4 D5) + Pi 5 จาก datasheets.raspberrypi.com
2. Insert เข้า Fusion เป็น component
3. วัด motor_mount_pitch_mm จริงจาก D1 → แก้ parameters.csv ถ้าไม่ตรง 18
4. สร้าง user parameters ใน Fusion ให้ตรงกับ parameters.csv ทุกตัว (16 ตัว)
5. ประกอบ: แชสซี → มอเตอร์ 4 ตัว → ล้อ → เสากล้อง → Pi 5 → แบต
6. Export parameters.csv ทับของเดิม → commit
7. Export mesh/STEP → cad/exports/
8. เขียน cad/urdf/weeding_rover.urdf
```

### ข้อจำกัดที่ห้ามละเมิด

```text
ความยาวมอเตอร์แต่ละตัว    <= 47 mm จากแกนกลาง (ซ้าย/ขวาชนกัน)
จุดกว้างสุดของรถ           145 mm  (= track 120 + wheel 25) — invariant ข้อ 5
ท้องรถถึงพื้น              35 mm   — invariant ข้อ 4 (> soil_variation 15)
มวลรวม                     <= 2.3 kg
camera_front_tilt_deg      45°     ห้ามขยับ — ผูกกับ gain และไม่มี test จับ
camera_front_height_mm     180 mm  เหมือนกัน
camera_down_height_mm      220 mm  ผูกกับการคำนวณ FOV ด้านบน
```

### ทำเสร็จแล้วต้องผ่าน

```bash
pytest tests/unit/test_cad_config_sync.py
pytest tests/unit/test_startup_invariants.py
```

**ถ้า invariant fail ให้แก้ของหรือแก้ CAD — ห้ามแก้ `config/` เพื่อให้ผ่าน**
(กฎนี้เขียนไว้ที่ [../../cad/README.md](../../cad/README.md#กฎการแก้ขนาด))

### Git LFS ตั้งไว้แล้ว

`.gitattributes` track `f3d · step · stp · stl · obj · dae · usd · usdc` แล้ว
`parameters.csv` และ `.urdf` **ไม่อยู่ใน LFS** โดยเจตนา — ต้องการ diff ที่อ่านได้

---

## ยังต้องเลือกต่อ

```text
D3  driver BTS7960 / IBT-2      สเปกเดิมใน BOM ยังใช้ได้ (stall 1.6 A × 2 = 3.2 A/ช่อง)
P1  แบต 3S LiPo                  กระทบงบมวลโดยตรง — เลือกพร้อมออกแบบแชสซี
P2  buck 5 V ×2                  Pi 5 กินไฟมากกว่า Pi 4 ต้องเช็คกระแสใหม่
P6  E-stop latching NC           ขนาดตัวปุ่มกระทบ layout
M1  แผ่นแชสซี                     3 mm อะคริลิก/อลูมิเนียม หรือ 3D print
```
