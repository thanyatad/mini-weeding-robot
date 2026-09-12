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
