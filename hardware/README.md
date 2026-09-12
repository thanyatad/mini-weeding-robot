# hardware

เอกสารของเครื่องจริง — สิ่งที่จับต้องได้ ประกอบได้ และวัดได้

```text
hardware/
├── bom/
│   └── poc-v2.md               รายการของ + ข้อกำหนดที่ต้องตรวจก่อนซื้อ
│
├── mechanical/
│   ├── dimensions.md           ขนาดทั้งหมด (derived จาก CAD)
│   ├── assembly.md             ลำดับประกอบ + ค่าที่ต้องวัดระหว่างทาง
│   └── coordinate-frames.md    นิยาม frame ทุกตัวในระบบ
│
└── electrical/
    ├── wiring.md               pin map + กับดักของ ESP32
    └── power.md                budget + topology + safety
```

---

## เริ่มอ่านจากไหน

| ถ้าคุณกำลัง… | อ่าน |
|---|---|
| ประเมินงบ / จะซื้อของ | [bom/poc-v2.md](bom/poc-v2.md) |
| ประกอบ rover | [mechanical/assembly.md](mechanical/assembly.md) |
| เดินสาย | [electrical/wiring.md](electrical/wiring.md) → [electrical/power.md](electrical/power.md) |
| เขียน firmware | [electrical/wiring.md](electrical/wiring.md) + [../docs/hardware.md](../docs/hardware.md) |
| เขียน perception / row follower | [mechanical/coordinate-frames.md](mechanical/coordinate-frames.md) |
| แก้ขนาด rover | [../cad/parameters/README.md](../cad/parameters/README.md) |

---

## `hardware/` กับ `docs/hardware.md` ต่างกันยังไง

| | ขอบเขต |
|---|---|
| `hardware/` | **เครื่องจริง** — ของที่ซื้อ, ขนาด, การประกอบ, สายไฟ, แปลงทดสอบ |
| `docs/hardware.md` | **firmware & integration** — ESP32 ทำอะไร, mixing, timeout, build/flash, HIL |

เส้นแบ่ง: *ถ้าต้องใช้ไขควง = `hardware/` · ถ้าต้องใช้ compiler = `docs/hardware.md`*

---

## เอกสารนี้ไม่ใช่ต้นทางของตัวเลข

ขนาดทุกตัวใน `mechanical/dimensions.md` เป็น **derived จาก CAD**

```text
cad/fusion/rover.f3d
   └─► cad/parameters/parameters.csv   ◄── ต้นทาง
          ├─► hardware/mechanical/dimensions.md   (เอกสาร)
          ├─► config/rover.yaml                   (runtime)
          ├─► cad/urdf/weeding_rover.urdf         (simulation)
          └─► config/drive_mixing_vectors.csv     (firmware + sim)
```

ถ้าตัวเลขที่นี่ขัดกับ `parameters.csv` → **CSV ถูก** แล้วเอกสารนี้ล้าสมัย

ตรวจ drift ด้วย:

```bash
pytest tests/unit/test_cad_config_sync.py
```

---

## ค่าที่เอกสารชุดนี้ยัง **เดา** อยู่

ตัวเลขในเอกสารชุดนี้เป็นค่าเริ่มต้นที่สอดคล้องกัน ไม่ใช่ค่าที่วัดจากของจริง
ห้าเก้าค่าต่อไปนี้ต้องวัดแล้วเขียนกลับก่อนเชื่ออะไรที่สืบทอดจากมัน

| ค่า | วัดที่ | ถ้าผิดแล้วเกิดอะไร |
|---|---|---|
| `wheel_v_min_mm_s` | V3 ล้อลอย | `omega_max` สูงเกิน ล้อข้างในหยุดเงียบ |
| `body_width_mm` (จุดกว้างสุด) | หลังใส่ล้อ | invariant ข้อ 5 ผ่านโดยที่ล้อยังเบียดใบพืช |
| `chassis_clearance_mm` (พร้อมโหลด) | หลังประกอบครบ | ท้องครูดยอดดินที่ invariant บอกว่าผ่าน |
| `crop_foliage_half_width_mm` | V1 asset พืช | กระทบ invariant ข้อ 5 **และ** corridor พร้อมกัน |
| `soil_variation_mm` | หลังปรับหน้าดิน | ล้อ 65 mm ไม่พอ (margin เหลือ 5 mm) |
| `runaway_budget_mm` | หลังวางลงแปลง | ค่าความปลอดภัยหลักของ MVP |
| FOV กล้องล่างจริง | ก่อนสั่งกล้อง | เห็นร่องไม่ครบ → ใบพืชผลถูกนับเป็นวัชพืช |
| `error_budget_mm` (homography) | หลัง calibrate | ตัวเลขใน weed log ไม่มีความหมาย |
| `gains.esp32` | V4 tune | ค้างเป็น `null` = ไม่ start (ตั้งใจ) |

---

## กับดักที่เอกสารชุดนี้เขียนไว้ให้แล้ว

เรื่องที่เสียเวลา debug มากที่สุดถ้าไม่รู้ก่อน — เรียงตามความเจ็บ

| เรื่อง | ที่ |
|---|---|
| **`omega` กลับด้าน → row follower เป็น positive feedback** | [electrical/wiring.md](electrical/wiring.md#ตรวจทิศทางก่อนปิดฝา) |
| **ESP32 กินไฟจาก Pi ทำให้ safety เหลือ 2 ชั้นไม่ใช่ 3** | [electrical/power.md](electrical/power.md#️-esp32-ต้องไม่กินไฟจาก-pi) |
| **Pi รีบูตตอนมอเตอร์ออกตัว (buck ตัวเดียว)** | [electrical/power.md](electrical/power.md#️-ต้องมี-buck-สองตัว--ไม่ใช่หนึ่ง) |
| **มอเตอร์เร็วเกินทำให้อยู่ใน deadband ตลอดเวลา** | [mechanical/dimensions.md](mechanical/dimensions.md#️-มอเตอร์ที่เร็วกว่า-แย่กว่า-ไม่ใช่ดีกว่า) |
| **`body_width` ≠ ความกว้างแชสซี (ล้อยื่นออก)** | [mechanical/dimensions.md](mechanical/dimensions.md#️-body_width--ความกว้างแชสซี) |
| **`camera_front_tilt` ผูกกับ gain — ไม่มี test จับ** | [mechanical/coordinate-frames.md](mechanical/coordinate-frames.md#️-แต่-extrinsic-ผูกกับ-gain) |
| ล้อซ้าย/ขวาคนละขนาด → ดูเหมือน gain ผิด | [mechanical/assembly.md](mechanical/assembly.md#ล้อคนละขนาดคือกับดักที่หายากที่สุด) |
| GPIO 34/35/36/39 ไม่มี internal pull-up | [electrical/wiring.md](electrical/wiring.md#️-gpio-34-35-36-39-ไม่มี-internal-pull-up) |
| E-stop ที่ตัด logic ด้วย → รายงานสถานะไม่ได้ | [electrical/power.md](electrical/power.md#e-stop-ตัดแค่-motor-rail) |
| soil reference plane ≠ ผิวดินจริง | [mechanical/coordinate-frames.md](mechanical/coordinate-frames.md#bed--ยังนิยามไว้-เพราะ-sim-และเอกสารต้องใช้) |
| mm ↔ m และ deg ↔ rad ที่รอยต่อ URDF | [mechanical/coordinate-frames.md](mechanical/coordinate-frames.md#unit-convention) |
| เสากล้องโยก → ภาพสั่น → `omega` กระตุก | [electrical/wiring.md](electrical/wiring.md#cable-management) |
| แบตอ่อน → deadband กินพิสัย `omega` แบบเงียบ | [electrical/power.md](electrical/power.md#low-voltage-behaviour) |

สามรายการบนสุดเป็นกลุ่มที่ **ทำให้ระบบดูเหมือนทำงานแล้วพังในแปลง** —
อ่านก่อนจ่ายไฟครั้งแรก

---

## ⚠️ MVP นี้ไม่ควรรันโดยไม่มีคนดู

```text
ไม่มี bumper switch
ไม่มี ToF / ultrasonic
ไม่มี obstacle avoidance
ไม่มี encoder จะรู้ว่าล้อหมุนจริงไหม
```

ตัวชดเชยทางกายภาพมีแค่สองอย่าง: **ขอบแปลงยกสูงกว่ารัศมีล้อ** และ
**`runaway_budget_mm` ที่ validate ตอน startup**

ถ้าขอบแปลงไม่ยก หรือ `runaway_budget` ยังไม่ได้วัดจริง — ต้องมีคนยืนถือ E-stop

ดู [../docs/superpowers/specs/2026-09-12-rover-mvp-design.md](../docs/superpowers/specs/2026-09-12-rover-mvp-design.md#13-known-limitations)
