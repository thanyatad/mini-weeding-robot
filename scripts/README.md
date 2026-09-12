# scripts

Entry point สำหรับ run และ flash

| Script | Purpose |
|---|---|
| `run_sim.sh` | เปิด Isaac Sim + โหลด scene (`sim/isaac/app.py`) |
| `run_controller.sh` | รัน Controller ตาม backend ใน `config/development.yaml` |
| `run_fake_loop.sh` | Closed-loop row following ด้วย `FakeRover` — **ไม่ต้องมี Isaac** |
| `run_all.sh` | เปิด Simulation + Bridge + Controller พร้อมกัน (SIL) |
| `flash_esp32.sh` | PlatformIO build + upload firmware ไป ESP32 |
| `import_urdf.sh` | Import `cad/urdf/weeding_rover.urdf` → `rover_base.usd` (generated) |

---

## Typical Workflows

### V0 — logic only, ไม่ต้องมีกราฟิก

```bash
./scripts/run_fake_loop.sh --offset-mm 60 --heading-deg 15 --seconds 3
```

พิมพ์ trace ของ `lateral_err` และ `min_clearance_mm` ออกมา ใช้ tune gain
ก่อนแตะ Isaac เลย — ตัวเดียวกับที่ `tests/unit/` เรียกใช้

```bash
# config/development.yaml → backend: fake
./scripts/run_controller.sh
```

### V1/V2 — Software-in-the-Loop

```bash
# config/development.yaml → backend: isaac
./scripts/run_all.sh
```

### V3 — Hardware-in-the-Loop

```bash
./scripts/flash_esp32.sh
# config/development.yaml → backend: esp32
./scripts/run_all.sh
```

⚠️ **ยกล้อลอยก่อน** — safety ทั้งสามชั้นต้องถูกทดสอบตอนที่การทดสอบล้มเหลว
แล้วไม่มีอะไรวิ่งหนี ดู [../docs/hardware.md](../docs/hardware.md#hardware-in-the-loop--ต้องล้อลอย)

### V4 — แปลงจริง

```bash
# ต้อง tune gains.esp32 ใน config/control.yaml ก่อน
# ถ้ายังเป็น null → startup จะ fail ด้วย config_invalid (ตั้งใจ)
./scripts/run_controller.sh
```

---

## Startup จะปฏิเสธ config ที่ขัดกัน

ทุก script ที่รัน controller เรียก `controller/startup_checks.py` ก่อน —
invariant 9 ข้อ fail ข้อใดก็ **ไม่ start** พร้อมบอกค่าที่ขัดกันเป็นตัวเลข

```text
config_invalid: clear_furrow (690 mm) must exceed
                body_width (520) + 2 × runaway_budget (60) = 640 mm
```

นี่ไม่ใช่ความรำคาญ — มันเป็นตัวแทนของ bumper switch ที่ MVP ไม่มี
ดู [../config/README.md](../config/README.md#startup-invariants)

---

## CAD → Simulation

หลังแก้ขนาดใน `cad/parameters/parameters.csv` และ regenerate mesh แล้ว:

```bash
./scripts/import_urdf.sh
pytest tests/unit/test_cad_config_sync.py
```

`import_urdf.sh` เขียนทับ `sim/isaac/robots/rover/rover_base.usd` เสมอ —
physics tuning อยู่ใน `rover.usd` ที่เป็น layer ทับ จึงไม่หาย

⚠️ ถ้าแก้ `camera_front_tilt_deg` หรือ `camera_front_height_mm`
**gain ของ row follower ใช้ไม่ได้แล้ว** — มุมและความสูงกำหนด lookahead distance
ต้อง tune ใหม่ ไม่ใช่แค่ regenerate USD

ดู [../cad/README.md](../cad/README.md#pipeline)

---

## Firmware Test บน Host

ไม่ต้องมีบอร์ด:

```bash
pio test -e native -d firmware/esp32
```

ทดสอบ skid-steer mixing เทียบ [`config/drive_mixing_vectors.csv`](../config/drive_mixing_vectors.csv)
ตารางเดียวกับที่ Python ใช้ — ถ้าตัวใดตัวหนึ่ง fail แปลว่า implementation
สองภาษาเพี้ยนจากกัน
