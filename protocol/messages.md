# Message Catalog

Contract กลางระหว่าง Controller, Bridge, ESP32 และ Simulation

Encoding: **JSON บรรทัดละ message** · Schema: `schema.json` · Examples: `examples/`

---

## Rules

1. Command แบ่งเป็น **2 ชนิด** ที่มีกฎต่างกัน — ดู §Command Classes
2. **Discrete command** ต้องมี `id` (integer, monotonic ต่อ session) และต้องได้ `ack`
3. **Streaming command** ต้องมี `seq` (integer, monotonic ต่อ session) และ **ไม่มี** `ack`
4. Discrete command ที่ไม่ได้ `ack` ภายใน timeout → Controller เข้าสู่ `ERROR`
   **กฎนี้ไม่ใช้กับ streaming command**
5. ระยะทางเป็น **mm**, ความเร็วเป็น **mm/s**, ความเร็วเชิงมุมเป็น **deg/s**, มุมเป็น **degree**
6. `emergency_stop` มีสิทธิ์สูงสุด — override ทุก command ที่ค้างอยู่ทันที
7. Streaming command เป็น **latest-wins** — ESP32 ใช้ `seq` สูงสุดที่พบในรอบนั้น ทิ้งที่เหลือ

---

## Command Classes

| | Streaming | Discrete |
|---|---|---|
| Message | `drive` | `stop` · `emergency_stop` · `reset` |
| Correlation | `seq` | `id` |
| `ack` | ไม่มี | ต้องมี |
| ตกหล่นได้ | ได้ | ไม่ได้ |
| Liveness | telemetry echo `last_seq` | `ack` |

เหตุผลที่ต้องแยก: `drive` ส่ง 10 Hz ตลอดเวลา ถ้าบังคับ `ack` ทุกตัว เฟรมที่ตก
หนึ่งเฟรมจะกลายเป็น `ERROR` และจะมี `ack` ไหลกลับ 10 Hz ที่ไม่มีใครใช้

---

## Streaming Command (Controller → Rover)

### drive

```json
{
  "type": "drive",
  "seq": 1234,
  "v_mm_s": 100.0,
  "omega_deg_s": -12.5
}
```

| Field | หน่วย | ความหมาย |
|---|---|---|
| `seq` | — | monotonic ต่อ session ใช้วัด link age และบังคับ latest-wins |
| `v_mm_s` | mm/s | `> 0` เดินหน้า |
| `omega_deg_s` | deg/s | `> 0` เลี้ยวซ้าย (CCW, z ขึ้น) |

ไม่มี response ต่อ message นี้

ESP32 ต้องได้รับ `drive` ใหม่ภายใน `command_timeout_ms` (300 ms) ไม่งั้นดับมอเตอร์เอง
และรายงาน `command_timeout` เมื่อ link กลับมา

**ค่าที่ส่งต้องผ่าน mixing แล้วอยู่ในพิสัยที่มอเตอร์ทำได้** — ดู
[`config/drive_mixing_vectors.csv`](../config/drive_mixing_vectors.csv)
และ invariant `drive` สองข้อใน [config/README.md](../config/README.md#startup-invariants)

---

## Discrete Commands (Controller → Rover)

### stop

```json
{"type": "stop", "id": 77}
```

หยุดแบบควบคุม — brake ค้าง กลับมาสั่ง `drive` ต่อได้โดยไม่ต้อง `reset`

ต่างจาก `drive` ที่ `v=0, omega=0` ซึ่งปล่อยให้ไหลตามแรงเฉื่อย

### emergency_stop

```json
{"type": "emergency_stop", "id": 78}
```

ตัด motor enable + latch — ต้อง `reset` ก่อนจะสั่ง `drive` ได้อีก

### reset

```json
{"type": "reset", "id": 79}
```

ล้าง latch ของ `emergency_stop` และ `command_timeout`

ESP32 ต้อง **ปฏิเสธ** `reset` ถ้า E-stop ทางกายภาพยังถูกกดอยู่ (sense line ยัง active)
— ตอบ `error` code `emergency_stop` แทน `ack`

---

## Responses (Rover → Controller)

### ack

```json
{"type": "ack", "id": 77}
```

เฉพาะ discrete command

### rover_state

Telemetry — publish ที่ **20 Hz** ไม่ผูกกับ command id

```json
{
  "type": "rover_state",

  "commanded": {
    "v_mm_s": 100.0,
    "omega_deg_s": -12.5
  },

  "last_seq": 1234,
  "estop": false,
  "motors_enabled": true,
  "uptime_ms": 48210
}
```

| Field | ความหมาย |
|---|---|
| `commanded` | **echo ของคำสั่ง ไม่ใช่ค่าที่วัดได้** — ไม่มี encoder |
| `last_seq` | `seq` ล่าสุดที่ ESP32 รับและใช้ Pi เอาไปคำนวณ `link_age_ms` |
| `estop` | สถานะ sense line หลัง E-stop (อ่านจากไฟจริง ไม่ใช่ตัวแปร software) |
| `motors_enabled` | `false` เมื่อ timeout / estop / ยังไม่ reset |
| `uptime_ms` | นาฬิกาของ ESP32 — ค่าลดลง = MCU รีบูต |

**ไม่มี field `measured`** โดยเจตนา ถ้าวันหนึ่งเพิ่ม encoder (M2) ให้เติม
`"measured": {...}` แบบ additive โดยไม่แตะ key เดิม

**ห้ามแบน `commanded` ออกมาเป็น `v_mm_s` ระดับ top-level** — คนอ่านจะเข้าใจว่า
เป็น feedback แล้วเขียน code ที่พึ่งพามันเหมือนเป็นค่าที่วัดได้

### error

```json
{
  "type": "error",
  "code": "command_timeout",
  "message": "no drive command for 312 ms"
}
```

`id` มีเฉพาะกรณีที่ error ตอบ discrete command

Code ที่กำหนดไว้:

```text
link_lost
command_timeout
row_lost
camera_timeout
config_invalid
communication_lost
emergency_stop
```

| Code | ผู้ตรวจพบ | ความรุนแรง | การจัดการของ Controller |
|---|---|---|---|
| `command_timeout` | ESP32 | สูง | ถือว่า rover หยุดแล้วจริง เข้า `ERROR` **ห้าม resume เอง** |
| `link_lost` | Pi | สูง | `stop()` + `ERROR` |
| `row_lost` | Pi | สูง | `stop()` + `ERROR` — ต่างจาก `row_end` ที่ไป `STOPPED` |
| `camera_timeout` | Pi | สูง | `stop()` + `ERROR` |
| `config_invalid` | Pi (startup) | ตาย | ไม่ start บอกค่าที่ขัดกันเป็นตัวเลข |
| `communication_lost` | Pi | สูง | `ERROR` — serial เปิดไม่ได้หรือหลุด |
| `emergency_stop` | ทั้งคู่ | ตาย | `ESTOP` — ออกได้เมื่อปลดปุ่มแล้ว ack |

`command_timeout` กับ `link_lost` เป็นเรื่องเดียวกันที่มองจากสองฝั่งและ
**ต้องมีทั้งคู่** — ถ้ามีแค่ฝั่ง Pi แล้ว Pi แครช ไม่มีใครสั่งหยุด

### Code ที่ตัดออกจาก design gantry

```text
endstop_error       ไม่มี endstop
motor_timeout       → แยกเป็น link_lost (Pi) / command_timeout (ESP32)
position_mismatch   ไม่มี position ให้ mismatch
tool_no_contact     ไม่มี tool  (กลับมาที่ M3)
tool_overreach      ไม่มี tool  (กลับมาที่ M3)
```

---

## Correlation Example

Discrete:

```text
STOP id=77
     ↓
ACK id=77
```

Streaming — ไม่มี correlation ต่อ message ใช้ telemetry ที่ไหลอยู่แล้ว:

```text
Pi                                ESP32
│                                 │
├── drive seq=101 ───────────────►│
├── drive seq=102 ───────────────►│
│                                 ├── rover_state last_seq=102 ──►
├── drive seq=103 ──── หาย ✗      │
├── drive seq=104 ───────────────►│
│                                 ├── rover_state last_seq=104 ──►
```

`seq=103` หายไปโดยไม่มีใครต้องทำอะไร — `seq=104` มาถึงใน 100 ms
และ `link_age_ms` ยังต่ำกว่า `link_lost_ms` อยู่
