# Communication Protocol

Transport อยู่ใน `bridge/` — message contract อยู่ใน `protocol/`

---

## Transport

```text
Serial เท่านั้น
```

Pi กับ ESP32 อยู่บนบอร์ดเดียวกัน — ไม่มี Wi-Fi ใน control path

**MQTT ออกจาก MVP** เพราะ control loop ไม่ข้ามเครือข่ายอีกแล้ว
log เขียนไฟล์บน Pi ดึงทีหลัง

Bandwidth ที่ **115200 baud** (11.5 KB/s):

```text
ขึ้น   drive       ~70 B × 10 Hz  =   700 B/s
ลง    rover_state ~140 B × 20 Hz = 2,800 B/s
                                  ─────────────
                                    ~30% ของ link
```

มี headroom พอเพิ่ม loop rate เป็น 50 Hz โดยไม่ต้องเปลี่ยน transport
หรือเปลี่ยน encoding เป็น MessagePack / Protobuf ภายหลังได้
**โดยไม่เปลี่ยน Controller** เพราะ Controller คุยกับ `Rover` interface เท่านั้น

---

## Encoding

POC ใช้ **JSON บรรทัดละ message** — schema อยู่ที่ `protocol/schema.json`

JSON ที่ 10 Hz บน serial ถูกพอ และอ่านด้วยตาได้ตอน debug ซึ่งมีค่ามากกว่า
การประหยัด byte ในขนาดงานนี้

---

## Streaming กับ Discrete — ความต่างที่เป็นหัวใจของ protocol นี้

Protocol ของ design gantry เป็น request/response ล้วน: ทุก command มี `id`
รอ `ack` แล้วรอ `result` และ **command ที่ไม่ได้ `ack` ภายใน timeout → `ERROR`**

ถูกต้องสำหรับ `move` / `home` / `tool` ที่สั่งครั้งเดียวแล้วรอผล

แต่ velocity command ส่ง 10 ครั้งต่อวินาทีตลอดเวลา ถ้าใช้ pattern เดิม
**เฟรมที่ตกหนึ่งเฟรมจะกลายเป็น ERROR** และจะมี `ack` ไหลกลับ 10 Hz ที่ไม่มีใครใช้

| | Streaming | Discrete |
|---|---|---|
| ตัวอย่าง | `drive` | `stop` · `emergency_stop` · `reset` |
| `ack` | **ไม่มี** | มี |
| ตกหล่นได้ | ได้ — ตัวถัดไปมาใน 100 ms | ไม่ได้ |
| Semantics | latest-wins | ต้องทำทุกตัว |
| ยืนยัน liveness | telemetry echo `last_seq` | `ack` |
| Correlation field | `seq` (monotonic) | `id` (monotonic) |

**กฎ "ไม่ได้ ack → ERROR" ใช้กับ discrete command เท่านั้น** — นี่คือการแก้กฎ
ของ protocol ไม่ใช่แค่เพิ่ม message type

---

## Drive Command (streaming)

```json
{"type":"drive","seq":1234,"v_mm_s":100.0,"omega_deg_s":-12.5}
```

ไม่มี response ต่อ message — สถานะอยู่ใน `rover_state` ที่ไหลอยู่แล้ว

ESP32 ต้องได้รับ `drive` ใหม่ภายใน `command_timeout_ms` ไม่งั้นดับมอเตอร์เอง

---

## Discrete Commands

```json
{"type":"stop","id":77}
{"type":"emergency_stop","id":78}
{"type":"reset","id":79}
```

ทั้งสามต้องได้ `ack` ภายใน timeout ไม่งั้น controller เข้า `ERROR`

---

## Rover State (telemetry, 20 Hz)

```json
{"type":"rover_state",
 "commanded":{"v_mm_s":100.0,"omega_deg_s":-12.5},
 "last_seq":1234,
 "estop":false,
 "motors_enabled":true,
 "uptime_ms":48210}
```

`commanded` เป็น **echo ของคำสั่ง ไม่ใช่ค่าที่วัดได้** — ไม่มี encoder
การซ้อนใต้ key ชื่อ `commanded` ทำให้อ่าน code แล้วรู้ทันที ห้ามแบนออกมาเป็น
`v_mm_s` ระดับ top-level เพราะคนอ่านจะเข้าใจว่าเป็น feedback

---

## `seq` + `last_seq` — วัด link age โดยไม่ต้องซิงก์นาฬิกา

Pi จำเวลาที่ส่ง `seq` แต่ละตัว · ESP32 echo `last_seq` ที่รับได้ · Pi คำนวณ

```python
link_age_ms = now() - sent_at[state["last_seq"]]
```

ไม่ต้องมี clock sync ระหว่าง Pi กับ ESP32 และไม่ต้องมี `ack` แยกสำหรับ `drive` —
telemetry ที่ต้องส่งอยู่แล้วทำหน้าที่นี้ไปพร้อมกัน

`uptime_ms` เป็นนาฬิกาของ ESP32 เอง ใช้ตรวจว่า MCU รีบูตกลางทาง (ค่าลดลง)
ไม่ใช่ใช้คำนวณ latency

---

## Latest-wins ต้องบังคับที่ฝั่ง ESP32

serial buffer สะสมได้ ถ้า ESP32 ประมวลผลเรียงตามคิว มันจะเลี้ยวตามคำสั่งเก่า
ที่ Pi ยกเลิกความคิดไปแล้ว

```text
ทุกรอบ loop:  อ่าน line ทั้งหมดที่มีใน buffer
              ใช้ drive ตัวที่ seq สูงสุด  ทิ้งที่เหลือ
              seq <= last_seq  →  ทิ้ง (out of order)
```

`stop` / `emergency_stop` ที่พบใน buffer ต้อง **ทำทันทีไม่ว่าลำดับไหน**
และ override `drive` ทุกตัวในรอบนั้น

---

## Error Codes

```json
{"type":"error","code":"command_timeout","message":"no drive command for 312 ms"}
```

```text
link_lost           Pi:    link_age_ms เกิน link_lost_ms
command_timeout     ESP32: ไม่ได้รับ drive เกิน command_timeout_ms → ดับมอเตอร์
row_lost            Pi:    estimate invalid ติดกัน แต่ green_fraction ยังสูง
camera_timeout      Pi:    capture ไม่คืนเฟรมในเวลา
config_invalid      Pi:    startup invariant ไม่ผ่าน → ไม่ start
communication_lost  Pi:    serial port หลุด / เปิดไม่ได้
emergency_stop      ทั้งคู่: E-stop ถูกกด
```

| Code | การจัดการของ Controller |
|---|---|
| `command_timeout` | ถือว่า rover หยุดแล้วจริง เข้า `ERROR` **ห้าม resume เอง** |
| `link_lost` | `stop()` + `ERROR` |
| `row_lost` | `stop()` + `ERROR` |
| `config_invalid` | ไม่ start เลย บอกค่าที่ขัดกันเป็นตัวเลข |

`command_timeout` กับ `link_lost` เป็นเรื่องเดียวกันที่มองจากสองฝั่ง และ
**ต้องมีทั้งคู่** — ถ้ามีแค่ฝั่ง Pi แล้ว Pi แครช ไม่มีใครสั่งหยุด

### Code ที่หายไปจาก design gantry

```text
endstop_error       ไม่มี endstop
motor_timeout       → แยกเป็น link_lost / command_timeout
position_mismatch   ไม่มี position ให้ mismatch
tool_no_contact     ไม่มี tool  (กลับมาที่ M3)
tool_overreach      ไม่มี tool  (กลับมาที่ M3)
```

---

## Correlation

เฉพาะ discrete command:

```text
STOP id=77
     ↓
ACK  id=77
```

Streaming command ไม่มี correlation แบบนี้ — ใช้ `last_seq` ใน telemetry แทน

---

## Files

| File | Purpose |
|---|---|
| `protocol/messages.md` | Message catalog แบบละเอียด |
| `protocol/schema.json` | JSON Schema สำหรับ validate ทุก message |
| `protocol/examples/` | ตัวอย่าง payload จริงสำหรับ test fixture |
| `config/drive_mixing_vectors.csv` | Golden vector ของ skid-steer mixing (ใช้ร่วมกับ firmware) |
