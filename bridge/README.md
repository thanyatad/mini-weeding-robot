# bridge

เชื่อม Controller กับ ESP32 หรือ Simulation

Protocol contract: [../docs/protocol.md](../docs/protocol.md)

---

## Layout

```text
bridge/
├── main.py                 # bridge process entry point
├── serial_link.py          # Serial transport — JSON บรรทัดละ message
├── protocol.py             # encode / decode + id correlation (discrete เท่านั้น)
├── seq_tracker.py          # seq → sent_at  แล้วคำนวณ link_age_ms จาก last_seq
└── simulator.py            # ESP32 emulator สำหรับ integration test
```

ไม่มี `mqtt.py` — ดู [§Transport](#transport)

---

## Role

```text
Controller ──drive / stop / estop──> bridge ──serial──> ESP32 / Simulation
                                  <──rover_state / ack / error──
```

Bridge รับผิดชอบ **transport + encoding เท่านั้น** — ไม่มี business logic

---

## Transport

```text
Serial เท่านั้น
```

Pi กับ ESP32 อยู่บนบอร์ดเดียวกัน control loop ไม่ข้ามเครือข่าย

**MQTT ถูกตัดออกจาก MVP** — Wi-Fi ไม่อยู่ใน control path อีกแล้ว
log เขียนไฟล์บน Pi ดึงทีหลัง

ที่ **115200 baud** (11.5 KB/s) ใช้ ~30% ของ link:

```text
ขึ้น   drive       ~70 B × 10 Hz  =   700 B/s
ลง    rover_state ~140 B × 20 Hz = 2,800 B/s
```

มี headroom พอเพิ่ม loop rate เป็น 50 Hz หรือเปลี่ยน encoding เป็น
MessagePack / Protobuf ภายหลังได้ **โดยไม่เปลี่ยน Controller**

---

## Streaming กับ Discrete — สองเส้นทางใน `protocol.py`

| | Streaming | Discrete |
|---|---|---|
| Message | `drive` | `stop` · `emergency_stop` · `reset` |
| Correlation | `seq` → `seq_tracker.py` | `id` → pending table |
| ตกหล่นได้ | ได้ | ไม่ได้ |
| Timeout → ERROR | **ไม่** | ใช่ |

`protocol.py` เก็บ pending command table และ raise timeout **เฉพาะ discrete command**

ถ้าเอา `drive` เข้า pending table ด้วย เฟรมที่ตกหนึ่งเฟรมจะกลายเป็น `ERROR`
และ table จะโตขึ้น 10 entry ต่อวินาทีตลอดเวลา

---

## `seq_tracker.py` — วัด link age โดยไม่ต้องซิงก์นาฬิกา

```python
tracker.sent(seq)                       # จำเวลาส่ง
...
link_age_ms = tracker.age(state["last_seq"])
```

ไม่ต้องมี clock sync ระหว่าง Pi กับ ESP32 — Pi ใช้นาฬิกาตัวเองทั้งขาส่งและขาคำนวณ
ESP32 แค่ echo `seq` กลับมา

`link_age_ms` เป็นค่าเดียวใน `get_drive_state()` ที่ **วัดได้จริง** —
`commanded` เป็น echo ไม่ใช่ feedback

`seq_tracker` ต้องตัด entry เก่าทิ้ง ไม่งั้นจะเก็บ 10 entry ต่อวินาทีไปเรื่อย ๆ
เก็บย้อนหลังเท่าที่ `link_lost_ms` ต้องการก็พอ

---

## `simulator.py`

ESP32 emulator — ใช้ใน `tests/integration/` เพื่อทดสอบ

```text
Controller + Bridge + ESP32 Emulator
```

โดยไม่ต้องมี hardware หรือ Isaac Sim

Emulator ต้องเลียนพฤติกรรมที่ firmware จริงต้องมี ไม่ใช่แค่ตอบ ack:

```text
[ ] command timeout — ไม่ได้ drive เกิน command_timeout_ms → motors_enabled: false
[ ] latest-wins — ยัด drive หลายตัว ใช้ตัว seq สูงสุด
[ ] ทิ้ง seq ที่ย้อนหลัง
[ ] stop / emergency_stop override drive ในรอบเดียวกัน
[ ] ปฏิเสธ reset ขณะ estop sense ยัง active
[ ] uptime_ms ลดลงเมื่อจำลองการรีบูต
```

ถ้า emulator ตอบ ack ทุกอย่างแบบสุภาพ integration test จะผ่านทั้งหมด
แล้วไปพังที่ V3 ซึ่งเป็นจุดที่ debug แพงกว่ามาก
