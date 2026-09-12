# tools

Utility ที่ไม่อยู่ใน runtime path ของหุ่นยนต์

| Directory | Purpose |
|---|---|
| `calibration/` | Capture calibration target, solve intrinsic + homography ของ **กล้องล่าง**, เขียนผลลง `config/camera.yaml` (กล้องหน้าไม่ calibrate) |
| `dataset/` | Export image + label จาก Isaac Sim (domain randomization) สำหรับเทรน weed detector |
| `visualization/` | Plot detection overlay, **valley histogram**, `lateral_err` trace, rover path, scenario result |

---

## Notes

- Tool ใน folder นี้ **ห้าม** ถูก import จาก `controller/` หรือ `perception/` runtime
- `dataset/` ใช้ ground truth จาก Isaac Sim ทำให้ได้ label ฟรี —
  ใช้คู่กับ `sim/isaac/environments/domain_randomization.py`
- Calibration procedure: [../docs/calibration.md](../docs/calibration.md)

## เครื่องมือที่ต้องมีตั้งแต่ V1

| เครื่องมือ | ทำไมจำเป็น |
|---|---|
| Plot **valley histogram** ต่อเฟรม | เป็นวิธีเดียวที่ debug `row_estimator` ได้เร็ว — เห็นทันทีว่าหุบหายไปที่เฟรมไหน |
| Plot `lateral_err` / `omega` เทียบเวลา | แยกให้ออกว่า gain สูงเกิน (แกว่ง) หรือ mechanical เพี้ยน (offset ค้าง) |
| Plot error map ของ homography ทั่วเฟรม | หาโซนที่ error เกิน budget ±20 mm |
| วัด `max_lateral_error` จาก log ของ run | เป็นค่าที่ corridor ของ weed detector พึ่งอยู่ — ต้องวัด ไม่ใช่เดา |

สองตัวแรกสำคัญกว่าที่คิด: `RowFollower` เป็น pure function ที่ test ได้ด้วยตัวเลข
แต่ **`row_estimator` ทำงานกับภาพจริง** และมันคือจุดที่ MVP จะพังจริง ๆ
ถ้าไม่มีเครื่องมือดู histogram จะเหลือแค่การเดาว่าทำไม `valid = False`
