# cad/exports

โฟลเดอร์นี้มีสองแหล่งที่มา และกฎคนละข้อ

| | ต้นฉบับ | แก้ยังไง |
|---|---|---|
| `meshes/` | `cad/parameters/parameters.csv` | รัน `python tools/generate_sim_meshes.py` — **ห้ามแก้ไฟล์ด้วยมือ** |
| `step/` · `stl/` | `cad/fusion/rover.f3d` | แก้ user parameter ใน Fusion แล้ว export ใหม่ |

`meshes/` เป็น simplified visual + collision สำหรับ simulator เท่านั้น
`step/` และ `stl/` เป็น geometry สำหรับผลิต — ยังว่างอยู่จนกว่า Fusion assembly
จะถูกแก้เป็น V0 ดู [../README.md](../README.md)

---

## Formats

| Folder | Format | ใช้ทำอะไร | ความละเอียด |
|---|---|---|---|
| `step/` | STEP (.step) | งานผลิต, ส่งร้าน, review ทางวิศวกรรม | เต็ม |
| `stl/` | STL (.stl) | 3D print แชสซี / bracket / ขายึดมอเตอร์ / เสากล้อง | สูง |
| `meshes/` | OBJ / DAE | visual + collision ใน simulation | **simplified** |

---

## `meshes/` ต้อง simplified

Mesh สำหรับ sim ไม่ใช่ mesh สำหรับพิมพ์ — เอา CAD mesh เต็ม ๆ ไปใส่ Isaac
จะทำให้ physics ช้าลงมากโดยไม่ได้ความแม่นยำที่มีความหมาย

| ประเภท | เป้าหมาย |
|---|---|
| Visual mesh | พอสวย ลด triangle ให้มากที่สุดเท่าที่ยังดูรู้เรื่อง |
| Collision mesh | **convex hull หรือ primitive** ไม่ใช่ mesh ละเอียด |

Collision ของ rover เป็น **box สำหรับตัวถัง + cylinder สำหรับล้อ 4 ตัว** เท่านั้น —
ใช้ primitive เร็วกว่าและเสถียรกว่า mesh เสมอ

**ล้อต้องไม่ใช่ mesh** แม้จะมีดอกยางใน CAD — mesh ล้อบน heightfield สร้าง contact
point จำนวนมาก ทำให้ solver ช้าลงมากและไม่เสถียร โดยไม่ได้ความแม่นยำที่มีความหมาย
เพราะ grip จริงมาจาก friction coefficient ใน `rover.usd` ไม่ใช่รูปดอกยาง

**MVP นี้ไม่มีข้อยกเว้น** — design gantry ยกเว้นให้หัว tool ใช้ mesh ได้
เพราะรูปทรงหัว tool มีผลกับการสัมผัสดินจริง MVP ไม่มี tool จึงไม่มีชิ้นไหน
ที่ต้องการ collision mesh ละเอียด (ข้อยกเว้นนี้กลับมาที่ M3)

---

## Export Checklist

เมื่อขนาดใน Fusion เปลี่ยน:

```text
[ ] step/    export ชิ้นที่กระทบ
[ ] stl/     export เฉพาะชิ้นที่ต้องพิมพ์ใหม่
[ ] meshes/  export + simplify + ตรวจว่า origin ตรงกับที่ URDF คาด
[ ] cad/parameters/parameters.csv
[ ] regenerate URDF → base USD
[ ] pytest tests/unit/test_cad_config_sync.py
```

**origin ของ mesh ต้องตรงกับ link frame ใน URDF** — เป็นความผิดพลาดที่พบบ่อยที่สุด
ในขั้นนี้ และอาการคือชิ้นส่วนลอยหรือหมุนผิดจุดใน Isaac
ดู [../../hardware/mechanical/coordinate-frames.md](../../hardware/mechanical/coordinate-frames.md)

---

## Git LFS

ไฟล์ในโฟลเดอร์นี้ควรอยู่ใน LFS ทั้งหมด ดู [../README.md](../README.md#git-lfs)
