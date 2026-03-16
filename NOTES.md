# 📝 ملاحظات مشروع الكرسي المتحرك الذكي

## 📍 الوضع الحالي
المحاكاة تعمل في Gazebo، الكرسي يتحرك بالكيبورد، والـ LiDAR شغال.
**المشكلة المتبقية:** ربط الـ SLAM بالـ TF Tree لرسم خريطة البيئة في RViZ.

---

## 🔧 مشكلة الـ TF (أولى الخطوات)

جازيبو بتنشر: `wheelchair/odom → wheelchair/base_footprint`  
URDF بتنشر: `base_footprint → base_link → lidar_link`  
**المشكلة:** مفيش وصلة بينهم.

**الحل موضوع في الكود:**  
`static_transform_publisher` في `spawn_wheelchair.launch.py` يربط الشجرتين.

### للتحقق إن الوصلة تعمل:
```bash
ros2 run tf2_ros tf2_echo wheelchair/odom lidar_link
```
لو طلع قيم (مش error) → الـ TF تمام والـ SLAM هيشتغل.

---

## 🚀 أوامر التشغيل

```bash
# النافذة الأولى - المحاكاة
source install/setup.bash
ros2 launch medical_wheelchair spawn_wheelchair.launch.py

# النافذة الثانية - رسم الخريطة
source install/setup.bash
ros2 launch medical_wheelchair mapper.launch.py

# النافذة الثالثة - التحكم بالكيبورد
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

في **RViZ**: غيري الـ `Fixed Frame` لـ `map` وشوفي الخريطة بتترسم.

---

## 🗺️ خطوات المشروع القادمة

| # | المهمة | الأدوات |
|---|---------|---------|
| 1 | ✅ تأكيد SLAM وظهور الخريطة | `mapper.launch.py` |
| 2 | حفظ الخريطة | `ros2 run nav2_map_server map_saver_cli -f my_map` |
| 3 | تثبيت Navigation Stack (Nav2) | `sudo apt install ros-humble-nav2-bringup` |
| 4 | إعداد Nav2 مع الخريطة المحفوظة | تعديل `nav2_params.yaml` |
| 5 | تحديد نقطة هدف من RViZ | أداة "Nav2 Goal" |
| 6 | تجربة تجنب العقبات | تحريك الصناديق ومراقبة إعادة التخطيط |

---

## 📁 الملفات المهمة

| الملف | الوظيفة |
|-------|---------|
| `launch/spawn_wheelchair.launch.py` | يشغل جازيبو + RViZ + Bridge + TF Bridge |
| `launch/mapper.launch.py` | يشغل SLAM لرسم الخريطة |
| `urdf/wheelchair.xacro` | تصميم الكرسي، المحركات، الليزر |
| `worlds/medical_office.sdf` | البيئة الداخلية (المكتب الطبي) |
| `rviz/medical_wheelchair.rviz` | إعدادات الرؤية في RViZ |

---

## 🏗️ هيكل المشروع
```
chair_ws/
├── src/
│   └── medical_wheelchair/
│       ├── launch/
│       │   ├── spawn_wheelchair.launch.py   ← التشغيل الرئيسي
│       │   └── mapper.launch.py              ← SLAM
│       ├── urdf/
│       │   └── wheelchair.xacro             ← تصميم الكرسي
│       ├── rviz/
│       │   └── medical_wheelchair.rviz      ← إعدادات RViZ
│       └── scripts/
│           └── tf_republisher.py            ← (احتياطي - غير مستخدم)
└── worlds/
    └── medical_office.sdf                   ← بيئة المكتب الطبي
```
