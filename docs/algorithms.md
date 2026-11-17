# Algoritma Persepsi, Navigasi, dan Kontrol AUV

Dokumen ini menjelaskan algoritma inti yang diimplementasikan di **MATLAB/Simulink** dan **ROS 2** untuk skenario *Underwater Drone Hide and Seek*.

---

## 1. Model Dinamika 6-DOF (Fossen)
Persamaan gerak BlueROV2 Heavy dimodelkan sebagai:
$$M \dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau + \tau_{\text{current}}$$
- **Massa & Added Mass ($M = M_{RB} + M_A$):** Memperhitungkan inersia bodi serta massa fluida yang ikut terakselerasi.
- **Buoyancy ($g(\eta)$):** Gaya apung Archimedes + torsi pemulih metasentris ($r_b - r_g$).
- **Hydrodynamic Drag ($D(\nu)\nu$):** Kombinasi skin friction linear dan form drag kuadratik relatif terhadap arus laut $\tau_{\text{current}}$.
- **Integrasi:** Runge-Kutta Orde 4 (RK4) pada $\Delta t = 0.05\text{ s}$ (`MATLAB/+auv/FossenDynamics.m`).

---

## 2. Persepsi & Sensor Fusion (Sonar + Kamera + EKF)
- **Forward-Looking Acoustic Sonar (`SonarModel.m`):** Mensimulasikan sapuan akustik 3D dengan cek *line-of-sight occlusion* terhadap obstacle dan noise jarak/sudut.
- **Optical Camera Detector (`CameraDetector.m` & `camera_target_detector.py`):** Segmentasi HSV warna biru target + filter kontur OpenCV untuk estimasi posisi relatif dalam jarak pandang air keruh.
- **3D Extended Kalman Filter (`EKFTargetTracker.m` & `sonar_target_tracker.cpp`):** State 6-dimensi $[x, y, z, v_x, v_y, v_z]^T$ yang melakukan prediksi *constant-velocity* saat target terhalang obstacle dan *measurement update* saat echo sonar atau deteksi kamera masuk.

---

## 3. Coverage Grid & 3D A* Path Planning
- **Coverage Grid (`CoverageGrid.m`):** Mencatat sel arena yang sudah dipindai oleh Seeker dan mengarahkan Seeker ke sel belum terjelajah terdekat saat fase `SEARCHING` / `LOST`.
- **3D A* Planner (`AStarPlanner3D.m`):** Membangun occupancy grid 3D dari obstacle arena (Coral Lighthouse, Shipwreck, Reef Pillars) dan mencari jalur bebas tabrakan.
- **Artificial Potential Field (APF):** Memberikan gaya repulsif reaktif dari echo sonar jarak dekat agar AUV tidak menabrak rintangan dinamis maupun statis.

---

## 4. State Machine Hide and Seek
State machine lengkap sesuai spesifikasi:
1. `IDLE` — Menunggu fase *headstart* Hider selesai.
2. `SEARCHING` — Menjelajahi sel `CoverageGrid` yang belum terperiksa menggunakan 3D A*.
3. `DETECTED` — Target terdeteksi di jarak jauh (> 18 m); mulai mendekat.
4. `APPROACHING` — Pengejaran *lead-pursuit* menggunakan estimasi kecepatan EKF.
5. `LOST` — Kehilangan kontak sensor sementara; mempertahankan prediksi EKF sebelum kembali ke `SEARCHING`.
6. `FOUND` — Jarak $\le 3.0\text{ m}$ tercapai (Seeker menang).
