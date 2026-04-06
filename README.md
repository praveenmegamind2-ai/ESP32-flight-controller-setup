# ESP32-flight-controller-setup

## 📌 Overview
Custom-built flight controller using an ESP32 microcontroller for quadcopter/VTOL stabilization. The system implements real-time PID control, sensor fusion, and a live telemetry dashboard.

Flightcontroller built using an ESP32 microcontroller. Built an HTML system visualizer, Performed sensor fusion using Kalman filters. All yaw, pitch and roll performed via a MPU6050 gyro and accel module. PID tuning was performed manually by checking the exact parameters P and D where the oscillating point occurs for roll, pitch and yaw. I gain was added later by observing any potentially unwanted drift.

