#  Autonomous AI-Powered Pothole Detection & Road Mapping Rover

**A low-cost, autonomous robotic vehicle capable of detecting road anomalies in real-time using Edge AI, geotagging hazards with GPS, and navigating dynamic environments using Sensor Fusion.**

---

##  Project Overview

Road maintenance is often reactive rather than proactive due to a lack of real-time data. This project solves that problem by deploying an autonomous rover that:

1. **Patrols** roads independently.
2. **Detects** potholes using a custom-trained **YOLOv8** Neural Network.
3. **Logs** the exact GPS coordinates of every hazard.
4. **Avoids** obstacles (walls, cliffs) using a hybrid sensor suite.

---

##  System Architecture

The robot operates on a **Master-Slave Architecture**:

* **Master (Brain):** NVIDIA Jetson Nano / Laptop running Python & YOLOv8. Handles Computer Vision and decision-making.
* **Slave (Body):** Arduino Uno. Handles motor control and reflex safety sensors.
* **Comms (Telemetry):** ESP32. Manages GPS data and Bluetooth communication with the mobile dashboard.

---

##  Key Features

* **Edge AI Vision:** Real-time pothole detection at 30 FPS using TensorRT-optimized YOLOv8.
* **Sensor Fusion Safety:** Combines Visual data with **Ultrasonic** (Obstacle Avoidance) and **IR** (Cliff Detection) to prevent crashes.
* **Precision Geotagging:** Logs `Latitude`, `Longitude`, and `G-Force` impact data to a CSV file for every detected hazard.
* **Return-to-Home (RTH):** Autonomous navigation back to the starting GPS coordinate upon mission completion.
* **Real-Time Dashboard:** Live status updates via Bluetooth to a mobile app.

---

##  Model Performance

The AI model was trained on a custom dataset of 2,700+ annotated road images.

| Metric | Score | Notes |
| --- | --- | --- |
| **mAP@50** | 0.55 | Consistent with benchmarks for irregular/amorphous objects. |

> **Note:** While the strict geometric mAP score is 0.55 due to the irregular shapes of road cracks, the functional

---

##  Hardware Requirements

* **Compute:** NVIDIA Jetson Nano (4GB) OR Laptop (for prototype)
* **Microcontroller:** Arduino Uno + ESP32
* **Sensors:**
* Neo-6M GPS Module
* MPU6050 (Gyroscope/Accelerometer)
* 4x HC-SR04 Ultrasonic Sensors
* IR Proximity Sensor (Cliff Detection)
* USB Webcam (Logitech C270 or similar)


* **Drive:** L298N Motor Driver + 4x DC Motors + 12V Li-Po Battery

---

## 💻 Installation & Setup

1. **Clone the Repository**
```bash
git clone https://github.com/YourUsername/Pothole-Rover.git
cd Pothole-Rover

```


2. **Install Dependencies**
It is recommended to use a virtual environment.
```bash
pip install -r requirements.txt

```


3. **Hardware Connection**
* Connect Arduino to USB Port `COM3` (or `/dev/ttyACM0`).
* Connect ESP32e to USB Port `COM5` (or `/dev/ttyACM1`).
* Ensure Camera is connected to Index `0`.


4. **Run the Autonomous Script**
```bash
python robot_main.py

```



---

##  Repository Structure

```text
Pothole-Detection-YOLOv8/
│
├── README.md               
├── requirements.txt        <-- List of libraries (ultralytics, opencv, etc.)
├── robot_main.py           <-- The final "Ultimate" Python script for the robot
├── visual_test.py          <-- The script you used for Manual Validation
│
├── weights/
│   └── best.pt             <-- Trained AI Brain 
│
├── config/
│   └── data.yaml           <-- The config file used for training
│
├── results/                
│   ├── confusion_matrix.png
│   ├── F1_curve.png
│   ├── PR_curve.png
│   └── val_batch0_pred.jpg 
│
└── samples/                
    ├── pothole_1.jpg
    └── pothole_2.jpg
```
---

##  Future Scope

* **LiDAR Integration:** For 3D volumetric analysis of potholes to estimate repair material costs.
* **Swarm Deployment:** Implementing centralized cloud mapping where multiple rovers update a single city map.
* **Solar Charging:** Adding solar panels for extended patrol durations.

---

## Team

* **Atharv Aggarwal** 
* **Tushar Vats**
* **Vaibhav Rawat**
* **Jagrav Raj**

---
