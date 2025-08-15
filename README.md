# T.R.A.S.H. — Trackable Recyclable Automated Sorting Hand  
**University of Central Florida – Group 7 Senior Design Project**

Missing:
- urdf file
- Arm Design, EE and possibly full design
- video
- website

This repository contains the ROS 2 Jazzy package developed for the **T.R.A.S.H.** project. The goal of this project was to design an autonomous robotic system capable of identifying, sorting, and categorizing recyclable materials using computer vision, embedded control, and spectrometry.

All scripts are written in **Python using `rclpy`** and were tested on **Ubuntu 24.04**. The project integrates multiple subsystems including camera vision, robotic control, GUI feedback, and real-time material classification.

---

## Repository Structure

Before running the ROS 2 nodes, the `motor_controller.ino` firmware must be uploaded to the ESP32.
This code handles all low-level motor actuation and is required for any robotic movement.
- Uploaded via Arduino IDE to an ESP32-S3
- Connected via GPIO pins
- Currently has pre-determined movements, beucase of lack of simulation testing with IK


This ROS 2 package `trash_project/trash_project` contains 8 primary nodes:

### 1. `basic_webcam_node.py`
- Verifies that the USB camera is recognized by the system.
- Functions similarly to `libcamera-hello` for simple camera validation.

### 2. `basic_cv_node.py`
- Loads a trained YOLO model from the `models/` directory.
- Captures camera frames and runs inference.
- Publishes detection direction to the `cv_direction` topic.

### 3. `basic_motor_node.py`
- Establishes UART serial communication with the ESP32 motor controller.
- Uses a USB-to-TTL adapter and confirms connection stability.
- Does not account for switching between `/dev/ttyUSB0` and `/dev/ttyUSB1`.

---

## Main Functional Nodes

### 4. `cv_node.py`
- Performs real-time object detection using YOLO.
- Tracks object center location and assigns a detection zone (left, center, right).
- Sends commands to guide pick-and-place operations.

### 5. `canny_node.py`
- Main computer vision node combining YOLO and Canny edge detection.
- If YOLO’s confidence is below 80%, the object is flagged as "unknown."
- Triggers the spectrometer for material confirmation in low-confidence scenarios.

### 6. `motor_node.py`
- Controls robot motion via a finite state machine (FSM).
- Sends motor angle commands to the ESP32 over UART based on detection inputs.
- Manages state transitions for pick, analyze, and bin placement sequences.
- Logs current joint angles and robot state throughout the process.

### 7. `spectrometer_node.py`
- Communicates with an STM32-based spectrometer over serial.
- Reads reflectance spectra and classifies materials as paper, plastic, or metal.
- Delays reading to allow the object to be fully centered before capture.
- Publishes classification data to inform sorting decisions.

### 8. `gui_node.py`
- Provides a live interface to monitor and control the robot.
- Displays object detection output, CV and spectrometer results, motor angles, and FSM state.
- Allows manual joint control and includes an emergency stop feature.

---

## YOLO Models Used

This project utilizes two versions of the YOLO (You Only Look Once) object detection framework for item recognition:

### YOLOv8
- Used for early model testing and baseline performance.
- Compatible with `best_v7.pt`.
- Integrated into the pipeline to verify item position and guide robotic motion.

### YOLOv12
- Used for improved detection performance in the final system.
- Compatible with `best_v8.pt` and `best_v9.pt`.
- Provides more accurate detection and better support for detecting recyclable objects under varied lighting and occlusion.

Both versions are loaded using **Ultralytics' PyTorch-based interface** and integrated into `cv_node.py` and `canny_node.py` depending on the configuration.

> Note: All models are stored in the `models/` directory and are loaded based on which `.pt` file is selected during runtime.

---

## Virtual Environment

To ensure reproducibility, a list of dependencies is included in: trash_venv_requirements.txt

### Setup Instructions:
```bash
python3 -m venv trash_env
source trash_env/bin/activate
pip install -r trash_venv_requirements.txt
```
## Hardware Components

- **Camera**: Arducam 4K 8MP IMX219 Autofocus USB Camera  
- **Main Processor**: Raspberry Pi 5 (8GB RAM)  
- **Motor Controller**: ESP32-S3-WROOM-1-N8  
- **Spectrometer MCU**: STM32F401  
- **Storage**: 64GB MicroSD  
- **Serial Interface**: USB to TTL (5-pin) Adapter Module  



