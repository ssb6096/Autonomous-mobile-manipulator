# Autonomous Mobile Manipulator

A mobile robot that maps its surroundings with a depth camera and picks and places objects with an onboard robot arm. It combines an **AmigoBot** mobile base, an **ASUS Xtion** RGB-D camera, **ORB-SLAM2** in ROS, and an **AL5B** arm driven by inverse kinematics in MATLAB.

[![Watch the demo on YouTube](https://img.youtube.com/vi/RUvS4jGKha0/hqdefault.jpg)](https://www.youtube.com/watch?v=RUvS4jGKha0)

▶️ **[Watch the demo on YouTube](https://www.youtube.com/watch?v=RUvS4jGKha0)** · 📑 [Project slides](docs/Autonomous_Mobile_Manipulator_Slides.pptx)

---

## Overview

- **Perception.** The AmigoBot is integrated with an Xtion camera, controlled through **OpenNI2**, to capture RGB and depth images of the environment. Raw RGB images are processed with **OpenCV** and cropped to the required resolution.
- **Mapping and localization.** The depth stream feeds the **ORB-SLAM2** package in **ROS** to build a 3D map. The point cloud is published and frames are processed continuously for localization and re-mapping of the environment.
- **Manipulation.** An **AL5B** arm mounted on the AmigoBot performs pick-and-place. Joint angles come from **inverse kinematics** (Denavit–Hartenberg parameters) implemented in **MATLAB**, and the servos are commanded over serial through an SSC-32 controller.

## Skills and tools

`ROS` · `ORB-SLAM2` · `OpenNI2` · `OpenCV` · `MATLAB` · `Python` · `Inverse kinematics` · `SLAM` · `RGB-D perception`

## Repository contents

| Path | What it is |
|---|---|
| `project al5b arm 11-20-18/` | MATLAB control code for the AL5B arm: `ArmRobot.m`, `FrameTransformation.m`, serial/SSC-32 drivers (`SerialPort.m`, `SSC32.m`), and lab scripts |
| `dh.zip`, `dh_parameters.zip` | Denavit–Hartenberg parameter scripts for the arm's kinematics |
| `ik6dof/` | Inverse-kinematics solver for a 6-DOF arm (third-party, © Andrea Cirillo, BSD license) |
| `DesigningRobotManipulatorAlgorithms/` | MathWorks "Robot Manipulator Control" example used as a reference (© The MathWorks, BSD license) |
| `IK/` | Python 3.7 virtual environment used during development |
| `docs/` | Project slides |

## Context

Team project from my M.S. in Electrical Engineering at Rochester Institute of Technology. Also on [Portfolium](https://portfolium.com/entry/autonomous-mobile-manipulator).

## License

This project is released under the [PolyForm Noncommercial License 1.0.0](LICENSE). You may use, modify and share it for **noncommercial purposes**, including academic research, teaching and personal study. Commercial use needs separate permission from the author.

Required Notice: Copyright (c) 2020 Sriparvathi Shaji Bhattathiri

Third-party code in this repository keeps its original license. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
