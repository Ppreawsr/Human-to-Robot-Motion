# Transformation of Human Movement to Robot Motion
This project focuses on simulating a robotic arm's movement based on human arm motion tracked by a camera. MediaPipe detects motion and sends joint data to a MATLAB/SIMULINK model. Inverse Kinematics calculates joint angles, verified by Forward Kinematics. Trajectory Planning anvelocityty Control improve movement efficiency, with results showing joint positions, angles, and velocities, along with a visual movement display.
## Overview
### Transformation of Human Movement to Robot Motion Demo
### Description
- **Mediapipe**
- **Inverse Kinematics**
- **Forward Kinematics**
- **Differential Kinematics**
- **Trajectory Planning**
- **Robot Modeling**
- **Dynamic**
## Table of Contents

- [Overview](#overview)
   * [Transformation of Human Movement to Robot Motion Demo](#transformationofhumanmovementtorobotmotion)
   * [Description](#description)
- [Installation](#installation)
- [Methodology](#methodology)
   * [Image Capture](#imagecapture)
   * [Dynamic Calculation](#dynamiccalculation)
	   * [Inverse Kinematics](#dynamiccalculation)
	   * [Forward Kinematics](#dynamiccalculation)
	   * [Differential Kinematics](#dynamiccalculation)
	   * [Trajectory Planning](#dynamiccalculation)
	   * [Robot Modeling](#dynamiccalculation)
	   * [Dynamic](#dynamiccalculation)
   * [ 3D Visualization](#3dvisualization)
   * [ System Architecture Diagram](#systemarchitecturediagram)
- [User Guide](#userguide)
- [Demo & Result](#demo&result)
- [Conclusion](#conclusion)
- [References](#references)

## Installation
 Use `pip` to install the libraries. Open a terminal and run the following commands:
### OpenCV (cv2)
` pip install opencv-python`
### Mediapipe
`pip install mediapipe`
### NumPy
`pip install numpy`
##  Methodology
### Image Capture
### Dynamic Calculation

 Variable Mapping and Meanings
-   `x,y` = `(xₑ,yₑ)` - The end-effector position.
-   `phi `= `γ` - The orientation angle of the end-effector.
-   `L1`, `L2`, `L3`:
-   `L1` = `L₁₂` ​- Length of the first link.
-   `L2`= `L₂₃` - Length of the second link.
-   `L3`= `L₃₄` - Length of the third link (to the end-effector).
-   `xw`, `yw`= `(x₃,y₃)` - Coordinates of the wrist point.
-   `theta1` = `θ₁` - First joint angle.
-   `theta2` = `θ₂` - Second joint angle.
-   `theta3`= `θ₃​` - Third joint angle (orientation)
- **Inverse Kinematics**
   * **Calculate the Wrist Position** 


- **Forward Kinematics**
- **Differential Kinematics**
- **Trajectory Planning**
- **Robot Modeling**
- **Dynamic**
### 3D Visualization
### System Architecture Diagram

## User Guide

## Demo & Result
### Example
### Validation

## Conclusion

## References
[1] Altayeb, Muneera. (2023). Hand Gestures Replicating Robot Arm based on MediaPipe. Indonesian Journal of Electrical Engineering and Informatics (IJEEI), 11(3), 727–737. https://doi.org/10.52549/ijeei.v11i3.4491

[2] Wang Ziwei  1, Rongjian Liang  2, Zhang Chen  3, Bin Liang  4. (2020). Fast and Intuitive Kinematics Mapping for Haman-Robot Motion Imitating: A Virtual-Joint-Based Approach. IFAC PapersOnLine, 53(2), 10011–10018. https://doi.org/10.1016/j.ifacol.2020.12.2720

[3] Egemen Aksoy  1, Arif Dorukan Çakır  2, Berat Alper Erol  3, Abdurrahman Gumus  4. Real Time Computer Vision Based Robotic Arm Controller with ROS and Gazebo Simulation Environment. https://doi.org/10.XXXX/elec.2023
