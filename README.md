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
\documentclass{article}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage{geometry}

\geometry{a4paper, margin=1in}

\title{3R Planar Manipulator - Step-by-Step Inverse Kinematics}
\author{}
\date{}

\begin{document}

\maketitle

\section*{Introduction}
This document explains the step-by-step process of calculating inverse kinematics for a 3R planar manipulator. The goal is to determine the joint angles (\( \theta_1, \theta_2, \theta_3 \)) given the end-effector position (\(x, y\)) and orientation (\(\phi\)).

\section*{Step 1: Calculate the Wrist Position}
To calculate the wrist position (\(x_w, y_w\)), subtract the contribution of the third link from the given end-effector position (\(x, y\)):

\[
x_w = x - L_3 \cos(\phi)
\]

\[
y_w = y - L_3 \sin(\phi)
\]

\section*{Step 2: Calculate the Distance to the Wrist Point}
Find the distance (\(r\)) between the origin (base of the manipulator) and the wrist point (\(x_w, y_w\)):

\[
r = \sqrt{x_w^2 + y_w^2}
\]

\section*{Step 3: Check Reachability}
Verify if the target position is reachable by checking the following conditions:
\begin{itemize}
    \item If \( r > L_1 + L_2 \), the target is outside the reachable workspace.
    \item If \( r < |L_1 - L_2| \), the target is too close and also unreachable.
\end{itemize}

\section*{Step 4: Calculate \( \theta_2 \)}
Determine the second joint angle (\( \theta_2 \)) using the cosine rule for the triangle formed by the base, wrist point, and second joint:

\[
\cos(\theta_2) = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}
\]

\[
\sin(\theta_2) = \sqrt{1 - \cos^2(\theta_2)}
\]

\[
\theta_2 = \tan^{-1}\left(\frac{\sin(\theta_2)}{\cos(\theta_2)}\right)
\]

\section*{Step 5: Calculate \( \theta_1 \)}
To calculate the first joint angle (\( \theta_1 \)), determine the angles \( \alpha \) and \( \beta \):

\[
\alpha = \tan^{-1}\left(\frac{y_w}{x_w}\right)
\]

\[
\beta = \tan^{-1}\left(\frac{L_2 \sin(\theta_2)}{L_1 + L_2 \cos(\theta_2)}\right)
\]

\[
\theta_1 = \alpha - \beta
\]

\section*{Step 6: Calculate \( \theta_3 \)}
Finally, calculate the third joint angle (\( \theta_3 \)) as the orientation adjustment needed to align the end-effector with \(\phi\):

\[
\theta_3 = \phi - (\theta_1 + \theta_2)
\]

\section*{Step 7: Handle Singularities}
Check for singularities to ensure valid calculations:
\begin{itemize}
    \item If \( \sin(\theta_2) = 0 \), the arm is fully extended or fully folded.
    \item Ensure all computed angles are within valid ranges.
\end{itemize}

\end{document}


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
