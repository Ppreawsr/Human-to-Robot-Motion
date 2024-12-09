# Transformation of Human Movement to Robot Motion
This project focuses on simulating a robotic arm's movement based on human arm motion tracked by a camera. MediaPipe detects motion and sends joint data to a MATLAB/SIMULINK model. Inverse Kinematics calculates joint angles, verified by Forward Kinematics. Trajectory planning and velocity control improve movement efficiency, with results showing joint positions, angles, and velocities, along with a visual movement display.

**This project is a part of FRA333 Kinematics of Robotics System, Institute of Field Robotics, King Mongkut’s University of Technology Thonburi**
## Overview
### Transformation of Human Movement to Robot Motion Demo
// demo picture 
### System Diagram / System Overview (Function and Argument)
// diagram picture
### Objectives
- To study the transformation of human arm movements into the simulated movement of a robotic arm model.
- To explore the use of MediaPipe and MATLAB software.
### Project Scope  
A. Input from Human Arm Motion Capture  
- MediaPipe reads the joint positions of only one arm.  
- Input is based solely on position, without considering velocity or acceleration in the actual system.  

B. Number of Joints in the Robotic Arm Model  
- The model consists of the shoulder, elbow, wrist, and the base of the middle finger only.  
- The simulated model has no more than 3 degrees of freedom (DOF).  

C. Visualization  
- Visualize the model as static images for display purposes.  
- Real-time movement is not required for visualization.  
### Description
- **Inverse Kinematics**
  - Used to transform MediaPipe input into configuration space for robotic systems, enabling accurate mapping of human motion to robotic actuators.
- **Trajectory Planning**
  - Focused on controlling the movement of robotic systems to follow a predefined path, ensuring efficiency and accuracy.
- **Robot Modeling**
  - Used to create simulation models for testing purposes, enabling iterative improvements and robust system design.
- **Forward Kinematics**
  - Applied to calculate the end-effector position and validate the model, especially since MediaPipe inputs with varying scales cannot be directly compared.
- **Inverse Dynamic**
  - Calculates the torque and forces required for motion to adhere to trajectory planning, ensuring physical feasibility and system stability.
## Table of Contents

- [Overview](#overview)
   * [Transformation of Human Movement to Robot Motion Demo](#transformationofhumanmovementtorobotmotion)
   * [Description](#description)
- [Installation](#installation)
- [Methodology](#methodology)
   * [Image Capture](#imagecapture)
   * [Calculation](#calculation)
	   * [Motion Capture](#motioncapture)
           * [Inverse Kinematics](#inversekinematics)
	   * [Trajectory Planning](#trajectoryplanning)
	   * [Motion Control](#motioncontrol)
	   * [Inverse Dynamics](#inversedynamics)
	   * [Robot Modeling](#robotmodeling)
	   * [Forward Kinematics](#forwardkinematics)
   * [ 3D Visualization](#3dvisualization)
   * [ System Architecture Diagram](#systemarchitecturediagram)
- [User Guide](#userguide)
- [Demo & Result](#demo&result)
- [Conclusion](#conclusion)
- [References](#references)
---
## Installation
 Use `pip` to install the libraries. Open a terminal and run the following commands:
### OpenCV (cv2)
` pip install opencv-python`
### Mediapipe
`pip install mediapipe`
### NumPy
`pip install numpy`

---
##  Methodology
### Motion Capture (with MediaPipe)
This process includes capturing 2D-coordinate from the human arm, preprocessing with moving average, calculating into unit vectors and adjusting scale to match the robotic arm model.
1. Capturing arm position input (X, Y)

   In this step, we use the “MediaPipe” library to capture input arm’s coordinate joint positions include :
   - Reference frame (0,0) : Left shoulder
   - Joints : Left shoulder, elbow and wrist
   - End-effector :  Left hand
![Project-Kinematics (2)](https://github.com/user-attachments/assets/e4dbe111-d1a9-42ed-9fb0-b3f8d88edf35)
2. Processing

   input is then preprocessed with with moving average for noise reduction
![code-processing](https://github.com/user-attachments/assets/56a6c661-5855-413a-a399-e4dc2c27daa9)
3. Model Mapping

   Calculate unit vector and adjust input’s scale to Transform the abstract 2D model into a real-world robotic arm model
![Project-Kinematics (4)](https://github.com/user-attachments/assets/36080f6b-964e-48d3-aacd-a40583a910e0)
The newly created model map is now of the same scale as model robot, with human input’s unit vector (in reference to shoulder frame) 
4. Real time communication with Simulink

   The processed input is then sent to Matlab’s Simulink in real time using UDP every
![Project-Kinematics (3)](https://github.com/user-attachments/assets/35be6119-c24a-4762-a723-636cbeb84b7a)

---
### Calculation
![3R planar](https://github.com/user-attachments/assets/de0d3961-aefc-4d3b-b7aa-7cc0295f7bd9)

Variable Mapping and Meanings
-  The end-effector position. $$x,y = (xₑ,yₑ)$$ 
-   The orientation angle of the end-effector.
$$phi = γ$$ 
-   $L1, L2, L3 :$
    * Length of the first link : $L1 = L₁₂$ ​
    * Length of the second link : $L2 = L₂₃$
    * Length of the third link (to the end-effector) : $L3 = L₃₄$
-  Coordinates of the wrist point : $xw,  yw= (x₃,y₃)$
-   First joint angle : `theta1` = $θ₁$ 
-   Second joint angle : `theta2` = $θ₂$
-   Third joint angle (orientation) : `theta3`= $θ₃$


Singularity is also calculated and excluded to prevent error in this calculation as there are 2 main conditions

Condition 1 : det(J) = 0
- For a 3-DOF planar robot with link lengths L1, L2, L3​ and joint angles θ1, θ2, θ3 
the Jacobian matrix is:
J = 
$$
J = 
\begin{bmatrix} 
-\sin(\theta_1)(L_1 + L_2\cos(\theta_2)) - \sin(\theta_1 + \theta_2)L_2 & -\sin(\theta_1 + \theta_2)L_2 & 0 \\ 
\cos(\theta_1)(L_1 + L_2\cos(\theta_2)) + \cos(\theta_1 + \theta_2)L_2 & \cos(\theta_1 + \theta_2)L_2 & 0 \\ 
0 & 0 & 1
\end{bmatrix}
$$

$$
\det(J) = L_1 L_2 \sin(\theta_2)
$$



- Meaning sin(θ2) can’t be equal 0

Condition 2 : Target out of reach
- This is simply calculated with total length of L1, L2 to see if wrist position stay within workspace

---
#### **Inverse Kinematics**
![2R_Planar_Manipulator](https://github.com/user-attachments/assets/ff68d60f-503d-445b-8ece-638ceb9dd275)

Determine the joint angles $(\theta_1, \theta_2, \theta_3 )$ given the end-effector position $(x, y)$ and orientation $(\phi)$

- Calculate the Wrist Position

    To calculate the wrist position $x_w, y_w$, subtract the contribution of the third link from the given end-effector position $(x, y)$:

$$x_w = x - L_3 \cos(\phi)$$
$$y_w = y - L_3 \sin(\phi)$$


- Calculate the Distance to the Wrist Point
   Find the distance $r$ between the origin (base of the manipulator) and the wrist point $x_w, y_w$:
$$r = \sqrt{x_w^2 + y_w^2}$$

- Check Reachability
  Verify if the target position is reachable by checking the following conditions:
If $r > L_1 + L_2$, the target is outside the reachable workspace.
 If $r < |L_1 - L_2|$, the target is too close and also unreachable.

-  Calculate $\theta_2$
   Determine the second joint angle $\theta_2$ using the cosine rule for the triangle formed by the base, wrist point, and second joint.

$$
\cos(\theta_2) = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

$$
\sin(\theta_2) = \sqrt{1 - \cos^2(\theta_2)}
$$

$$
\theta_2 = \tan^{-1}\left(\frac{\sin(\theta_2)}{\cos(\theta_2)}\right)
$$

-  Calculate $\theta_1$
To calculate the first joint angle  $\theta_1$, determine the angles $( \alpha )$ and $( \beta )$
$$\alpha = \tan^{-1}\left(\frac{y_w}{x_w}\right)$$
$$\beta = \tan^{-1}\left(\frac{L_2 \sin(\theta_2)}{L_1 + L_2 \cos(\theta_2)}\right)$$
$$\theta_1 = \alpha - \beta$$
- Calculate $\theta_3$
Finally, calculate the third joint angle $\theta_3$ as the orientation adjustment needed to align the end-effector with $(\phi)$.
$$\theta_3 = \phi - (\theta_1 + \theta_2)$$
- Handle Singularities
Check for singularities to ensure valid calculations
   - If $\sin(\theta_2) = 0$, the arm is fully extended or fully folded.
Ensure all computed angles are within valid ranges.
- Summary
$$\theta_1 = \alpha - \beta$$
$$\theta_2 = \tan^{-1}\left(\frac{\sin(\theta_2)}{\cos(\theta_2)}\right)$$
$$\theta_3 = \phi - (\theta_1 + \theta_2)$$

---
#### **Forward Kinematics**
	
When there is a movement of the human arm with specified coordinates (x, y), the model calculates the Inverse Kinematics using the (x, y) coordinates of the human arm. The result is q1, q2, q3. The model uses the q values of the human arm to control the movement of a 3DOF robotic arm. The 3DOF robotic arm then sends its q1, q2, q3 values back to be calculated in Forward Kinematics to verify whether the movement patterns of the human arm and the robotic arm match. The Forward Kinematics calculation produces (x, y) coordinates of the robotic arm, which are then compared with the (x, y) coordinates of the human arm movement. If the coordinates match, it indicates that the movement is correct.
- Calculate the Forward Kinematics to obtain the (x, y) coordinates of the robotic arm
![Forward](https://github.com/user-attachments/assets/e4b13c52-8cec-4dba-a7fd-121b637b7c27)

- Matlab 

![matlab](https://github.com/user-attachments/assets/6960d93f-59c5-41af-97f4-38ae74550ef0)
- Simulink
![simulink](https://github.com/user-attachments/assets/190d1676-65c6-413d-96e0-af15996aae74)

---

#### **Trajectory Planning**
**Find maximum acceleration**
1. Calculate the Moments of Inertia
         Each link contributes to the total moment of inertia $(I)$ of the system, which is a key factor in determining the maximum torque and, consequently, the acceleration. 
For a uniform rod rotating about one end: 
$$ I = \frac{1}{3} m L^2 $$ 
 - Link Parameters: 
   - Link 1: $(m_1 = 1 \, \text{kg}, L_1 = 5 \, \text{m})$ 
   - Link 2: $(m_2 = 1 \, \text{kg}, L_2 = 4 \, \text{m})$
   - Link 3: $(m_3 = 1 \, \text{kg}, L_3 = 1 \, \text{m})$ 
   Total Moment of Inertia
$$ I_{\text{total}} = I_1 + I_2 + I_3 $$ 
Substitute $I_i = \frac{1}{3} m_i L_i^2$
 $$ I_1 = \frac{1}{3}(1)(5^2) = \frac{25}{3} \, \text{kg ⋅ m}^2 $$ 
 $$I_2 = \frac{1}{3}(1)(4^2) = \frac{16}{3} \, \text{kg⋅m}^2 $$ 
 $$I_3 = \frac{1}{3}(1)(1^2) = \frac{1}{3} \, \text{kg⋅m}^2  $$
 $$ I_{\text{total}} = \frac{25}{3} + \frac{16}{3} + \frac{1}{3} = 14 \, \text{kg⋅m}^2 $$
2. Maximum Torque
Assume the motors can apply a total torque $T_{\text{max}}$ on the system. If the motors can apply $T_{\text{max}} = 50 \, \text{Nm}$, we can estimate $(a_{\text{max}})$. 
Using the rotational analog of Newton's second law 
$$ \alpha = \frac{T}{I}$$
 $$ a_{\text{max}} = \alpha \cdot L_{\text{eff}} $$
  Where $(L_{\text{eff}})$ is the effective length of the links (weighted average of lengths)
  $$ L_{\text{eff}} = \frac{\sum m_i L_i}{\sum m_i} = \frac{1 \cdot 5 + 1 \cdot 4 + 1 \cdot 1}{3} = 3.33 \, \text{m} $$ 
  Substitute values: 
  $$\alpha = \frac{50}{14} \approx 3.57 \, \text{rad/s}^2 $$ 
  $$a_{\text{max}} = \alpha \cdot L_{\text{eff}} \approx 3.57 \cdot 3.33 \approx 11.9 \, \text{m/s}^2 $$ 
  3. Maximum Velocity
  The maximum velocity $(v_{\text{max}})$ depends on
   *  Link lengths: Longer links result in higher end-effector velocities for the same joint velocities. 
   *  Practical constraints (e.g., motor limits, safety). 
  Assume the robot moves through its maximum angular displacement $( \theta_{\text{max}} = 90^\circ = \frac{\pi}{2} \, \text{rad} )$ in 1 second:
  $$v_{\text{max}} = \omega_{\text{max}} \cdot L_{\text{eff}}$$ 
  With $\omega_{\text{max}} = \frac{\pi}{2} \, \text{rad/s}:$
  
  $$v_{\text{max}} = \frac{\pi}{2} \cdot 3.33 \approx 5.23 \, \text{m/s} ]$$ 
    4. Final Recommendations
-  Maximum Acceleration: $a_{\text{max}} = 10 \, \text{m/s}^2 )$ (rounding for simplicity).  
 - Maximum Velocity: $v_{\text{max}} = 5 \, \text{m/s}$.
 
 **Find Velocity**
 1. Average Linear Velocity
          The linear speed of a human hand (end effector) during daily activities typically ranges between:
 - $1.5 \, \text{m/s}$ (e.g. reaching for an object).
 - $5 \, \text{m/s}$ (e.g. rapid hand movements like throwing).

> Reference:  Fast human arm movements, such as a tennis serve, can reach up to 20 m/s, but this is an extreme case.

2. Angular Velocity of Human Joints
- Shoulder Joint: $2 \, \text{rad/s}$ – $5 \, \text{rad/s}$
- Elbow Joint: $2 \, \text{rad/s}$ – $6 \, \text{rad/s}$
 - Wrist Joint: $1 \, \text{rad/s} - 4 \, \text{rad/s}$


---

#### **Adjusting Robot’s End-Effector Speed**
To make the robot’s speed more human-like, we can scale down the maximum angular velocity (\( \omega_{\text{max}} \)) or adjust the link lengths.

1. Adjust Angular Velocity $\omega_{\text{max}}$
We aim for the linear velocity of the end effector to fall between 1.5 m/s and 5 m/s.

    Using the formula:
$$v_{\text{linear}} = \omega \cdot L$$
   For the longest link $L = 5 \, \text{m}$:
$$\omega_{\text{max}} = \frac{v_{\text{linear}}}{L}$$
   If $v_{\text{linear}} \approx 2.5 \, \text{m/s}$
$$\omega_{\text{max}} = \frac{2.5}{5} = 0.5 \, \text{rad/s}.$$

2. Adjust Acceleration $\alpha_{\text{max}}$
For human-like smoothness, angular acceleration values should be around:
- Shoulder: $2 \, \text{rad/s}^2 - 4 \, \text{rad/s}^2$
- Elbow/Wrist: $3 \, \text{rad/s}^2 - 6 \, \text{rad/s}^2$

We can set $\alpha_{\text{max}} = 3 \, \text{rad/s}^2$

3. Adjust Jerk $( j_{\text{max}} )$
Human movements are typically jerk-limited to minimize discomfort or instability:
- Set $j_{\text{max}} = 20 \, \text{rad/s}^3$


#### **Revised Linear Velocity**
Using $omega_{\text{max}} = 0.5 \, \text{rad/s}$
$$v_{\text{linear}} = 0.5 \cdot 5 = 2.5 \, \text{m/s}$$

This is well within the range of human-like motion.

---

#### **Updated README Calculations**

1. Moment of Inertia
$$I_{\text{total}} = 14 \, \text{kg} \cdot \text{m}^2$$

2. **Maximum Angular Acceleration**:
$$\alpha_{\text{max}} = 3 \, \text{rad/s}^2$$

3. **Maximum Angular Velocity**:
$$\omega_{\text{max}} = 0.5 \, \text{rad/s}$$

4. **Maximum End-Effector Velocity**:
$$v_{\text{linear}} = \omega_{\text{max}} \cdot L = 0.5 \cdot 5 = 2.5 \, \text{m/s}$$

 ---
#### **Robot Modeling**

---

#### **Dynamic**

----
### 3D Visualization

---


## User Guide



## Demo & Result
### Example
### Validation

## Conclusion

---
## References
[1] Altayeb, Muneera. (2023). Hand Gestures Replicating Robot Arm based on MediaPipe. Indonesian Journal of Electrical Engineering and Informatics (IJEEI), 11(3), 727–737. https://doi.org/10.52549/ijeei.v11i3.4491

[2] Wang Ziwei  1, Rongjian Liang  2, Zhang Chen  3, Bin Liang  4. (2020). Fast and Intuitive Kinematics Mapping for Haman-Robot Motion Imitating: A Virtual-Joint-Based Approach. IFAC PapersOnLine, 53(2), 10011–10018. https://doi.org/10.1016/j.ifacol.2020.12.2720

[3] Egemen Aksoy  1, Arif Dorukan Çakır  2, Berat Alper Erol  3, Abdurrahman Gumus  4. Real Time Computer Vision Based Robotic Arm Controller with ROS and Gazebo Simulation Environment. https://doi.org/10.XXXX/elec.2023
