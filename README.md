

# Transformation of Human Movement to Robot Motion
This project focuses on simulating a robotic arm's movement based on human arm motion tracked by a camera. MediaPipe detects motion and sends joint data to a MATLAB/SIMULINK model. Inverse Kinematics calculates joint angles, verified by Forward Kinematics. Trajectory planning and velocity control improve movement efficiency, with results showing joint positions, angles, and velocities, along with a visual movement display.

**This project is a part of FRA333 Kinematics of Robotics System, Institute of Field Robotics, King Mongkut’s University of Technology Thonburi**

## Table of Contents

- [Overview](#overview)
   * [Transformation of Human Movement to Robot Motion Demo](#transformation-of-human-movement-to-robot-motion)
   * [Description](#description)
- [Installation](#installation)
- [Methodology](#methodology)
   * [Image Capture](#image-capture)
   * [Calculation](#calculation)
   * [Motion Capture](#motion-capture)
	   * [Inverse Kinematics](#inverse-kinematics)
	   * [Trajectory Planning](#trajectory-planning)
	   * [Motion Control](#motion-control)
	   * [Inverse Dynamics](#inverse-dynamics)
	   * [Robot Modeling](#robot-modeling)
	   * [Forward Kinematics](#forward-kinematics)
   * [ 3D Visualization](#3d-visualization)
- [User Guide](#user-guide)
- [Conclusion](#conclusion)
- [References](#references)
- 
## Overview
### Transformation of Human Movement to Robot Motion Demo

https://github.com/user-attachments/assets/0d501b27-6180-4bc1-8208-0573c5746edb

### System Diagram / System Overview (Function and Argument)
![System_Diagram_Kinematic_Project_2](https://github.com/user-attachments/assets/5d19a23b-62d3-466f-ad21-9005602aa3e5)

### Objectives
- To study the transformation of human arm movements into the simulated movement of a robotic arm model.
- To explore the use of MediaPipe and MATLAB software.
### Project Scope  
A. Input from Human Arm Motion Capture  
- MediaPipe reads the joint positions of only one arm.  
- Input is based solely on position, without considering velocity or acceleration in the actual system.  

B. Number of Joints in the Robotic Arm Model  
- The model consists of the shoulder, elbow, wrist, and the base of the middle finger only.  
- The simulated model has no more than 3 degrees of freedom. (DOF)

C. Visualization  
- Visualize the model as static images for display purposes.  
- Real-time movement is not required for visualization.
### Requirement

1. MediaPipe Integration:
   - Extract joint positions of a single arm in real-time using MediaPipe.
   - Use only positional data, ignoring velocity or acceleration in the input system.
   - Map human arm motion to corresponding robotic arm joint configurations.

2. Inverse Kinematics:
   - Convert task-space coordinates (e.g., end-effector position) into joint space variables (q_1, q_2, q_3).
   - Handle multiple solutions and singularities in joint space, ensuring stability.

3. Trajectory Planning:
   - Generate smooth and continuous motion trajectories for joint angles (q_1, q_2, q_3).
   - Enforce physical constraints such as maximum velocity, acceleration, and jerk limits.

4. Robot Modeling:
   - Create a 3-DOF planar robotic arm model for simulation and control.
   - Include realistic parameters such as link lengths, masses, and joint configurations.

5. Forward Kinematics:
   - Transform the robot’s joint space variables (q_1, q_2, q_3) back into task-space coordinates (x, y, phi).
   - Compare task-space outputs with input data to calculate motion error.

6. Inverse Dynamics:
   - Calculate the required joint torques based on the arm's weight and dynamics.
   - Focus on extracting torque values for analysis without implementing control efforts.

### Progress
![Project-Kinematics (5)](https://github.com/user-attachments/assets/a9a86e5c-f1cd-41e3-aa6e-1093edf6c914)
### Description
- **Inverse Kinematics**
  - Used to transform MediaPipe input into configuration space for robotic systems, enabling accurate mapping of human motion to robotic actuators.
- **Trajectory Planning**
  - Focused on controlling the movement of robotic systems to follow a predefined path, ensuring efficiency and accuracy.
- **Robot Modeling**
  - Used to create simulation models for testing purposes, enabling iterative improvements and robust system design.
- **Forward Kinematics**
  - Applied to calculate the end-effector position and validate the model, especially since MediaPipe inputs with varying scales cannot be directly compared.
- **Inverse Dynamics**
  - Calculates the torque and forces required for motion to adhere to trajectory planning, ensuring physical feasibility and system stability.
---
## Installation
 Use `pip` to install the libraries. Open a terminal and run the following commands
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
### Inverse Kinematics
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

- Matlab Part
	Calculated inverse kinematic equation is applied to matlab function block 

![image](https://github.com/user-attachments/assets/2b8dec14-b8ba-4902-8353-2519da8ea5a2)

Input :
- End-effector position $(x,y)$
- End-effector orientation $(phi)$
- Elbow position $(x,y)$ : determining pose for multiple solution cases.
- Link Length $(L_1, L_2, L_3)$ : length of all 3 link of the model robot (5, 4 and 1 meters)

Output :
- Orientation : $q_1, q_2, q_3$


**Singularity** is also calculated and excluded to prevent error in this calculation as there are 2 main conditions


Condition 1 : $det(J) = 0$
- For a 3-DOF planar robot with link lengths L1, L2, L3​ and joint angles $θ1, θ2, θ3$
the Jacobian matrix is:

$$ J = \begin{bmatrix} \sin(\theta_1)(L_1 + L_2\cos(\theta_2)) - \sin(\theta_1 + \theta_2)L_2 & -\sin(\theta_1 + \theta_2)L_2 & 0 \\ \cos(\theta_1)(L_1 + L_2\cos(\theta_2)) + \cos(\theta_1 + \theta_2)L_2 & \cos(\theta_1 + \theta_2)L_2 & 0 \\ 0 & 0 & 0 \end{bmatrix} $$


$$det(J) = L_1L_2sin(θ_2)$$


- Meaning $sin(θ_2)$ can’t be equal 0

Condition 2 : Target out of reach
- This is simply calculated with total length of $L1, L2$ to see if wrist position stay within workspace
- The calculation is mentioned above


- Summary
$$\theta_1 = \alpha - \beta$$
$$\theta_2 = \tan^{-1}\left(\frac{\sin(\theta_2)}{\cos(\theta_2)}\right)$$
$$\theta_3 = \phi - (\theta_1 + \theta_2)$$

Singularities
- $sin(θ_2)$ can’t be equal 0
- $r > L_1 + L_2, r < |L_1 - L_2|$


---
### Robot Modeling
These are steps to create the planar 3-DOF manipulator by using Simscape from Simulink.

**Step 1** : Design the robot

![step-1](https://github.com/user-attachments/assets/9a4d4b76-5fc7-4ae8-a3f2-6980055adfd9)

These are the first designs of the planar 3-DOF manipulator which looks like a human arm. The model consists of 3 joints and 3 links.
- Link 1 is 5 meters from joint 1 
- Link 2 is 4 meters from joint 2
- Link 3 is 1 meters from joint 3

**Step 2** : Using Simulink’s Library called “ Simscape ” to create the model 

![image](https://github.com/user-attachments/assets/7779f63c-32f0-49be-bcc5-829bb3dba433)

First select “ Utilities ” and find a box named ‘ Solver Configuration ’. Then select “ Multibody ” next, select “ Frames and Transforms ” and “ Utilities ” find boxes named ‘ World Frame ’ , ‘ Reference Frame ’ and ‘ Mechanism Configuration ’ After finding all these boxes, place them on the workspace.

![image](https://github.com/user-attachments/assets/42710db8-cbd7-4f33-ba67-6facec0c5985)

These boxes are the core of creating Modeling without them, the physical modeling environment will not function, as they establish the simulation's framework and relationships between components. Also you can check your World frame and Reference frame by pressing play button. 

![image](https://github.com/user-attachments/assets/2302b77f-24c9-4235-909a-8af266f768ad)

**Step 3** : Using “ Rigid Transform block ” to transform from world frame into your desired frame example, rotate the frame by rotating about 90 degrees about X axis.

![image](https://github.com/user-attachments/assets/8340d83b-c236-4be5-8c70-a985840a78da)

You can apply a circular solid block to see how the frame changes like this picture below !

![image](https://github.com/user-attachments/assets/8f723b47-6f8b-4bc6-81b3-214133e21c55)

**Step 4** : After adjusting the frame you want, now add a block called ‘ Revolute Joint ’(Blue highlight on the right). This block has many function to play with such as state target or equilibrium but the key is this block can make your circular solid block rotate
![image](https://github.com/user-attachments/assets/f76d33d7-a114-4c06-9065-c8fede47727a)

### **Model Overview**

![model-overview](https://github.com/user-attachments/assets/ea92214d-75c4-442d-a83f-8b79d69e1225)

![image](https://github.com/user-attachments/assets/8adf25d7-48d5-4010-a6f1-9a532c107220)

There are 6 solid blocks :
1. Brick Solid
	- L1 : mass is 1 kg length is 5 meters 
	- L2 : mass is 1 kg length is 4 meters 
	- L3 : mass is 1 kg length is 1 meters 
2. Cylindrical Solid
	- J1 : mass is 0 kg also is a rotating joint
	- J2 : mass is 0 kg also is a rotating joint
	- J3 : mass is 0 kg also is a rotating joint


---



#### Trajectory Planning
This trajectory part take input from inverse kinematic to use as reference position and calculate trajectory output

- Goal :
  - Use Trajectory to reduce sudden change in motion
  - Dynamically updating reference position in real time when input signal change
- Trajectory used :
  - Adjusted Trapezoidal Trajectory (large motion range)
  - Cubic Trajectory (small motion range, noisy input)
- Method :
  - We adjusted Trapezoidal Trajectory which require constant start and finish time to suit constantly changing input by calculating accelerate and steady duration with distant to target
  - While this method is similar to PID it is designed to help with reducing sudden change in velocity, acceleration from using PID alone as well as allowing limitation for velocity, acceleration and jerk
  - Cubic trajectory is integrated for small motion range and noisy input

![image](https://github.com/user-attachments/assets/0dace636-bdb0-4613-9e3a-958a53d345ca)

Note : Max values are based on calculation part and slight adjustment for each joints to match human motion and ensure smoothness
Inputs: 
$qf$ : Target position (from I.K.)
$q0$ : Current position feedback
$dt$ : Sampling time
$v_{\text{max}}$ : Maximum velocity
$a_{\text{max}}$ : Maximum acceleration
$j_{\text{max}}$  : Maximum jerk
threshold : Motion range threshold for switching between trajectory
Outputs: 
$q$   : Position
$q_{\text{dot}}$  : Velocity
$q_{\text{ddot}}$ : Acceleration


**Find maximum acceleration**
1. Calculate the Moments of Inertia
         Each link contributes to the total moment of inertia $(I)$ of the system, which is a key factor in determining the maximum torque and, consequently, the acceleration. 
For a uniform rod rotating about one end: 
$$I = \frac{1}{3} m L^2$$ 
 - Link Parameters: 
   - Link 1: $m_1 = 1 \, \text{kg}, L_1 = 5 \, \text{m}$ 
   - Link 2: $m_2 = 1 \, \text{kg}, L_2 = 4 \, \text{m}$
   - Link 3: $m_3 = 1 \, \text{kg}, L_3 = 1 \, \text{m}$ 

Total Moment of Inertia
$$I_{\text{total}} = I_1 + I_2 + I_3$$ 

Substitute $I_i = \frac{1}{3} m_i L_i^2$

 $$I_1 = \frac{1}{3}(1)(5^2) = \frac{25}{3} \, \text{kg ⋅ m}^2$$
 
 $$I_2 = \frac{1}{3}(1)(4^2) = \frac{16}{3} \, \text{kg⋅m}^2$$
 
 $$I_3 = \frac{1}{3}(1)(1^2) = \frac{1}{3} \, \text{kg⋅m}^2$$
 
 $$I_{\text{total}} = \frac{25}{3} + \frac{16}{3} + \frac{1}{3} = 14 \, \text{kg⋅m}^2$$
 
2. Maximum Torque
Assume the motors can apply a total torque $T_{\text{max}}$ on the system. If the motors can apply $T_{\text{max}} = 50 \, \text{Nm}$, we can estimate $(a_{\text{max}})$. 
Using the rotational analog of Newton's second law 
$$\alpha = \frac{T}{I}$$
 $$a_{\text{max}} = \alpha \cdot L_{\text{eff}}$$

  Where $L_{\text{eff}}$ is the effective length of the links (weighted average of lengths)
  $$L_{\text{eff}} = \frac{\sum m_i L_i}{\sum m_i} = \frac{1 \cdot 5 + 1 \cdot 4 + 1 \cdot 1}{3} = 3.33 \, \text{m}$$ 
  
  Substitute values: 
  
  $$\alpha = \frac{50}{14} \approx 3.57 \, \text{rad/s}^2$$ 
  
  $$a_{\text{max}} = \alpha \cdot L_{\text{eff}}$$ 
  
  $$\approx 3.57 \cdot 3.33 \approx 11.9 \, \text{m/s}^2$$ 
  
  4. Maximum Velocity
  The maximum velocity $(v_{\text{max}})$ depends on
   *  Link lengths: Longer links result in higher end-effector velocities for the same joint velocities. 
   *  Practical constraints (e.g. motor limits, safety.)
  
  Assume the robot moves through its maximum angular displacement $( \theta_{\text{max}} = 90^\circ = \frac{\pi}{2} \, \text{rad} )$ in 1 second:
  $$v_{\text{max}} = \omega_{\text{max}} \cdot L_{\text{eff}}$$ 
  With $\omega_{\text{max}} = \frac{\pi}{2} \, \text{rad/s}:$
  
  $$v_{\text{max}} = \frac{\pi}{2} \cdot 3.33 \approx 5.23 \, \text{m/s} ]$$ 
  5. Final Recommendations
-  Maximum Acceleration: $a_{\text{max}} = 10 \, \text{m/s}^2$ (rounding for simplicity.)
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

### **Adjusting Robot’s End-Effector Speed**
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

#### **Trajectory Summary (calculation)**

1. **Moment of Inertia**
$$I_{\text{total}} = >20 \, \text{kg} \cdot \text{m}^2$$

2. **Maximum Angular Acceleration**:
$$\alpha_{\text{max}} = 3-5 \, \text{rad/s}^2$$

3. **Maximum Angular Velocity**:
$$\omega_{\text{max}} = 0.5 \, \text{rad/s}$$

4. **Maximum End-Effector Velocity**:
$$v_{\text{linear}} = \omega_{\text{max}} \cdot L = 0.5 \cdot 5 = 2.5 \, \text{m/s}$$

![Raw_Traject_input_output](https://github.com/user-attachments/assets/b94a4482-791d-45aa-a542-1c237614440a)
---
#### **Trajectory Performance Test**
#### **Position**
![image-removebg-preview](https://github.com/user-attachments/assets/f69ae0b1-d3c1-4f45-9eb5-c9d4a19459c0)

As a result, 

Input/Reference signal :
   
   - Trajectory is slightly delayed after Raw input.

Output (PID) : 
   - Trajectory is seen smoother and slightly faster than raw output 

Overall : 
   - Trajectory is better but clear different is hard to see, proceed to velocity comparison

#### **Velocity**
![Project-Kinematics (6)](https://github.com/user-attachments/assets/a8ff9468-657e-485b-af24-6d0a2287c643)

For velocity we use line graph for measurement 

Input/Reference signal :
   - Trajectory is very smooth compared to fluctuating raw signal

Output (PID) : 
   - At steady stage Raw signal and Trajectory are of similar length, Trajectory is slightly more smooth
   - However there are significant extreme value change when change occur 

Overall : 
   - Trajectory output is effective for smoothening velocity
   - Thus, concluded.


---
### Forward Kinematics
	
When there is a movement of the human arm with specified coordinates (x, y), the model calculates the Inverse Kinematics using the (x, y) coordinates of the human arm. The result is q1, q2, q3. The model uses the q values of the human arm to control the movement of a 3DOF robotic arm. The 3DOF robotic arm then sends its q1, q2, q3 values back to be calculated in Forward Kinematics to verify whether the movement patterns of the human arm and the robotic arm match. The Forward Kinematics calculation produces (x, y) coordinates of the robotic arm, which are then compared with the (x, y) coordinates of the human arm movement. If the coordinates match, it indicates that the movement is correct.
- Calculate the Forward Kinematics to obtain the (x, y) coordinates of the robotic arm
$$x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3)$$

$$y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3)$$

- Matlab 

![matlab](https://github.com/user-attachments/assets/6960d93f-59c5-41af-97f4-38ae74550ef0)
- Simulink
![simulink](https://github.com/user-attachments/assets/190d1676-65c6-413d-96e0-af15996aae74)

![Screenshot 2024-12-11 144355](https://github.com/user-attachments/assets/16996204-952b-4114-8bb9-365032dd62b7)

It was found that incorporating the trajectory into the model resulted in a high error value during the initial phase, which gradually approached zero over time.

---


### **Inverse Dynamics Integration**
Find Inverse Dynamic of the robot by using Euler Method

**Defined Parameters**
   - L1 = 5m and mass is 1kg
   - L2 = 4m and mass is 1kg
   - L3 = 1m and mass is 1kg
   - g = gravity 9.81 m/s2
   - Joint Angles, Joint Velocities, Joint Accelerations

**Equations of Motion**

$$M(q)\ddot{q} + B(q)\dot{q} + g(q) = \tau$$


Where 
   - $M(q)$ is an inertia matrix
   - $B(q,qdot)$ is a coriolis and centrifugal matrix
   - $g(q)$ is gravity

**Find M(q)**

    M(1, 1) = I(1) + I(2) + I(3) + m(2)*L(1)^2 + m(3)*(L(1)^2 + L(2)^2 + 2*L(1)*L(2)*c2);
    
    M(1, 2) = I(2) + I(3) + m(3)*(L(2)^2 + L(1)*L(2)*c2);
    
    M(1, 3) = I(3);
    
    M(2, 1) = M(1, 2);
    
    M(2, 2) = I(2) + I(3) + m(3)*L(2)^2;
    
    M(2, 3) = I(3);
    
    M(3, 1) = M(1, 3);
    
    M(3, 2) = M(2, 3);
    
    M(3, 3) = I(3);

**Find B(q,qdot)**

    B(1, 2) = -m(3)*L(1)*L(2)*s2*qd(2);
    
    B(1, 3) = -m(3)*L(1)*L(2)*s2*qd(3);
    
    B(2, 1) = m(3)*L(1)*L(2)*s2*qd(1);
    
    B(2, 3) = -m(3)*L(2)*L(3)*s3*qd(3);
    
    B(3, 1) = m(3)*L(1)*L(2)*s2*qd(1);
    
    B(3, 2) = m(3)*L(2)*L(3)*s3*qd(2);

**Find G(q)**

    G(1) = -g * (m(1)*L(1)/2*c1 + m(2)*(L(1)*c1 + L(2)/2*c12) + m(3)*(L(1)*c1 + L(2)*c12 + L(3)/2*c123));
    
    G(2) = -g * (m(2)*L(2)/2*c12 + m(3)*(L(2)*c12 + L(3)/2*c123));
    
    G(3) = -g * m(3)*L(3)/2*c123;

These are codes we used in order to find the Inverse kinematic of the robot.
![image](https://github.com/user-attachments/assets/3f686d61-593e-432e-befd-7c8135673265)

The torque calculated through inverse dynamics was tested, and the graph lacked smoothness during periods of significant movement. However, the values were relatively stable and formed a realistic pattern consistent with the motion of the human arm.

![image-DY](https://github.com/user-attachments/assets/3daea5e5-dbc8-4d38-bf49-3d55d163563e)

The torque values were then tested on the model, and it was found that the model came to a halt and could not move due to the occurrence of singularity. It is hypothesized that this issue arose because the model was not well-designed, such as errors in frame orientation or restrictions within the Simulink system that may have caused the singularity.

----

## User Guide

https://github.com/user-attachments/assets/2659b10a-2745-4335-bb0c-b3db2ac1d299


- **Step1** : User must obtain a file of mediapipe code which can detect your body and movement. After the user obtains it, User must run this file first then wait until the two windows appear
![Screenshot 2024-12-11 135747](https://github.com/user-attachments/assets/40069d0b-dbc2-491d-bc8c-1f98276d6f63)
![Screenshot 2024-12-11 135843](https://github.com/user-attachments/assets/3370ce8a-f13e-42dd-83ad-d2e175462603)

- **Step2** : Open MATLAB/SIMULINK, inside the file There are 4 parts:
1. UDP and Input : This area is all about receive data from Vscode (MediaPipe)
2. 3 Parts of input to Model. You can choose one out of 3 parts to run the model, the other you can comment out.
	- Inverse Kinematic Only 
	- Trajectory Only
	- Inverse Dynamic Only

![Screenshot 2024-12-11 135946](https://github.com/user-attachments/assets/4d993a45-0078-431a-9a6c-2e6fc0e55243)

After choose input to model press run the code then wait for a second until simulink is done compiling and Time is start counting

![Screenshot 2024-12-11 140022](https://github.com/user-attachments/assets/8a03c1b2-e602-4473-bd4d-bb4a9f2ba8cf)

- **Step3** : Try waving in front of the camera with your left arm. You will see your arm being tracked and the model starts to move. Enjoy !

![image](https://github.com/user-attachments/assets/06c922f2-5b0d-4b35-a7a8-8d592bda5418)

## Conclusion

---
## References
[1] Aksoy, Egemen 1, Çakır, Arif Dorukan 2, Erol, Berat Alper 3, Gumus, Abdurrahman 4. Real Time Computer Vision Based Robotic Arm Controller with ROS and Gazebo Simulation Environment. https://doi.org/10.XXXX/elec.2023

[2] Altayeb, Muneera. (2023). Hand Gestures Replicating Robot Arm based on MediaPipe. Indonesian Journal of Electrical Engineering and Informatics (IJEEI), 11(3), 727–737. https://doi.org/10.52549/ijeei.v11i3.4491

[3] Juecoree. (2021). Forward and Reverse Kinematics for 3R Planar Manipulator. Retrieved from https://hive.blog/hive-196387/@juecoree/forward-and-reverse-kinematics-for-3r-planar-manipulator

[4] Wang, Ziwei 1, Liang, Rongjian 2, Chen, Zhang 3, Liang, Bin 4. (2020). Fast and Intuitive Kinematics Mapping for Human-Robot Motion Imitating: A Virtual-Joint-Based Approach. IFAC PapersOnLine, 53(2), 10011–10018. https://doi.org/10.1016/j.ifacol.2020.12.2720
