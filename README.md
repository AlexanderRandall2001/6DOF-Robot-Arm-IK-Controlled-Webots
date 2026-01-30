# 6DOF_Robot_Arm_IK_Controlled_Webots

Implements forward and inverse kinematics for a PUMA-560 6-DOF arm in Python, with a Webots controller moving the end effector between arbitrary target positions and orientations using numerical Jacobian-based IK.

## Demo

![PUMA-560 Arm Motion](6DOF_Robot_Arm_animation.gif)

## Features

### Forward Kinematics (FK)
- Computes end-effector position and orientation from given joint angles.
- Uses Denavit-Hartenberg (DH) parameters for accurate robotic arm modeling.
- Provides xyz coordinates and Euler angles for easy analysis and visualization.

### Inverse Kinematics (IK)
- Solves for joint angles from arbitrary end-effector positions and orientations.
- Implements Jacobian-based damped least squares (DLS) method for stable convergence.
- Handles orientation errors using rotation vectors for accurate end-effector alignment.
- Accepts initial joint guesses for faster, more reliable numerical solutions.

### Webots Simulation & Controller
- Implements PUMA-560 Denavit–Hartenberg parameters and joint limits for realistic motion
- Moves the robotic arm between arbitrary 3D target poses in real-time.
- Uses the IK solver to compute joint positions dynamically.
- Applies proportional gains to gradually move motors toward the target.
- Demonstrates full 6-DOF motion, including orientation adjustments.
