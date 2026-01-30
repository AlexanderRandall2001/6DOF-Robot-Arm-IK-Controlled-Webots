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

## Example Usage
The code snippets below will test each of the functions

```python
# Forward Kinematics (FK)

theta_fk = (0, 0.523599, 1.0472, 0, -0.523599, 0.785398)
dh_table = [
    [0, 0.089, 0,  1.5708],
    [0.523599, 0,     0.425, 0],
    [1.0472, 0,     0.392, 0],
    [0, 0.109, 0,  1.5708],
    [-0.523599, 0.095, 0, -1.5708],
    [0.785398, 0.082, 0,  0]
]

fk_solver = fk_6dof(dh_table)
joints, orientation = fk_solver.fk()
print("FK joint positions:", joints)
print("FK end-effector orientation (Euler xyz):", orientation)

# Inverse Kinematics (IK)

dh_table = [
    [0, 0.089, 0,  1.5708],
    [0, 0,     0.425, 0],
    [0, 0,     0.392, 0],
    [0, 0.109, 0,  1.5708],
    [0, 0.095, 0, -1.5708],
    [0, 0.082, 0,  0]
]

target_pos = (0.5, 0.0, 0.2)                
target_quat = (0, 0, 0, 1)                 

ik_solver = ik_6dof(dh_table)
theta_initial = np.array([0, 0.1, 0.2, 0, 0, 0])  
theta_sol, Ts, error_norm = ik_solver.solve(target_pos, target_quat, theta_initial=theta_initial)
print("IK joint solution (theta1..theta6):", theta_sol)
print("Final end-effector error norm:", error_norm)


