from controller import Robot
import numpy as np
from scipy.spatial.transform import Rotation
from Robot_Arm_6DOF_IK import ik_6dof 

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
motors = []

for name in motor_names:
    m = robot.getDevice(name)
    m.setPosition(0.0)
    m.setVelocity(1.0)
    motors.append(m)

# DH parameters for 6-DOF arm

dh_table = [
    [0, 0.089, 0,  1.5708],
    [0, 0,     0.425, 0],
    [0, 0,     0.392, 0],
    [0, 0.109, 0,  1.5708],
    [0, 0.095, 0, -1.5708],
    [0, 0.082, 0,  0]
]

ik_solver = ik_6dof(dh_table)

# End effector target poses

targets = [
    (np.array([ 0.450,  0.000, 0.200]), Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()),
    (np.array([ 0.400, -0.100, 0.200]), Rotation.from_euler('xyz', [30, -5, 5], degrees=True).as_quat()),
    (np.array([ 0.400, -0.100, 0.230]), Rotation.from_euler('xyz', [110, 5, -5], degrees=True).as_quat()),
    (np.array([ 0.300, -0.150, 0.310]), Rotation.from_euler('xyz', [120, 0, 0], degrees=True).as_quat()),
    (np.array([ 0.100, -0.300, 0.250]), Rotation.from_euler('xyz', [90, 0, 0], degrees=True).as_quat()),
    (np.array([ -0.050, -0.400, 0.210]), Rotation.from_euler('xyz', [60, 0, 0], degrees=True).as_quat()),
    (np.array([ -0.150, -0.300, 0.270]), Rotation.from_euler('xyz', [50, 0, 0], degrees=True).as_quat())
]

# Joint limits (rad)

joint_limits = [
    (-2.792, 2.792),
    (-3.92, 0.78),
    (-0.78, 3.92),
    (-1.9198, 2.96),
    (-1.745, 1.745),
    (-4.64, 4.64)
]

# Initial rest configuration

theta_current = np.array([0, np.pi/6, np.pi/3, 0, 0, 0])

for target_pos, target_quat in targets:
    try:
        theta_sol, Ts, _ = ik_solver.solve(
            target_pos,
            target_quat,
            theta_initial=theta_current
        )

        theta_sol = np.clip(
            theta_sol,
            [low for low,_ in joint_limits],
            [high for _,high in joint_limits]
        )

    except ValueError:
        print("Target unreachable:", target_pos)
        continue

    # Final end effector error

    Ts_final = ik_solver.fk(theta_sol)
    ee_pos_final = Ts_final[-1][:3,3]

    R_target = Rotation.from_quat(target_quat).as_matrix()
    R_current = Ts_final[-1][:3,:3]

    pos_error = np.linalg.norm(target_pos - ee_pos_final)
    ori_error = np.linalg.norm(
        ik_solver.orientation_error(R_current, R_target)
    )

    print(f"Final EE position: {ee_pos_final}, orientation: {Rotation.from_matrix(R_current).as_quat()}")
    print(f"Final true position error: {pos_error:.4f} m, orientation error: {ori_error:.4f} rad")

    # Proportional joint motion

    done = [False]*6
    while robot.step(timestep) != -1:
        diff = theta_sol - theta_current
        alpha = np.array([0.3, 0.3, 0.3, 0.15, 0.15, 0.15])

        theta_current += diff * alpha

        for i, m in enumerate(motors):
            m.setPosition(theta_current[i])
            done[i] = abs(diff[i]) < 1e-3

        if all(done):
            break

print("All targets processed.")