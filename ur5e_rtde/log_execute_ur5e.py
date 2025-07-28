"""
*******************************************************************************

Project: system-id ur5e
File: log_execute_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: July 28, 2025

Description:
Python script using RTDE protocol for logging data and executing a trajectory
from a csv file (joint-space) at the same time.

License:
This script is licensed under the MIT License.
You may obtain a copy of the License at
    https://opensource.org/licenses/MIT

SPDX-License-Identifier: MIT

Disclaimer:
This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose, and noninfringement. In no event shall the
authors be liable for any claim, damages, or other liability, whether in an
action of contract, tort, or otherwise, arising from, out of, or in connection
with the software or the use or other dealings in the software.

*******************************************************************************
"""
import csv, os, time, yaml
from ur5e_rtde import get_receive_interface, get_control_interface

_pkg_dir = os.path.dirname(__file__)
_cfg_path = os.path.abspath(os.path.join(_pkg_dir, '..', 'config', 'config.yml'))
with open(_cfg_path, 'r') as f:
    cfg = yaml.safe_load(f)

LOG_RATE_HZ = cfg['UR5E']['LOG_RATE_HZ']

# --- trajectory loading ---
traj_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'traj')
csv_traj = os.path.join(traj_dir, 'ur5e_smooth_random_joint_traj_v1.csv')  # TODO
trajectory = []
with open(csv_traj, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row['time'])
        q = [float(row[f'q{i}']) for i in range(1, 7)]
        trajectory.append((t, q))

# --- logging setup ---
log_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'logs')
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, 'ur5e_smooth_random_joint_traj_test.csv')  # TODO
period = 1.0 / LOG_RATE_HZ

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()

# move to start
_, q0 = trajectory[0]
rtde_ctrl_iface.moveJ(q0, speed=1.0, acceleration=0.5)

speed = 0.1
acceleration = 0.1
lookahead_time = 0.1
gain = 100

start = time.time()
prev_t = trajectory[0][0]

with open(log_path, 'w', newline='') as csvf:
    writer = csv.writer(csvf)
    writer.writerow([
        'time','q1','q2','q3','q4','q5','q6',
        'qd1','qd2','qd3','qd4','qd5','qd6',
        'qdd1','qdd2','qdd3','qdd4','qdd5','qdd6',
        'x','y','z','R_x','R_y','R_z',
        'V_x','V_y','V_z','omega_x','omega_y','omega_z',
        'F_x','F_y','F_z','T_x','T_y','T_z',
        'actualI1','actualI2','actualI3','actualI4','actualI5','actualI6',
        'outputI1','outputI2','outputI3','outputI4','outputI5','outputI6',
        'tau1','tau2','tau3','tau4','tau5','tau6',
        'temp1','temp2','temp3','temp4','temp5','temp6',
        'ctrl_tau1','ctrl_tau2','ctrl_tau3','ctrl_tau4','ctrl_tau5','ctrl_tau6'
    ])

    try:
        for t_des, q_des in trajectory[1:]:
            loop_start = time.time()
            dt = t_des - prev_t
            prev_t = t_des

            # command next segment
            rtde_ctrl_iface.servoJ(q_des, speed, acceleration, dt, lookahead_time, gain)

            # collect data
            ts = rtde_recv_iface.getTimestamp()                       # timestamp in s
            qs = rtde_recv_iface.getActualQ()                         # actual_q, [q₁,…,q₆] in rad
            qds = rtde_recv_iface.getActualQd()                       # actual_qd, [q̇₁,…,q̇₆] in rad/s
            qdds = rtde_recv_iface.getTargetQdd()                     # target_qdd, [qdd1, qdd2, ..., qdd6] in rad/s^2
            tcp_pose = rtde_recv_iface.getActualTCPPose()             # actual_TCP_pose, [x,y,z, Rx, Ry, Rz]
            tcp_speed = rtde_recv_iface.getActualTCPSpeed()           # actual_TCP_speed, [vx, vy, vz, ωx, ωy, ωz]
            tcp_force = rtde_recv_iface.getActualTCPForce()           # actual_TCP_force, [Fx, Fy, Fz, Tx, Ty, Tz]
            q_current = rtde_recv_iface.getActualCurrent()            # actual_current, [I1, I2, …, I6] in mA
            q_ctrl_current = rtde_recv_iface.getJointControlOutput()  # joint_control_output, [I1, I2, …, I6] in mA
            target_moment = rtde_recv_iface.getTargetMoment()         # target_moment, [T1, T2, ..., T6] in Nm
            q_temperatures = rtde_recv_iface.getJointTemperatures()   # joint_temperatures [t1, t2, ..., t6] in degrees Celsius
            torques = rtde_ctrl_iface.getJointTorques()               # get_joint_torques [τ₁, τ₂, τ₃, τ₄, τ₅, τ₆] in Nm

            writer.writerow([
                f"{ts:.4f}",
                *[f"{v:.4f}" for v in qs],
                *[f"{v:.4f}" for v in qds],
                *[f"{v:.4f}" for v in qdds],
                *[f"{v:.4f}" for v in tcp_pose],
                *[f"{v:.4f}" for v in tcp_speed],
                *[f"{v:.4f}" for v in tcp_force],
                *[f"{v:.4f}" for v in q_current],
                *[f"{v:.4f}" for v in q_ctrl_current],
                *[f"{v:.4f}" for v in target_moment],
                *[f"{v:.4f}" for v in q_temperatures],
                *[f"{v:.4f}" for v in torques]
            ])
            csvf.flush()

            # maintain rate
            dt_loop = time.time() - loop_start
            to_sleep = period - dt_loop
            if to_sleep > 0:
                time.sleep(to_sleep)

        rtde_ctrl_iface.servoStop()
        rtde_ctrl_iface.stopScript()
    except KeyboardInterrupt:
        print('Execution interrupted by user.')
        rtde_ctrl_iface.servoStop()
        rtde_ctrl_iface.stopScript()

rtde_ctrl_iface.disconnect()