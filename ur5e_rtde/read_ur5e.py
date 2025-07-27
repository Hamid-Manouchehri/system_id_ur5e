"""
*******************************************************************************

Project: system-id ur5e
File: read_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 11, 2025

Description:
Python script using RTDE protocol for reading status of UR5e and Logging in a
.csv file independently.

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
#!/usr/bin/env python3

import os, csv, time, yaml
from ur5e_rtde import get_receive_interface, get_control_interface

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()

with open("../config/config.yml", "r") as f:
    cfg = yaml.safe_load(f)

LOG_RATE_HZ = cfg['UR5E']['LOG_RATE_HZ']

csv_file_name = "ur5e_smooth_random_joint_traj_test.csv"  # TODO
csv_dir = '../data/logs/'
os.makedirs(csv_dir, exist_ok=True)

csv_path = os.path.join(csv_dir, csv_file_name)

period = 1.0 / LOG_RATE_HZ
with open(csv_path, "w", newline="") as csvf:
    writer = csv.writer(csvf)
    # write your header row:
    writer.writerow(["time", "q1", "q2", "q3", "q4", "q5", "q6",
                     "qd1", "qd2", "qd3", "qd4", "qd5", "qd6",
                     "qdd1", "qdd2", "qdd3", "qdd4", "qdd5", "qdd6",
                     "x", "y", "z", "R_x", "R_y", "R_z",
                     "V_x", "V_y", "V_z", "omega_x", "omega_y", "omega_z",
                     "F_x", "F_y", "F_z", "T_x", "T_y", "T_z",
                     "actualI1", "actualI2", "actualI3", "actualI4", "actualI5", "actualI6",
                     "outputI1", "outputI2", "outputI3", "outputI4", "outputI5", "outputI6",
                     "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
                     "temp1", "temp2", "temp3", "temp4", "temp5", "temp6", 
                     "tau1", "tau2", "tau3", "tau4", "tau5", "tau6"])
    t0 = time.time()

    try:
        print("\n\nReading data from UR5e and saving as a CSV file...\n\n")
        while True:
            loop_start = time.time()

            # ts  = time.time() - t0
            ts = rtde_recv_iface.getTimestamp()                        # timestamp in s
            qs  = rtde_recv_iface.getActualQ()                         # actual_q, [q₁,…,q₆] in rad
            qds = rtde_recv_iface.getActualQd()                        # actual_qd, [q̇₁,…,q̇₆] in rad/s
            qddsTarget = rtde_recv_iface.getTargetQdd()                # target_qdd, [qdd1, qdd2, ..., qdd6] in rad/s^2
            tcpPose = rtde_recv_iface.getActualTCPPose()               # actual_TCP_pose, [x,y,z, Rx, Ry, Rz]
            tcpSpeed = rtde_recv_iface.getActualTCPSpeed()             # actual_TCP_speed, [vx, vy, vz, ωx, ωy, ωz]
            tcpForce = rtde_recv_iface.getActualTCPForce()             # actual_TCP_force, [Fx, Fy, Fz, Tx, Ty, Tz]
            qCurrent = rtde_recv_iface.getActualCurrent()              # actual_current, [I1, I2, …, I6] in mA
            qControlCurrent = rtde_recv_iface.getJointControlOutput()  # joint_control_output, [I1, I2, …, I6] in mA
            targetMoment = rtde_recv_iface.getTargetMoment()           # target_moment, [T1, T2, ..., T6] in Nm
            qTemperatures = rtde_recv_iface.getJointTemperatures()     # joint_temperatures [t1, t2, ..., t6] in degrees Celsius
            torques = rtde_ctrl_iface.getJointTorques()                # get_joint_torques [τ₁, τ₂, τ₃, τ₄, τ₅, τ₆] in Nm

            # write it out:
            writer.writerow([f"{ts:.4f}"] + 
                            [f"{v:.4f}" for v in qs] +
                            [f"{v:.4f}" for v in qds] + 
                            [f"{v:.4f}" for v in qddsTarget] + 
                            [f"{v:.4f}" for v in tcpPose] +
                            [f"{v:.4f}" for v in tcpSpeed] +
                            [f"{v:.4f}" for v in tcpForce] + 
                            [f"{v:.4f}" for v in qCurrent] +
                            [f"{v:.4f}" for v in qControlCurrent] +
                            [f"{v:.4f}" for v in targetMoment] + 
                            [f"{v:.4f}" for v in qTemperatures] +
                            [f"{v:.4f}" for v in torques])
            csvf.flush()

            # sleep to enforce rate
            dt = time.time() - loop_start
            to_sleep = period - dt

            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("Logging stopped by user.")
