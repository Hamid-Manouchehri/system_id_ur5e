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
from ur5e_rtde import get_receive_interface

rtde_recv_iface = get_receive_interface()


with open("../config/config.yml", "r") as f:
    cfg = yaml.safe_load(f)

LOG_RATE_HZ = cfg['UR5E']['LOG_RATE_HZ']

csv_file_name = "ur5e_data_test.csv"  # TODO
csv_dir = '../data/logs/'
os.makedirs(csv_dir, exist_ok=True)

csv_path = os.path.join(csv_dir, csv_file_name)

period = 1.0 / LOG_RATE_HZ
with open(csv_path, "w", newline="") as csvf:
    writer = csv.writer(csvf)
    # write your header row:
    writer.writerow(["time", "q1", "q2", "q3", "q4", "q5", "q6",
                     "qd1", "qd2", "qd3", "qd4", "qd5", "qd6",
                     "x", "y", "z", "R_x", "R_y", "R_z",
                     "V_x", "V_y", "V_z", "omega_x", "omega_y", "omega_z",
                     "I1", "I2", "I3", "I4", "I5", "I6",
                     "F_x", "F_y", "F_z", "T_x", "T_y", "T_z"])
    t0 = time.time()

    try:
        while True:
            loop_start = time.time()
            # read your data:
            qs  = rtde_recv_iface.getActualQ()  # [q₁,…,q₆] in rad
            qds = rtde_recv_iface.getActualQd()  # [q̇₁,…,q̇₆] in rad/s
            tcpPose = rtde_recv_iface.getActualTCPPose()  # [x,y,z, Rx, Ry, Rz]
            tcpSpeed = rtde_recv_iface.getActualTCPSpeed()  # [vx, vy, vz, ωx, ωy, ωz]
            qCurrent = rtde_recv_iface.getActualCurrent()  # [I1, I2, …, I6] in mA
            tcpForce = rtde_recv_iface.getActualTCPForce()  # [Fx, Fy, Fz, Tx, Ty, Tz]
            ts  = time.time() - t0

            # write it out:
            writer.writerow([f"{ts:.4f}"] + [f"{v:.4f}" for v in qs] +
                            [f"{v:.4f}" for v in qds] + 
                            [f"{v:.4f}" for v in tcpPose] +
                            [f"{v:.4f}" for v in tcpSpeed] +
                            [f"{v:.4f}" for v in qCurrent] +
                            [f"{v:.4f}" for v in tcpForce])
            csvf.flush()

            # sleep to enforce rate
            dt = time.time() - loop_start
            to_sleep = period - dt

            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("Logging stopped by user.")
