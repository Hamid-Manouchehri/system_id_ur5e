"""
*******************************************************************************

Project: system-id ur5e
File: execute_joint_traj.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: July 20, 2025

Description:
Python script using RTDE protocol for reading joint positions from
CSV file.

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
import os, csv, time, yaml
from ur5e_rtde import get_receive_interface, get_control_interface

# rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()

# Reading the CSV joint trajectory file and executing on UR5e.
csv_file_name = 'ur5e_smooth_random_joint_traj_v1.csv'  # TODO
csv_traj_file = "../data/traj/"
joint_trajectory = os.path.join(csv_traj_file, csv_file_name)

trajectory = []
with open(joint_trajectory, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row["time"])
        q = [float(row[f"q{i}"]) for i in range(1,7)]
        trajectory.append((t, q))

t0, q0 = trajectory[0]
rtde_ctrl_iface.moveJ(q0, speed=1, acceleration=.5)
# exit()

# Parameters
speed = .1
acceleration = .1
lookahead_time = 0.1  # within [0.03;0.2]
gain = 100  # within [100;2000]

start = time.time()
prev_t = t0
for t_des, q_des in trajectory[1:]:
    # how long until this point?
    dt = t_des - prev_t
    prev_t = t_des

    # call servoJ for this segment
    rtde_ctrl_iface.servoJ(q_des, speed, acceleration, dt, lookahead_time, gain)

    # maintain real-time pacing (optional if dt accurate)
    elapsed = time.time() - start
    if t_des > elapsed:
        time.sleep(t_des - elapsed)

rtde_ctrl_iface.servoStop()
rtde_ctrl_iface.stopScript()
rtde_ctrl_iface.disconnect()