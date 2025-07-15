"""
*******************************************************************************

Project: system-id ur5e
File: write_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 11, 2025

Description:
Python script using RTDE protocol for writing command on UR5e.

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

import numpy as np
import os, csv, time, yaml, argparse
from ur5e_rtde import get_receive_interface, get_control_interface
from ur5e_rtde._modules import ur5e_homming, setup_configuration, read_csv_trajectory

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()  

with open("../config/config.yml", "r") as f:
    cfg = yaml.safe_load(f)

EXCITING_TRAJ_FILE = cfg['DATA']['EXCITING_TRAJ_FILE']

FREQUENCY   = 50               # Hz (UR5e RTDE max ~125 Hz) TODO
DT          = 1.0 / FREQUENCY   # seconds per control step
LOOKAHEAD   = 0.1               # seconds for look-ahead interpolation
GAIN        = 300               # servoJ gain

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Script to call different functions based on arguments.")
    parser.add_argument(
        "action",
        nargs="?",
        choices=["home", "setup_config", "random", "exciting_traj"],
        default="home",  # TODO
        help="Specify the action to perform: 'home' or 'setup_config' (defaults to 'home')."
    )
    args = parser.parse_args()

    try:
            
        if args.action == "home":
            ur5e_homming(speed=0.5, accel=0.5)

        elif args.action == "setup_config":
            setup_configuration(speed=0.5, accel=0.5)

        elif args.action == "random":
            joint_setup_position = np.array([0.0, -1.57, -.57, -1.57, 0.0, 0.0])  # Rad
            rtde_ctrl_iface.moveJ(joint_setup_position, speed=0.5, acceleration=0.5)
            joint_setup_position = np.array([0.0, -1.57, .57, -1.57, 0.0, 0.0])  # Rad
            rtde_ctrl_iface.moveJ(joint_setup_position, speed=0.5, acceleration=0.5)

            joint_setup_position = np.array([0.0, -1.57, -.57, -1.57, 0.0, 0.0])  # Rad
            rtde_ctrl_iface.moveJ(joint_setup_position, speed=0.5, acceleration=0.5)
            joint_setup_position = np.array([0.0, -1.57, .57, -1.57, 0.0, 0.0])  # Rad
            rtde_ctrl_iface.moveJ(joint_setup_position, speed=0.5, acceleration=0.5)
            
            ur5e_homming(speed=0.5, accel=0.5)

        elif args.action == "exciting_traj":
            traj = read_csv_trajectory(EXCITING_TRAJ_FILE)
            print(traj)
            if not traj:
                raise RuntimeError("No trajectory points loaded!")
            
            start_time = time.time()
            for idx, (t_des, q_des) in enumerate(traj):
                t0 = time.time()

                # send the next joint-target
                rtde_ctrl_iface.moveJ(q_des, speed=0.1, acceleration=0.1)

                # (optional) read back current pose for logging/debug
                actual_q = rtde_recv_iface.getActualQ()
                # print(f"{t_des:.3f} | desired: {q_des} | actual: {[round(a,3) for a in actual_q]}")

                # enforce real-time pacing
                dt = time.time() - t0
                if dt < DT:
                    time.sleep(DT - dt)


        print("Robot stopped.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Ensure the connection is closed
        rtde_ctrl_iface.disconnect()
        rtde_recv_iface.disconnect()