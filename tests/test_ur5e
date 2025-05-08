"""
*******************************************************************************

Project: system-id ur5e
File: test_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 8, 2025

Description:
Python script using RTDE protocol for reading status of UR5e.

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

import rtde_receive
import yaml

with open("../config/config.yml", 'r') as file:
    config = yaml.safe_load(file)

ROBOT_ID = config["UR5E"]["ROBOT_ID"]

# Initialize RTDE Receive interface
rtde_receive_interface = rtde_receive.RTDEReceiveInterface(ROBOT_ID)


def format_float_list(float_list, decimals=5):
    
    return [round(val, decimals) for val in float_list]


if __name__ == "__main__":

    try:
        # Read joint positions
        joint_positions = rtde_receive_interface.getActualQ()
        print("Joint Positions (rad):\n", format_float_list(joint_positions))

        # Read joint velocities
        joint_velocities = rtde_receive_interface.getActualQd()
        print("\nJoint Velocities (rad/s):\n", format_float_list(joint_velocities))

        # Read TCP position (end-effector pose w.r.t. base)
        tcp_position = rtde_receive_interface.getActualTCPPose()
        print("\nEnd-Effector Position (m and rad):\n", format_float_list(tcp_position))

        # Read TCP speed
        tcp_speed = rtde_receive_interface.getActualTCPSpeed()
        print("\nEnd-Effector Speed (m/s and rad/s):\n", format_float_list(tcp_speed))

    except Exception as e:
        print(f"An error occurred: {e}")
