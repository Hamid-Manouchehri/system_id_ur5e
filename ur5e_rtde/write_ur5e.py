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
import os, csv, time, yaml
from ur5e_rtde import get_receive_interface, get_control_interface

recv_iface = get_receive_interface()
ctrl_iface = get_control_interface()  

def ur5e_homming(speed=0.1, accel=0.5):
    """ UR5e upright."""
    joint_home_position = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])  # Rad
    ctrl_iface.moveJ(joint_home_position, speed=speed, acceleration=accel)


def setup_configuration(speed=0.1, accel=0.5):
    # joint_setup_position = np.array([1.48174, -1.23385, 1.96952, -0.70358, 2.62826, -3.11278])  # Rad Graspio_1
    joint_setup_position = np.array([1.40706, -1.31536, 1.61768, -1.85297, 4.708, -3.28425])  # Rad Graspio_1
    ctrl_iface.moveJ(joint_setup_position, speed=speed)


if __name__ == "__main__":

    try:
        
        ur5e_homming(speed=0.5, accel=0.5)
        exit()
        setup_configuration(speed=0.5, accel=0.5)
        exit()

        print("Robot stopped.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Ensure the connection is closed
        ctrl_iface.disconnect()