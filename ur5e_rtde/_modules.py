"""
*******************************************************************************

Project: system-id ur5e
File: _read_write_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 8, 2025

Description:
Python module using RTDE protocol for reading and writing commands of UR5e.

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

import numpy as np
import rtde_control
import rtde_receive
import yaml
from ur5e_rtde import get_receive_interface, get_control_interface

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()


def ur5e_moveJ(joint_goal, speed=0.1, accel=0.5):
    rtde_ctrl_iface.moveJ(np.array(joint_goal), speed=speed, acceleration=accel)


# def ur5e_moveL():
#     tcp_position = np.array([-0.12433, -0.67481, 0.17184, -0.89113, 1.6404, -1.65547])  # m and Rad
#     rtde_control_interface.moveL(tcp_position, speed=speed, acceleration=accel, asynchronous=False)

def ur5e_homming(speed=0.1, accel=0.5):
    """ UR5e upright."""
    joint_home_position = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])  # Rad
    rtde_ctrl_iface.moveJ(joint_home_position, speed=speed, acceleration=accel)


def setup_configuration(speed=0.1, accel=0.5):
    joint_setup_position = np.array([1.40706, -1.31536, 1.61768, -1.85297, 4.708, -3.28425])  # Rad
    rtde_ctrl_iface.moveJ(joint_setup_position, speed=speed, acceleration=accel)