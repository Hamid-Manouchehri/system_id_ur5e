"""
*******************************************************************************

Project: system-id ur5e
File: read_write_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 8, 2025

Description:
Python script using RTDE protocol for reading and writing commands of UR5e.

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

def get_tcp_pose(robot_id):
    rtde = rtde_receive.RTDEReceiveInterface(robot_id)
    pose = rtde.getActualTCPPose()
    rtde.disconnect()
    return pose

def movej(robot_id, joints, speed=0.2, accel=0.5):
    rtde = rtde_control.RTDEControlInterface(robot_id)
    rtde.moveJ(np.array(joints), speed=speed, acceleration=accel)
    rtde.disconnect()

def movel(robot_id, pose, speed=0.2, accel=0.5, async=False):
    rtde = rtde_control.RTDEControlInterface(robot_id)
    rtde.moveL(pose, speed=speed, acceleration=accel, asynchronous=async)
    rtde.disconnect()
