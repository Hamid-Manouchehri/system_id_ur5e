"""
*******************************************************************************

Project: system-id ur5e
File: read_ur5e.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: July 27, 2025

Description:
Python script using URScript for reading status of UR5e and Logging in a
.csv file independently.

Using python script as: URScript generator + Network pip

We do not run URScript under python's interpreter, but we compose
URScript text in python and configure proper TCP socket for streaming that
to robot's controller.


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
from rtde_control import RTDEControlInterface as RTDEControl

robot_ip = "192.168.1.100"        # Replace with the IP of your robot

# Create the RTDE Control interface (starts the RTDE control script automatically)
rtde_c = RTDEControl(robot_ip)

try:
    # Retrieve joint torques (already compensated for gravity, friction, etc.)
    torques = rtde_c.getJointTorques()
    print("Joint torques [Nm]:", torques)
finally:
    # Always stop the script when finished
    rtde_c.stopScript()




