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
from ur5e_rtde._read_write_ur5e import ur5e_homming, setup_configuration

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()  

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Script to call different functions based on arguments.")
    parser.add_argument(
        "action",
        nargs="?",
        choices=["home", "setup_config"],
        default="home",
        help="Specify the action to perform: 'home' or 'setup_config' (defaults to 'home')."
    )
    args = parser.parse_args()

    try:
            
        if args.action == "home":
            ur5e_homming(speed=0.5, accel=0.5)

        elif args.action == "setup_config":
            setup_configuration(speed=0.5, accel=0.5)


        print("Robot stopped.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Ensure the connection is closed
        rtde_ctrl_iface.disconnect()