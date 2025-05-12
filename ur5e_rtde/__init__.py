"""
*******************************************************************************

Project: system-id ur5e
File: __init__.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 11, 2025

Description: -

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

import os, yaml, rtde_receive, rtde_control

_pkg_dir = os.path.dirname(__file__)
_cfg_path = os.path.abspath(os.path.join(_pkg_dir, '..', 'config', 'config.yml'))
with open(_cfg_path) as f:
    _cfg = yaml.safe_load(f)
ROBOT_ID = _cfg['UR5E']['ROBOT_ID']

_rtde_recv = None
_rtde_ctrl = None

def get_receive_interface():
    """Always safe to call from multiple scripts."""
    global _rtde_recv
    if _rtde_recv is None:
        _rtde_recv = rtde_receive.RTDEReceiveInterface(ROBOT_ID)
    return _rtde_recv

def get_control_interface():
    """Only call this if you actually need to send commands."""
    global _rtde_ctrl
    if _rtde_ctrl is None:
        _rtde_ctrl = rtde_control.RTDEControlInterface(ROBOT_ID)
    return _rtde_ctrl
