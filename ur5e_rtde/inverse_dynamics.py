"""
*******************************************************************************

Project: system-id ur5e
File: inverse_dynamics.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 17, 2025

Description:
Python script calculating inverse dynamics of UR5e robot using 
Recursive Newton-Euler Algorithm (RNEA).

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
from ur5e_rtde._modules import ur5e_homming, setup_configuration

rtde_recv_iface = get_receive_interface()
rtde_ctrl_iface = get_control_interface()  


# def rnea(q, qd, qdd, g=[0,0,-9.81]):

#     for i in range(6):   # forward
#         R[i], p[i], pc[i] = dh_transform(...)
#         ω[i], α[i], a[i], ac[i] = fwd_recursion(...)
#         F[i] = m[i]*ac[i]
#         N[i] = J[i]@α[i] + np.cross(ω[i], J[i]@ω[i])

#     f = n = np.zeros((7,3))
#     τ = np.zeros(6)

#     for i in reversed(range(6)):  # backward

#         f[i] = R[i+1]@f[i+1] + F[i]
#         n[i] = N[i] + R[i+1]@n[i+1] + np.cross(pc[i],F[i]) \
#                + np.cross(p[i+1], R[i+1]@f[i+1])
#         τ[i] = n[i]@z + (Jm[i]+Jr[i])*qdd[i]

#     return τ



if __name__ == "__main__":

    pass

    # tau = rnea(q, qd, qdd)      # returns a 6‑element torque vector
