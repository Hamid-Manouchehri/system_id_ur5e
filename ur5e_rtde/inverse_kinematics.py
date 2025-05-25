"""
*******************************************************************************

Project: system-id ur5e
File: inverse_kinematics.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 24, 2025

Description:
Python script calculating inverse kinematics of UR5e robot and deriving homogeneous 
transformation matrices.

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
# import os, csv, time, yaml, argparse
# from ur5e_rtde import get_receive_interface, get_control_interface
# from ur5e_rtde._modules import ur5e_homming, setup_configuration

# rtde_recv_iface = get_receive_interface()
# rtde_ctrl_iface = get_control_interface() 


def reformat_list(data, decimal_places):
  """
  Reformat a list of double numbers to a specified number of decimal places.

  Args:
    data: A list of double numbers.
    decimal_places: The number of decimal places to keep.

  Returns:
    A new list with the reformatted numbers.
  """
  return [round(number, decimal_places) for number in data]

# UR5e DH parameters (modified DH) from Table I :contentReference[oaicite:1]{index=1}
# Each entry: (a_i, alpha_i, d_i)
DH_PARAMS = [
    (   0.0,  np.pi/2, 0.1625 ),   # joint 1
    (  -0.425,  0.0,   0.0    ),   # joint 2
    (  -0.3922, 0.0,   0.0    ),   # joint 3
    (   0.0,   np.pi/2, 0.1333 ),   # joint 4
    (   0.0,  -np.pi/2, 0.0997 ),   # joint 5
    (   0.0,    0.0,   0.0996 ),   # joint 6
]

def dh_transform(a, alpha, d, theta):
    """Compute one link’s modified-DH homogeneous transform."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [   ct,   -st,    0,     a],
        [st*ca, ct*ca, -sa,  -d*sa],
        [st*sa, ct*sa,  ca,   d*ca],
        [    0,     0,    0,     1],
    ])

def forward_kinematics(q):
    """
    Compute the base→tool homogeneous transform for UR5e.
    :param q: array-like of 6 joint angles [rad]
    :return: 4×4 numpy array T06
    """
    T = np.eye(4)
    for (a, alpha, d), qi in zip(DH_PARAMS, q):
        T = T @ dh_transform(a, alpha, d, qi)
    return T

def inverse_kinematics(T06, tol=1e-6):
    """
    Analytic IK for UR5e (“wrist-partition”).
    Returns a list of up to 8 solutions [q1…q6] in radians.
    :param T06: 4×4 end-effector pose
    :param tol: singularity threshold
    """
    R06 = T06[:3,:3]
    p06 = T06[:3, 3]
    # wrist center
    d6 = DH_PARAMS[5][2]
    pw = p06 - d6 * R06[:,2]

    sols = []
    x, y, z = pw
    d1 = DH_PARAMS[0][2]
    a2, a3 = DH_PARAMS[1][0], DH_PARAMS[2][0]

    # q1 has two possibilities
    for sign1 in (+1, -1):
        # q1 = np.arctan2(y, x) + sign1 * np.arccos(d1 / np.hypot(x, y))
        q1 = sign1 * np.arccos(d1 / np.hypot(x, y))

        # compute q2, q3 via planar geometry
        r = np.hypot(x, y)
        s = z - d1
        D = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3)
        if abs(D) > 1: 
            continue
        for sign3 in (+1, -1):
            q3 = np.arctan2(sign3 * np.sqrt(1 - D*D), D)
            # q2
            phi = np.arctan2(s, r)
            psi = np.arctan2(a3 * np.sin(q3), a2 + a3 * np.cos(q3))
            q2 = phi - psi

            # compute R03
            T03 = np.eye(4)
            for (a, alpha, d), qi in zip(DH_PARAMS[:3], (q1, q2, q3)):
                T03 = T03 @ dh_transform(a, alpha, d, qi)
            R03 = T03[:3,:3]
            R36 = R03.T @ R06

            # q5 (two possibilities)
            q5 = np.arctan2(np.hypot(R36[0,2], R36[1,2]), R36[2,2])
            if abs(np.sin(q5)) < tol:
                # singular: collapse q4+q6
                q4 = 0
                q6 = np.arctan2(-R36[1,0], R36[0,0])
                sols.append([q1, q2, q3, q4, q5, q6])
            else:
                q4 = np.arctan2(R36[1,2], R36[0,2])
                q6 = np.arctan2(R36[2,1], -R36[2,0])
                sols.append([q1, q2, q3, q4, q5, q6])

    return sols

def wrap(x):
    return (x + np.pi) % (2*np.pi) - np.pi

# Example usage
if __name__ == "__main__":
    # choose a test configuration
    q_test = [-0.00242, -1.56754, 0.00979, -1.5565, -0.01208, -0.00371]
    q_test = np.array([wrap(a) for a in q_test])
    print("actual joint angles(rad):\n", q_test, "\n")

    T = forward_kinematics(q_test)
    print("Forward FK → T06 =\n", T)

    ik_sols = inverse_kinematics(T)
    print("\nInverse solutions (rad):")
    ik_sols = [[wrap(a) for a in sol] for sol in ik_sols]
    for sol in ik_sols:
        print(reformat_list(sol, 3))





