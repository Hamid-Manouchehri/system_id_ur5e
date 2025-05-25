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


# UR5 DH parameters (Craig’s modified DH) from Table I :contentReference[oaicite:1]{index=1}
DH_PARAMS = [
    # (a_i,    alpha_i,       d_i,      theta_i,    variable name)
    (0.0,      np.pi/2,    0.1625,      'q1'),
    (-0.425,   0.0,        0.0,         'q2'),
    (-0.39225, 0.0,        0.0,         'q3'),
    (0.0,      np.pi/2,    0.1333,      'q4'),
    (0.0,     -np.pi/2,    0.0997,      'q5'),
    (0.0,      0.0,        0.0996,      'q6'),
]

def dh_transform(a, alpha, d, theta):
    """One-link DH transform (base→link)"""
    sa, ca = np.sin(alpha), np.cos(alpha)
    st, ct = np.sin(theta), np.cos(theta)
    return np.array([
        [   ct,   -st,    0,      a],
        [st*ca, ct*ca, -sa,  -d*sa],
        [st*sa, ct*sa,  ca,   d*ca],
        [   0,      0,    0,      1],
    ])

def forward_kinematics(q):
    """
    Compute full 0→6 transformation T06 given joint angles q=[q1..q6].
    Returns T06 as a 4×4 homogeneous matrix.
    """
    T = np.eye(4)
    for (a, alpha, d, _), qi in zip(DH_PARAMS, q):
        T = T @ dh_transform(a, alpha, d, qi)
    return T

def inverse_kinematics(T06):
    """
    Analytic inverse kinematics for UR5.
    Input:  4×4 end-effector pose T06.
    Output: list of up to 8 solutions [q1..q6], in radians.
    """
    # Extract R and p
    R06 = T06[:3,:3]
    p06 = T06[:3,3]
    # wrist center
    d6 = DH_PARAMS[5][2]
    p05 = p06 - d6 * R06[:,2]
    x, y, z = p05

    sols = []
    # θ1: two solutions
    for sign1 in [+1, -1]:
        q1 = np.arctan2(y, x) + sign1 * np.arccos(DH_PARAMS[0][2] / np.hypot(x, y)) + np.pi / 2.0
        # prepare for q2, q3
        a2, a3 = DH_PARAMS[1][0], DH_PARAMS[2][0]
        # r, s in plane 1→2→3
        r = np.hypot(x, y)
        s = z - DH_PARAMS[0][2]
        # law of cosines for θ3
        D = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3)
        if abs(D) > 1: continue
        for sign3 in [+1, -1]:
            q3 = np.arctan2(sign3 * np.sqrt(1 - D*D), D)
            # θ2
            phi = np.arctan2(s, r)
            psi = np.arctan2(a3 * np.sin(q3), a2 + a3 * np.cos(q3))
            q2 = phi - psi
            # compute R03
            q123 = [q1, q2, q3]
            T03 = np.eye(4)
            for (a, alpha, d, _), qi in zip(DH_PARAMS[:3], q123):
                T03 = T03 @ dh_transform(a, alpha, d, qi)
            R03 = T03[:3,:3]
            R36 = R03.T @ R06
            # θ4, θ5, θ6 from R36
            # avoid singularities: sin(q5)=0
            q5 = np.arctan2(np.hypot(R36[0,2], R36[1,2]), R36[2,2])
            if abs(np.sin(q5)) < 1e-6:
                # singular: force one solution
                q4 = 0
                q6 = np.arctan2(-R36[1,0], R36[0,0])
            else:
                q4 = np.arctan2(R36[1,2], R36[0,2])
                q6 = np.arctan2(R36[2,1], -R36[2,0])
            sols.append([q1, q2, q3, q4, q5, q6])
    return sols

# Example usage:
if __name__ == "__main__":
    # some test joint angles
    q_test = [1.40707, -1.31538, 1.61768, -1.85295, 4.70799, -3.28426]  # TODO
    print("actual ur5e joint angles(rad):\n", q_test, "\n")
    T06 = forward_kinematics(q_test)
    print("T06 from FK:\n", T06)

    ik_sols = inverse_kinematics(T06)
    print("\nIK solutions (rad):")
    for sol in ik_sols:
        print(reformat_list(sol, 3))




