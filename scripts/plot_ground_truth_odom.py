#!/usr/bin/env python3
"""
Plot ground truth odometry of a specified robot
"""
import argparse
import pathlib

import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot ground truth odometry of a specified robot')
    parser.add_argument('-d', '--dataroot', type=lambda p: pathlib.Path(p).absolute(), help='Kimera-Multi Data root')
    parser.add_argument('-s', '--sequence', type=str, default='1207', help="'1014' (campus outdoor) OR '1207' (tunnels) OR 1208 (hybrid)")
    parser.add_argument('-r', '--robot', type=str, default='acl_jackal', help="Name of robot to use")
    args = parser.parse_args()

    csvfile = args.dataroot / 'ground_truth' / args.sequence / f"{args.robot}_gt_odom.csv"
    
    #timestamp_kf,x,y,z,qw,qx,qy,qz
    D = np.loadtxt(csvfile, delimiter=',', skiprows=1)

    t = D[:,0] * 1e-9
    p = D[:,1:4]
    q = D[:,4:]
    
    fig, ax = plt.subplots()
    ax.plot(p[:,0], p[:,1])
    ax.axis('square')

    plt.show()