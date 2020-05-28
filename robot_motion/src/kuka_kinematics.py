#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This class is customized to match the functionality of the application.

import numpy as np
from numpy import sin, cos

def FK_kuka(q):
#
    FK = [0]*16
    # FK(0) = ((((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*cos(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]))*sin(q[6])
    # FK(1) = -((((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*cos(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]))*cos(q[6])
    # FK(2) = -(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5])
    # FK(3) = -0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0])
    # FK(4) = ((((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*cos(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]))*sin(q[6])
    # FK(5) = -((((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*cos(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]))*cos(q[6])
    # FK(6) = -(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5])
    # FK(7) = -0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]) - 0.42*sin(q[0])*sin(q[1])
    # FK(8) = (((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*cos(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-(sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[4]))*sin(q[6])
    # FK(9) = -(((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*cos(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-(sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[4]))*cos(q[6])
    # FK(10) = -((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5])
    # FK(11) = -0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]) + 0.36
    # FK(12) = 0
    # FK(13) = 0
    # FK(14) = 0
    # FK(15) = 1

    FK_ret = np.array(FK)
    np.reshape(FK_ret,(4,4))
    #
    return FK