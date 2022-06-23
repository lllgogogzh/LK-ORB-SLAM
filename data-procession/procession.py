import rospy
import sys, select, os
import math
import tty, termios
import numpy as np


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    a=[' -0.000561235 \n','1.2\t']
    a=map(float,a)
    print(a)

    filename = sys.argv[1] #string
    f=open(filename,'r')
    row=f.readline()
    print(type(row))
    print(row.strip().split(' '))
    row=map(float,row)
    arrrow = np.array(row)
    print(arrrow)
    test = np.array([1,2,3])
    print(test)