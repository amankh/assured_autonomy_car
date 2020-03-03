#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
#import rospy
#from std_msgs.msg import String
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from interval import interval, inf, imath#
#from transformations import *

#amankh
import rospy
import ackermann_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False


def executeFlowstar(counter):
    '''
        Function: Execute reachability computation by calling flow*
        Input: none
        Output: none
    '''
    #os.system('cd flowstar-2.1.0;'+'./flowstar < '+'model/RC_bicycle_template_new'+str(counter)+'.model')
    flow = list()
    #os.system('pwd')
    data = open('RC_bicycle'+str(counter)+'.plt', 'r').read()
    lines = data.split('\n\n\n')
    #print type(lines[0])
    for ll in lines:#one block is a rectange
        #print block.split('\n')
        block = ll.split('\n')
        #print block
        rectange_x = list()
        rectange_y = list() 
        for item in block:
            itemm = item.split(' ')
            if isfloat(itemm[0]) == True and isfloat(itemm[1]) == True:
                rectange_x.append(float(itemm[0]))
                rectange_y.append(float(itemm[1]))
        print rectange_x
        print rectange_y
        if len(rectange_x) > 0 and len(rectange_y) > 0:
            x_c = 0.5*(min(rectange_x)+max(rectange_x))
            y_c = 0.5*(min(rectange_y)+max(rectange_y))
            width = max(rectange_x)-min(rectange_x)
            height = max(rectange_y)-min(rectange_y)
            flow.append((x_c, y_c, width, height))  
    print flow           
executeFlowstar(0)
