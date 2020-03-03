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

def executeFlowstar(counter):
    '''
        Function: Execute reachability computation by calling flow*
        Input: none
        Output: none
    '''
    #os.system('cd flowstar-2.1.0;'+'./flowstar < '+'model/RC_bicycle_template_new'+str(counter)+'.model')
    flow = list()
    #os.system('pwd')
    with open('RC_bicycle'+str(counter)+'.plt') as f:
        f.split('\n\n')
        print f 
executeFlowstar(0)
