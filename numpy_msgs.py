import numpy as np

import roslib; roslib.load_manifest(PKG)

import rospy
import numpy as np


def encode(array):

    dtype = array.dtype.name

    if dtype is 'float32':
        shape = array.shape
        
    
