import pylab
import matplotlib.ticker as ticker
import pytz, datetime, time
pacific = pytz.timezone('US/Pacific')
import tables
import numpy as np
import numpy
import cam_params
from scipy import linalg
import matplotlib.pyplot as plt
import sympy as sp
import time
import scipy
import scipy.optimize.optimize as optimize

import multiaxis_script as ma


def center(P):
    # there is also a copy of this in flydra.reconstruct, but included
    # here so this file doesn't depend on that.
    
    # P is Mhat
    orig_determinant = numpy.linalg.det
    def determinant( A ):
        return orig_determinant( numpy.asarray( A ) )
    # camera center
    X = determinant( [ P[:,1], P[:,2], P[:,3] ] )
    Y = -determinant( [ P[:,0], P[:,2], P[:,3] ] )
    Z = determinant( [ P[:,0], P[:,1], P[:,3] ] )
    T = -determinant( [ P[:,0], P[:,1], P[:,2] ] )

    C_ = numpy.array( [[ X/T, Y/T, Z/T ]] ).T
    return C_

def build_Bc(X3d,x2d):
    B = []
    c = []

    assert len(X3d)==len(x2d)
    if len(X3d) < 6:
        print 'WARNING: 2 equations and 11 unknowns means we need 6 points!'
    for i in range(len(X3d)):
        X = X3d[i,0]
        Y = X3d[i,1]
        Z = X3d[i,2]
        x = x2d[i,0]
        y = x2d[i,1]

        B.append( [X, Y, Z, 1, 0, 0, 0, 0, -x*X, -x*Y, -x*Z] )
        B.append( [0, 0, 0, 0, X, Y, Z, 1, -y*X, -y*Y, -y*Z] )

        c.append( x )
        c.append( y )
    return numpy.array(B), numpy.array(c)

def getMhat(data = None, data_file = None):
    
    if data_file is not None:
        data = np.loadtxt(  data_file,delimiter=',')

    # in terms of alpha
    x2d_alpha = data[:,0:2]
    x2d = np.tan(x2d_alpha)
    X3d = data[:,3:6]

    B,c = build_Bc(X3d,x2d)
    DLT_avec_results = numpy.linalg.lstsq(B,c)
    a_vec,residuals = DLT_avec_results[:2]
    a_vec = a_vec.T
    Mhat = numpy.array(list(a_vec)+[1])
    Mhat.shape=(3,4)
    
    print Mhat

    cam_id = 'newcam'
    print cam_id,center(Mhat).T,'residuals:',float(residuals)
    
    return Mhat
    
    

def decomp(Mhat):

    K,R,t = cam_params.decomp(Mhat)
    
    # t is NOT the camera center in world coordinates - use function center(Mhat) for that
    # P = [M | -Mt], M = KR
    
    return K,R,t
    
def getw2c(Mhat):

    K,R,t = decomp(Mhat)
    T = np.concatenate( (R, t), axis=1 )
    
    return T
