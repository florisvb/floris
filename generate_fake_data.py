import numpy as np
import camera_math
import cvNumpy
import cv

class FakeData:
    
    def __init__(self):
        print 'initializing'

    def to_motor_coords(self, obj_pos):
        # takes 3D object as input, returns the three corresponding motor positions
        # back out desired motor positions
        if len(obj_pos) == 3:
            obj_pos = np.hstack((obj_pos, [1]))
        obj_pos = np.asarray(obj_pos)
        #print obj_pos, self.Mhat.shape
        q = np.dot(self.Mhat,obj_pos.T)
        r = q[0] # camera coord x
        s = q[1] # camera coord y
        t = q[2] # camera coord z
        v = s/t
        u = r/t 
        distc = self.calc_distc(obj_pos)
        #distc = np.linalg.norm(q) # equivalent to above function call
        #print 'pos_3d: ', obj_pos
        #print 'distc: ', r,s,t
        pan_pos = np.arctan2(u,1) # focal length of 1, arbitrary
        tilt_pos = np.arctan2(v,1)
        focus_pos = 0#self.focus.calc_focus(obj_pos)
        motor_coords = [pan_pos, tilt_pos, focus_pos]
        #print motor_coords

        return motor_coords


    def calc_distc(self,pos_3d):
        #print pos_3d, self.camera_center
        distc = scipy.linalg.norm( pos_3d[0:3]-self.camera_center ) # (might need to check orientation of vectors)
        
        return distc

    def calibrate(self):

        K = cvNumpy.array_to_mat(np.eye(3))
        rvec_np = np.random.random(3)
        rvec = cvNumpy.array_to_mat(rvec_np)
        tvec_np = np.random.random(3)
        tvec = cvNumpy.array_to_mat(tvec_np)
        distCoeffs = cvNumpy.array_to_mat(np.zeros(4))

        # convert rotation vector to rotation matrix
        rmat = cvNumpy.array_to_mat(np.zeros([3,3]))
        cv.Rodrigues2( rvec, rmat )

        # generate P matrix
        self.R = cvNumpy.mat_to_array(rmat)
        self.t = cvNumpy.mat_to_array(tvec)
        self.K = cvNumpy.mat_to_array(K)
        Rt = np.hstack((self.R,self.t.T))
        self.P = np.dot( self.K, Rt )
        self.Mhat = self.P
        self.camera_center = camera_math.center(self.P)[:,0]

        # inverse Mhat:
        self.Mhatinv = np.linalg.pinv(self.Mhat)


        # generate fake 3D data:
        fake3d = np.random.random([10,3])
        fake2d = np.zeros([10,2])
        for i in range(np.shape(fake3d)[0]):
            p,t,f = self.to_motor_coords(fake3d[i,:])
            fake2d[i,:] = [p,t]


        ################# calibration ################3
        
        K_new = cvNumpy.array_to_mat(np.eye(3))
        rvec_np_new = np.random.random(3)
        rvec_new = cvNumpy.array_to_mat(rvec_np)
        tvec_np_new = np.random.random(3)
        tvec_new = cvNumpy.array_to_mat(tvec_np)
        distCoeffs_new = cvNumpy.array_to_mat(np.zeros(4))

        points3D = cvNumpy.array_to_mat(np.asarray( fake3d ))
        points2D = cvNumpy.array_to_mat(np.asarray( np.tan( fake2d ) ))

        cv.FindExtrinsicCameraParams2(points3D, points2D, K, distCoeffs, rvec, tvec)

        tvec_np_new = cvNumpy.mat_to_array(tvec)
        rvec_np_new = cvNumpy.mat_to_array(rvec)
        
        print 'original tvec: ', tvec_np
        print 'new tvec: ', tvec_np_new
        print 
        print 'original rvec: ', rvec_np
        print 'new rvec: ', rvec_np_new
        
