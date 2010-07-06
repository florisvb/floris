import camera_math
import ptf_camera_classes as ptf
import pickle
import numpy as np
import mpl_toolkits.mplot3d.axes3d as axes3d
import matplotlib.pyplot as plt
import sys

sys.path.append("/home/floris/src/flydra/flydra")
import reconstruct


class Calibration:

    def __init__(self):
        
        self.camera_center = [0,0,0]
        self.focus = ptf.FocusMotor()


    def load_calibration_data(self, filename=None):
            if filename is None:
                print 'NEED FILENAME'
            fname = (filename)
            fd = open( fname, mode='r')
            print
            print 'loading calibration... '
            self.data = pickle.load(fd)
            
    def load_flydra_cameras(self, flydra_calibration_xml):
        
        self.flydra_calibration = reconstruct.Reconstructor(flydra_calibration_xml)
            
    def calibrate(self):
        
        self.Mhat, residuals = camera_math.getMhat(self.data)
        self.camera_center = np.array(camera_math.center(self.Mhat).T[0])
        print '*'*80
        print 'DLT camera center: ', self.camera_center
        print 'Mhat: '
        print self.Mhat
        self.Mhatinv = np.linalg.pinv(self.Mhat)
        #self.Mhat3x3inv = np.linalg.inv(self.Mhat[:,0:3]) # this is the left 3x3 section of Mhat, inverted
        #self.Mhat3x1 = self.Mhat[:,3] # this is the rightmost vertical vector of Mhat
            
        self.focus.calibrate(data=self.data, camera_center=self.camera_center)
        
        return        
        
    def plot(self):
    
        fig = plt.figure(0)
        ax = axes3d.Axes3D(fig)
        ax.scatter3D(self.data[:,3], self.data[:,4], self.data[:,5])
        
        # principal point for ptf:
        princ_pt = self.Mhat[2,0:3]
        print princ_pt, self.camera_center
        ax.plot( [self.camera_center[0], princ_pt[0]], [self.camera_center[1], princ_pt[1]], [self.camera_center[2], princ_pt[2]])
        
        # flydra cameras
        
        
        # axes 3d will not allow plotting a single point - so append the origin as hack  
        hack = np.zeros([2,3])
        hack[0,:] = self.focus.camera_center
        hack[1,:] = self.camera_center
        ax.scatter3D(hack[:,0], hack[:,1], hack[:,2])
        
        ax.set_xlabel('flydra x coordinates, meters')
        ax.set_ylabel('flydra y coordinates, meters')
        ax.set_zlabel('flydra z coordinates, meters')
        
        
if __name__=='__main__':
    cal = Calibration()
    cal.load_calibration_data('/home/floris/src/floris/ptf_calibration_20100706_104637')
    cal.load_flydra_cameras('/home/floris/calibrations/20100703_2/20100703_cal_scaled_3cams.xml')
    cal.calibrate()
    cal.plot()