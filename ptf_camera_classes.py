import numpy as np
import pickle
import scipy.optimize
import matplotlib.pyplot as plt
import camera_math

class PTMotor:
    def __init__(self, motor):
    
        self.motor = motor
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        self.center_displacement = 0
        self.angular_displacement = 0
        if self.motor == 'tilt': self.ps3_gain = -0.01
        if self.motor == 'pan': self.ps3_gain = -0.01
        self.home = 0
        self.latency = 0.01
        
class FocusMotor:
    def __init__(self):
    
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        self.ps3_gain = 0.05
        self.home = 0
        
        # motor calibration values
        self.original_camera_center=[0,0,0]
        self.camera_center=[0,0,0]
        self.coeffs = [1,0,0]
        self.latency = 0.01

    def calibrate(self, data=None, camera_center=None, filename=None, plot=False):
        
        if data is not None:
            self.data = data

        if data is None:
            if filename is None:
                print 'NEED FILENAME, or DATA!!'
            if filename is not None:
                fd = open( filename, mode='r')
                print
                print 'loading calibration... '
                self.data = pickle.load(fd)
                
        if camera_center is None:
            raise ValueError( 'no camera center given...')
        else:
            self.camera_center = camera_center
            
            
        # find focus - get polyfit for sqrt(distc), use that as seed
        self.focus = self.data[:,2]
        self.focus_polyfit(power=0.5)
        tmp = scipy.optimize.fmin( self.focus_fmin_func, self.coeffs, full_output = 1, disp=0)
        #coeffs = [self.camera_center[0], self.camera_center[1], self.camera_center[2]]
        #for i in self.coeffs:
        #    coeffs.append(i)
        #tmp = scipy.optimize.fmin( self.fmin_camera_center_func, coeffs, full_output = 1, disp=0)
        #self.camera_center = tmp[0][0:3]
        #self.coeffs = tmp[0][3:6]
        #print
        #print 'final error: '
        #print tmp[1]
        self.coeffs = tmp[0]
        print 'coefficients: '
        print self.coeffs
        print 'camera center: '
        print self.camera_center
            
        fig = None
        if plot:
            if fig is None:    
                fig = plt.figure()
            #print distc, focus
            self.focus_fmin_func(self.coeffs)
            plt.scatter(self.distc,self.focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.calc_focus(distc=x) for x in xi]
            
            plt.title('Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            plt.show()
            print 'plotted focus'
            
    def fmin_camera_center_func(self, coeffs):
        self.camera_center = coeffs[0:3]
        focus_coeffs = coeffs[3:6]
        tmp = scipy.optimize.fmin( self.focus_fmin_func, focus_coeffs, full_output = 1, disp=0)
        print 'camera_center: ', self.camera_center, tmp[1]
        return tmp[1]

        
    def calc_distc(self,pos_3d,factor=1):
        if len(pos_3d) == 4:
            pos_3d = pos_3d[0:3] # in case we're being sent an adjusted pos
        distc = np.linalg.norm( pos_3d-self.camera_center ) # (might need to check orientation of vectors)
        return distc
        
    def calc_focus(self, pos_3d=None, distc=None):
        if pos_3d is not None:
            distc = self.calc_distc(pos_3d)
        #focus_pos = distc*self.coeffs[0] + self.coeffs[1]
        #focus_pos = self.coeffs[1]*distc**self.coeffs[0]+self.coeffs[2] 
        focus_pos = self.coeffs[0]*np.log(distc+self.coeffs[1])+self.coeffs[2]
        #(self.coeffs[1]**(distc+self.coeffs[0])) + self.coeffs[2]
        #print focus_pos, distc, self.coeffs
        return focus_pos
        
    def calc_distc_from_focus(self, focus_pos):
        distc = np.exp((focus_pos-self.coeffs[2])/self.coeffs[0])-self.coeffs[1]
        #distc = (focus_pos - self.coeffs[1]) / self.coeffs[0]
        #distc = ((focus_pos-self.coeffs[2])/self.coeffs[1])**(1.0/self.coeffs[0])
        #np.log(focus_pos - self.coeffs[2]) / np.log(self.coeffs[1]) - self.coeffs[0] 
        return distc
        
    def focus_polyfit(self, power=0.5):
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])**power   
        tmp = np.polyfit(self.distc, self.focus, 1)
        #self.coeffs[0] = power
        self.coeffs[1] = tmp[0]
        self.coeffs[2] = tmp[1]
        print 'seed coeffs: '
        print self.coeffs

    def focus_fmin_func(self,coeffs):
        self.coeffs = coeffs
        
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])        
        f = [self.calc_focus(distc=d) for d in self.distc]
        err = np.sum( np.abs( f-self.focus ))
        return err  
            
