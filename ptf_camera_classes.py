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
        if self.motor == 'tilt': self.ps3_gain = -0.02
        if self.motor == 'pan': self.ps3_gain = -0.02 
        self.home = 0
        
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
        self.coeffs = [-3.83926152e-07,   1.33637230e+00,  -2.97889319e+00]

    def calibrate(self, data=None, camera_center=None, filename=None):
    
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
            print 'no camera center given, calculating...'
            Mhat, residuals = camera_math.getMhat(self.data)
            camera_center = np.array(camera_math.center(Mhat).T[0])
        else:
            self.camera_center = camera_center
            self.original_camera_center = camera_center
        
        #fd = open( filename, mode='r')
        #print 'loading calibration... '
        #self.data = pickle.load(fd)
        #self.original_camera_center = [ 0.26387096,  2.01547775, -6.21817195]
        
        tmp = scipy.optimize.fmin( self.fmin_func, self.original_camera_center, full_output = 1, disp=0)
        self.camera_center = tmp[0]
        print 'old camera center: ', self.original_camera_center
        print 'new camera center: ', self.camera_center
        
        
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])
        
        tmp = scipy.optimize.fmin( self.focus_fmin_func, self.coeffs, full_output = 1, disp=0)
        self.coeffs = tmp[0]
        
        print 'coefficients: ', self.coeffs
        
        
        fig = None
        if 1:
            if fig is None:    
                fig = plt.figure(1)
            #print distc, focus
            plt.scatter(self.distc,self.focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.calc_focus(x) for x in xi]
            
            plt.title('Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            plt.show()
            print 'plotted focus'

        fig.show()        
        return 1
        
        
    def calc_distc(self,pos_3d):
        distc = np.linalg.norm( pos_3d-self.camera_center ) # (might need to check orientation of vectors)
        return distc
        
    def calc_focus(self, distc):
        #focus_pos = distc*self.coeffs[0] + self.coeffs[1]
        focus_pos = (self.coeffs[1]**(distc+self.coeffs[0])) + self.coeffs[2]
        #print focus_pos, distc, self.coeffs
        return focus_pos
        
    def calc_distc_from_focus(self, focus_pos):
        #distc = (focus_pos - self.coeffs[1]) / self.coeffs[0]
        distc = np.log(focus_pos - self.coeffs[2]) / np.log(self.coeffs[1]) - self.coeffs[0] 
        return distc
        
    def fmin_func(self,camera_center):
        self.camera_center=camera_center
        print camera_center
        # find the distances to the camera center
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])
            
        # get a polyfit for focus pos vs distc
        self.focus = self.data[:,2]
        #self.coeffs = np.polyfit(distc, focus, 1)
        tmp = scipy.optimize.fmin( self.focus_fmin_func, self.coeffs, full_output = 1, disp=0)
        self.coeffs = tmp[0]

        
        # recalculate distc from the polyfit inverse
        new_distc = self.calc_distc_from_focus(self.focus)
        
        # take difference btwn the distc and the recalculated distcs: this the the thing to minimize
        dist_diff = np.sum( np.abs( new_distc - self.distc ) )
        
        center_diff = np.sum( np.abs( camera_center - self.original_camera_center ))
        
        #print 'camera center: ', camera_center, 'dist diff: ', dist_diff
        print dist_diff
        
        return dist_diff+center_diff*.01

    def focus_fmin_func(self,coeffs):
        self.coeffs = coeffs
        f = [self.calc_focus(d) for d in self.distc]
        err = np.sum( np.abs( f-self.focus ))
        return err  
            
            
            
        # find the distances to the camera center
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])
            
        # get a polyfit for focus pos vs distc
        self.focus = self.data[:,2]
        #self.coeffs = np.polyfit(distc, focus, 1)
        tmp = scipy.optimize.fmin( self.focus_fmin_func, self.coeffs, full_output = 1, disp=0)
        self.coeffs = tmp[0]

        
        # recalculate distc from the polyfit inverse
        new_distc = self.calc_distc_from_focus(self.focus)
        
        # take difference btwn the distc and the recalculated distcs: this the the thing to minimize
        dist_diff = np.sum( np.abs( new_distc - self.distc ) )
        center_diff = np.sum( np.abs( camera_center - self.original_camera_center ))
        err = dist_diff + center_diff*.001      
        print 'calculating camera center, error: ', new_distc
        
        return err

    def focus_fmin_func(self,coeffs):
        self.coeffs = coeffs
        f = [self.calc_focus(d) for d in self.distc]
        err = np.sum( np.abs( f-self.focus ))
        return err  
            
            
