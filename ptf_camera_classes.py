import numpy as np
import pickle
import scipy.optimize
import matplotlib.pyplot as plt
import camera_math
import linemath
import scipy.interpolate as interp

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
        #self.focus_polyfit(power=0.5)
        self.coeffs = [0,0,0]
        
        
        
        # for static case: need to find the ray of the calibration points
        # then find the point along that ray that will best work as the camera center..
        
        
        
        
        
        
        
        
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
            
    def calibrate_static(self, data=None, filename=None, plot=False, guess=0):
    
        self.focusstyle = 'exp'
        style = self.focusstyle
        
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
                
        # ray = a line fitted to the data points given
        self.ray = linemath.Ray(data=self.data)
            
        def fmin_focus(coeffs):
            self.coeffs = coeffs
            self.distc = np.zeros(np.shape(self.data)[0])
            for i in range(len(self.distc)):
                self.distc[i] = self.calc_distc(self.data[i,3:6], static=False)        
            f = [self.calc_focus(distc=d, style=style) for d in self.distc]
            err = np.sum( np.abs( f-self.focus ))
            #print err
            return err  
            
        def fmin_camera_center(camera_center_seed):
            self.camera_center = self.ray.get_line(camera_center_seed[0], 't')
            #print 'camera_center straight from function: ', self.camera_center
            #self.focus_polyfit(power=0.5)
            self.coeffs = [1, 1, 0]
            tmp = scipy.optimize.fmin( fmin_focus, self.coeffs, full_output = 1, disp=0)
            self.coeffs = tmp[0]
            print self.coeffs
            return tmp[1] # error
            
        # find focus - get polyfit for sqrt(distc), use that as seed
        self.focus = self.data[:,2]
        camera_center_seed = guess
        self.camera_center = self.ray.get_line(camera_center_seed, 't')
        tmp = scipy.optimize.fmin( fmin_camera_center, camera_center_seed, full_output = 1, disp=0)
        camera_center_seed = tmp[0]
        final_error = fmin_camera_center(camera_center_seed)
        
        print 'coefficients: '
        print self.coeffs
        print 'camera center: '
        print self.camera_center
        print 'final error: '
        print final_error
        
        fmin_focus(self.coeffs)
        self.interp_distc = self.distc
        self.interp_focus = self.focus
        self.interp = interp.interp1d(self.interp_distc, self.interp_focus, kind='linear', bounds_error=False)

            
        fig = None
        if plot:
            if fig is None:    
                fig = plt.figure()
            #print distc, focus
            plt.scatter(self.distc,self.focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.calc_focus(distc=x, style='exp') for x in xi]
            
            plt.title('Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            plt.show()
            print 'plotted focus'
            
            # plot raw points and ray
            self.ray.plot()
            
            
    def fmin_camera_center_func(self, coeffs):
        self.camera_center = coeffs[0:3]
        focus_coeffs = coeffs[3:6]
        tmp = scipy.optimize.fmin( self.focus_fmin_func, focus_coeffs, full_output = 1, disp=0)
        print 'camera_center: ', self.camera_center, tmp[1]
        return tmp[1]

        
    def calc_distc(self,pos_3d,factor=1, static=False):
        if len(pos_3d) == 4:
            pos_3d = pos_3d[0:3] # in case we're being sent an adjusted pos
        if static is False:
            distc = np.linalg.norm( pos_3d-self.camera_center ) # (might need to check orientation of vectors)
        elif static is True:
            vec = pos_3d-self.camera_center
            #print 'vec: ', pos_3d, self.camera_center
            #print 'ray vector: ', self.ray.vector
            distc = np.abs(np.dot(vec, self.ray.vector))
        
        return distc
        
    def calc_focus(self, pos_3d=None, distc=None, static=False, style=None):
        if style is None:
            try:
                style = self.focusstyle
            except:
                self.focysstyle = 'log'
                style = 'log'            
            
        if pos_3d is not None:
            distc = self.calc_distc(pos_3d, static=static)
        #focus_pos = distc*self.coeffs[0] + self.coeffs[1]
        #focus_pos = self.coeffs[1]*distc**self.coeffs[0]+self.coeffs[2] 
        if style == 'log':
            focus_pos = self.coeffs[0]*np.log(distc+self.coeffs[1])+self.coeffs[2]
        if style == 'exp':
            focus_pos = self.coeffs[0]*np.exp(distc**self.coeffs[1])+self.coeffs[2]
        if style == 'exponential':
            focus_pos = self.coeffs[0]*distc**(self.coeffs[1])+self.coeffs[2]
        if style == '1/x':
            focus_pos = self.coeffs[0]/(distc+self.coeffs[1])+self.coeffs[2]
        if style == 'interp':
            focus_pos = self.interp(distc)
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
        self.coeffs[0] = 1
        self.coeffs[1] = tmp[0]
        self.coeffs[2] = tmp[1]

    def focus_fmin_func(self,coeffs):
        self.coeffs = coeffs
        
        self.distc = np.zeros(np.shape(self.data)[0])
        for i in range(len(self.distc)):
            self.distc[i] = self.calc_distc(self.data[i,3:6])        
        f = [self.calc_focus(distc=d) for d in self.distc]
        err = np.sum( np.abs( f-self.focus ))
        print err
        return err  
            
