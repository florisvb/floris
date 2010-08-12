import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import scipy.optimize
import time

# calculate angle required to hit a target
# equations from: http://www.grc.nasa.gov/WWW/K-12/airplane/flteqs.html
        
class Mortar:
    def __init__(self, g=9.81, vt=15.24, v0=91.44, x0=0, y0=0):
        # environment and projectile constants:
        self.g = g
        self.vt = vt
        self.v0 = v0
        
        # mortar constants
        self.x0 = x0
        self.y0 = y0
        
        self.calc_max_air_time()
        
    def calc_max_air_time(self):
        
        t,x,y = self.ballistic_trajectory(89.9*np.pi/180, max_t=100, resolution=1000, ground=0, search_ground=True)
        self.maxairtime = t[-1]

    def ballistic_trajectory(self, theta, max_t=3, resolution=500, ground=0., speed=None, search_ground=False):
        
        t = np.linspace(0,max_t,1000)
        vy0 = self.v0*np.sin(theta)
        vx0 = self.v0*np.cos(theta)
        vy = (vy0/self.vt - np.tan(self.g * t / self.vt)) / (1.0 + (vy0/self.vt) * np.tan (self.g * t / self.vt)) * self.vt
        y = (self.vt**2 / (2*self.g) ) * np.log( (vy0**2 + self.vt**2) / ( vy**2 + self.vt**2) )+self.y0
        x = (self.vt**2 / (self.g) ) * np.log( (self.vt**2 + self.g*vx0*t) / (self.vt**2) )+self.x0
        
        # search for where the projectile hits the ground:
        ground_index = -1
        if search_ground:
            for i, yval in enumerate(y):
                if (yval-ground) < 0 and i > 2:
                     ground_index = i
                     break
    
        return t[0:ground_index],x[0:ground_index],y[0:ground_index]
    
    def ballistic_impact(self, theta, t_impact = 1 ):
        # constants for the ball
        vy0 = self.v0*np.sin(theta)
        vx0 = self.v0*np.cos(theta)
        t = t_impact
        vy = (vy0/self.vt - np.tan(self.g * t / self.vt)) / (1.0 + (vy0/self.vt) * np.tan (self.g * t / self.vt)) * self.vt
        vx = self.vt**2 * vx0 / (self.vt**2 + self.g * vx0 * t) 
        return np.sqrt(vx**2+vy**2)

    def fmin_func(self, theta, xdes, ydes):
        t,x,y = self.ballistic_trajectory ( theta, max_t=self.maxairtime )
        ix = np.argmin(np.abs(x-xdes))
        xval = x[ix]
        yval = y[ix]
        err = np.abs(yval-ydes)
        #print err, xval, yval
        return err
    
    def calc_theta(self, xdes, ydes, theta0=45*np.pi/180, ftol = 0.01, verbose=0, chart=False):
        if verbose:
            t0 = time.time()
        if chart:
            print 'used chart'
            theta0, tguess, errguess = self.chart.calc_angle(xdes,ydes)
            
        f = scipy.optimize.fmin(self.fmin_func, theta0, args=(xdes, ydes), disp=0, ftol=ftol, full_output=1)
        theta_opt = f[0][0]
        err = f[1]
            
        if verbose:
            print 'calculation time: ', time.time()-t0
            t,x,y = self.ballistic_trajectory (theta_opt, max_t=self.maxairtime)
            ix = np.argmin(np.abs(x-xdes))
            error = err
            t_impact = t[ix]
            speed = self.ballistic_impact(theta_opt, t_impact)  
            plt.figure(0)
            plt.plot(x,y)
            plt.plot(xdes, ydes, '*')
            print 
            print 'theta: ', theta_opt*180/np.pi, ' deg'
            print 'time to impact: ', t_impact, ' sec'
            print 'speed at impact: ', speed, ' m/s'
            print 'error: ', err, ' m'
            if err > ftol:
                print 'NOTE: error > ', ftol, ' m'
            print
            return theta_opt, err, t, x, y, t_impact

        else:
             return theta_opt
             
     
    
if __name__ == '__main__':
    
    # initialize the mortar/projectile
    mortar = Mortar(g=9.81, vt=15.24, v0=91.44, x0=0, y0=0)
    
    # calculate angles etc.
    xf = 26
    yf = 23
    theta, err, t, x, y, t_impact = mortar.calc_theta(xf, yf, theta0=45*np.pi/180, verbose=1, ftol=0.05)
    
    




