import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d
import pickle
import scipy.optimize

class Ray:

    def __init__(self, data=None, filename=None):
    
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
    
        self.points_3d = self.data[:,3:6]
        self.fit_line()
        
        # get a vector description:
        if 1:
            p1 = np.array(self.get_line(0,'t'))
            p2 = np.array(self.get_line(1,'t'))
            vec = p2-p1
            self.vector = vec / np.linalg.norm(vec)
        
    def parameterize_line(self, points=None):
    
        if points == None:
            points = self.points_3d
    
        # x = a1*t + b1
        # y = a2*t + b2
        # z = a3*t + b3
        # let t = x -> a1 = 1, b1 = 0
        self.t = np.linspace(0,1,len(points[:,0]))
        self.x_fit = np.polyfit(self.t, points[:,0], 1)
        self.y_fit = np.polyfit(self.t, points[:,1], 1)
        self.z_fit = np.polyfit(self.t, points[:,2], 1)
        
    def fit_line_lstsq(self):
    
        # before parameterizing, get a least squares fit!
        yz = self.points_3d[:,1:3]
        x = self.points_3d[:,0]
        x = x.reshape(len(x), 1)
        x1 = np.hstack( ( x, np.ones([len(x), 1]) ) )
        m = np.linalg.lstsq(x1, yz)[0]
        self.m = m
        
        # get corrected points
        yz = np.dot(x1, self.m).reshape(len(yz), 2)
        self.points_fixed = np.hstack( [x, yz] )
        
        return m

    def fit_line(self):
    
        m = self.fit_line_lstsq()
        
        self.parameterize_line(points = self.points_fixed)
        
    
    def get_line(self,v,var):
    
        if var is 'x':
            t = (v-self.x_fit[1]) / self.x_fit[0]
        elif var is 'y':
            t = (v-self.y_fit[1]) / self.y_fit[0]
        elif var is 'z':
            t = (v-self.z_fit[1]) / self.z_fit[0]
        elif var is 't':
            t = v
        
        x = self.x_fit[0]*t + self.x_fit[1]
        y = self.y_fit[0]*t + self.y_fit[1]
        z = self.z_fit[0]*t + self.z_fit[1]
        pt = [x, y, z]
        return pt
        
    def plot(self, fig = None, ax = None):
    
        if fig is None:
            fig = plt.figure(0)
        if ax is None:
            ax = axes3d.Axes3D(fig)
    
        # raw data
        ax.scatter3D(self.points_3d[:,0],self.points_3d[:,1],self.points_3d[:,2])
        
        # least square data
        ax.scatter3D(self.points_fixed[:,0],self.points_fixed[:,1],self.points_fixed[:,2])

        # fitted line
        t = np.linspace(0,1,10)
        
        pt = np.zeros([len(t), 3])
        for i, enum in enumerate(t):
            pt[i,:] = self.get_line(enum,'t')
        ax.plot3D(pt[:,0],pt[:,1],pt[:,2])
        
        return ax
        
def test_linemath():

    x = np.linspace(-1,1,10)
    y = np.zeros_like(x)
    z = x
    
    data = np.zeros([len(x), 6])
    data[:,3] = x
    data[:,4] = y
    data[:,5] = z
    
    ray = Ray(data=data)
    try:
        ray.plot()
    except:
        pass
    return ray
    
    
    
    
    
        
        
        
        
        
        
        
