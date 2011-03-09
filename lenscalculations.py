# Floris van Breugel 
# March 8, 2010
# California Institute of Technology

# Resources:
#   http://www.dofmaster.com/equations.html


import numpy as np
import sympy as sp
import matplotlib.pyplot as plt


'''
Basler Ace:
pixel_pitch = .0056
sensor_height = 2.4
'''

# to aid in finding the correct sensor height use the following function / dictionary
# http://en.wikipedia.org/wiki/Image_sensor_format
def get_sensor_height_from_sensor_type(sensor_type):
    sensor_dict = { '1/4': 2.4, '1/3': 3.6, '1/2': 4.8, '2/3': 6.6, '4/3': 13., 'APS-C': 14.8, 'APS-H': 18.6, '35mm': 24. } 
    try:
        return sensor_dict[sensor_type]    
    except:
        print 'sensor_type unrecognized'
        return sensor_dict
        
## NOTES:

# all calculations done with a square sensor: essentially assume the rectangle is cropped to a square
# field of view calculations are done on the width/height of the square, not the diagonal
        
class Camera:
    # all units should always be in mm
    def __init__(self, focal_length, pixel_pitch, sensor_height, aperture, circle_of_confusion_type='pixels', 
                        circle_of_confusion_pixel_factor=2., circle_of_confusion_zeiss_factor=1730., circle_of_confusion_arbitrary=.025):
        self.focal_length = focal_length # mm
        self.pixel_pitch = pixel_pitch # mm
        self.sensor_size = sensor_height # mm, recommended to use square sensor for calculations. use width, not diagonal.
        self.aperture = aperture
        
        # circle of confusion paramters
        self.circle_of_confusion_type = circle_of_confusion_type
        self.circle_of_confusion_pixel_factor = circle_of_confusion_pixel_factor
        self.circle_of_confusion_zeiss_factor = circle_of_confusion_zeiss_factor
        self.circle_of_confusion_arbitrary = circle_of_confusion_arbitrary
        
        self.calc_dependent_parameters()
        
    def calc_dependent_parameters(self):
        if self.circle_of_confusion_type == 'pixels':
            self.circle_of_confusion = self.circle_of_confusion_pixel_factor*self.pixel_pitch # intuitive measure: based on the actual pixel size
        elif self.circle_of_confusion_type == 'zeiss':
            self.circle_of_confusion = self.sensor_width*np.sqrt(2) / self.circle_of_confusion_zeiss_factor # 'zeiss formula': http://en.wikipedia.org/wiki/Zeiss_formula
        elif self.circle_of_confusion_type == 'arbitrary':
            self.circle_of_confusion = self.circle_of_confusion_arbitrary
        self.angle_of_view = 2*np.arctan2( self.sensor_size , (2.*self.focal_length ) )
        self.hyperfocal_distance = self.focal_length**2. / (self.aperture*self.circle_of_confusion) + self.focal_length
            
    def set_focal_length(self, f):
        self.focal_length = f
        self.calc_dependent_parameters()
    def set_aperture(self, a):
        self.aperture = a
        self.calc_dependent_parameters()
    def set_circle_of_confusion_arbitrarily(self, val):  # set an arbitrary circle of confusion: http://en.wikipedia.org/wiki/Circle_of_confusion
        self.circle_of_confusion_arbitrary = val # mm
        self.calc_dependent_parameters()
    
    def get_min_dist(self, imaged_area_dim):
        min_dist = (imaged_area_dim/2.) / np.tan(self.angle_of_view / 2.)
        return min_dist
        
    def get_imaged_area_at_dist(self, dist):
        imaged_area = np.tan(self.angle_of_view / 2.)*dist*2
        return imaged_area
    
    def get_focus_range_from_near_distance(self, min_dist):
        focus_setting = ( min_dist*(self.hyperfocal_distance - 2*self.focal_length) ) / ( self.hyperfocal_distance - self.focal_length - min_dist )
        max_dist = focus_setting*(self.hyperfocal_distance - self.focal_length) / ( self.hyperfocal_distance - focus_setting )
        return min_dist, max_dist, focus_setting
        
    def get_size_of_imaged_object(self, obj_length, dist, result='pixels'): # return size in pixels or mm (imaged size on the actual sensor in mm that is)
        imaged_area = self.get_imaged_area_at_dist(dist)
        ratio = obj_length / imaged_area
        size_in_mm = ratio*self.sensor_size
        size_in_pixels = size_in_mm / self.pixel_pitch
        
        if result == 'pixels':
            return size_in_pixels
        elif result == 'mm':
            return size_in_mm
            
    def get_min_dist_required_for_focus_range(self, desired_range=1000., result='min_dist', plot=False):
        # use this to calculate the closest distance you can use for your filming volume given a certain volume depth
        min_dist_at_hyperfocal = self.hyperfocal_distance*(self.hyperfocal_distance-self.focal_length) / (self.hyperfocal_distance + self.hyperfocal_distance - 2*self.focal_length)
        min_dist_arr = np.linspace(0,min_dist_at_hyperfocal,1000)
        max_dist_arr = np.zeros_like(min_dist_arr)
        foc_dist_arr = np.zeros_like(min_dist_arr)
            
        for i, min_dist in enumerate(min_dist_arr):
            min_dist, max_dist, focus_setting = self.get_focus_range_from_near_distance(min_dist)
            min_dist_arr[i] = min_dist
            max_dist_arr[i] = max_dist
            foc_dist_arr[i] = focus_setting
        foc_range = max_dist_arr - min_dist_arr
        min_dist_for_desired_range = np.interp(desired_range, foc_range, min_dist_arr)
        
        if plot:
            plt.figure(1)
            plt.plot(min_dist_arr)
            plt.plot(max_dist_arr)
            plt.xlabel('min focus distance')
            plt.ylabel('max focus distance')
            
            plt.figure(2)
            plt.plot(min_dist_arr, foc_range)
            plt.xlabel('min focus distance')
            plt.ylabel('focus range')
            plt.plot(min_dist_for_desired_range*np.ones_like(max_dist_arr), max_dist_arr, 'r')
        
        if result == 'min_dist':
            return min_dist_for_desired_range
        else:
            return min_dist_arr, max_dist_arr, foc_dist_arr, min_dist_for_desired_range
            
    def get_blurred_image_size(self, width, height, dist, result='pixels'):
        imaged_width = self.get_size_of_imaged_object(width, dist, result='mm')
        imaged_height = self.get_size_of_imaged_object(height, dist, result='mm')
        blurred_width = (imaged_width + self.circle_of_confusion)
        blurred_height = (imaged_height + self.circle_of_confusion)
        if result == 'pixels':
            return blurred_width / self.pixel_pitch, blurred_height / self.pixel_pitch
        elif result == 'mm':
            return blurred_width, blurred_height
                
    def print_min_stats(self):
        
        min_dist_for_desired_range = self.get_min_dist_required_for_focus_range(desired_range=1000.)
        min_dist, max_dist, focus_setting = self.get_focus_range_from_near_distance(min_dist_for_desired_range)
        
        imaged_area = self.get_imaged_area_at_dist(min_dist)
        obj_size_min = self.get_size_of_imaged_object(2, max_dist, result='pixels')
        obj_size_max = self.get_size_of_imaged_object(2, min_dist, result='pixels')
        blurred_width_min, blurred_height_min = self.get_blurred_image_size(2,1,max_dist)
        blurred_width_max, blurred_height_max = self.get_blurred_image_size(2,1,min_dist)
        
        print 'close dist: ', min_dist_for_desired_range
        print 'far dist: ', max_dist
        print 'imaged area (length): ', imaged_area        
        print 'size range of fly in pixels: ', obj_size_min, ' : ', obj_size_max
        print 'blurred min ratio: ', blurred_width_min / blurred_height_min         
        print 'blurred max ratio: ', blurred_width_max / blurred_height_max
        
    def plot_stats(self):
        
        desired_range = 500.
        min_dist_for_desired_range = self.get_min_dist_required_for_focus_range(desired_range=desired_range)
        
        
        min_dist_arr = np.linspace(min_dist_for_desired_range, min_dist_for_desired_range+1000, 100)
        imaged_area_arr = np.zeros_like(min_dist_arr)
        obj_size_min_arr = np.zeros_like(min_dist_arr)
        
        for i, min_dist in enumerate(min_dist_arr):
        
            max_dist = min_dist+desired_range
            imaged_area_arr[i] = self.get_imaged_area_at_dist(min_dist)
            obj_size_min_arr[i] = self.get_size_of_imaged_object(2, max_dist, result='pixels')
        
        plt.figure(1)
        plt.plot(min_dist_arr/1000., imaged_area_arr/1000.)
        plt.xlabel('min distance, m')
        plt.ylabel('imaged area, m')
        
        plt.figure(2)
        plt.plot(min_dist_arr/1000., obj_size_min_arr)
        plt.xlabel('min distance, m')
        plt.ylabel('min imaged obj, pixels')
        
    
        
        
def get_basler_ace():
    focal_length = 2
    aperture = 8
    pixel_pitch = .0056
    sensor_height = 2.4
    
    basler = Camera(focal_length, pixel_pitch, sensor_height, aperture, circle_of_confusion_type='pixels', circle_of_confusion_pixel_factor=2.)
    return basler
        
        
        
        
        
        
        
        
