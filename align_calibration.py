import numpy as np
import scipy.linalg
import flydra.align as align
import flydra.reconstruct as reconstruct



# the script allows you to align an existing calibration with an object of known size and orientation to create a new (more meaningful) calibration
# use flydra_analysis_plot_kalman_2d to get the necessary 3D points ('x' marks a spot, 'i' find the best fit intersection)


########### user entered values ######################################

orig_cal_file = '/home/floris/data/calibrations/20100803_arena/20100803_cal_scaled.xml'
new_xml_file = '/home/floris/data/calibrations/20100803_arena/20100803_cal_aligned.xml'

# center: enter the 3d point that you would like to be the center of the new calibration
center = np.array([-0.00160242,  0.0196297 ,  0.15449429])

# for scale: enter two points, s1 and s2. the distance between these two points will be scaled to equal the desired size
s1 = np.array([ 0.05066727, -0.01933785, -0.02735403])
s2 = np.array([ 0.04925564,  0.09637216, -0.03131262])
desired_size = 0.1

# zaxis: enter two points along the desired zaxis
p1 = np.array([0.00204226,  0.00212723,  0.11089473])
p2 = np.array([  0.05369163, -0.00499771, -0.09935711])

#######################################################################

############ calculate rotation, scale, and translation ###############

size = scipy.linalg.norm(s2-s1)
scale = desired_size/size

nz = (p1 - p2) / scipy.linalg.norm((p1 - p2))
z_axis = np.array([0,0,1])

# source: wikipedia, rotation matrices
rot_axis = np.cross(z_axis,nz) / scipy.linalg.norm( np.cross(z_axis,nz) )
rot_angle = -1*np.arccos(np.dot(nz,z_axis))
Rn = np.array([ [ rot_axis[0]**2+(1-rot_axis[0]**2)*np.cos(rot_angle),
                    rot_axis[0]*rot_axis[1]*(1-np.cos(rot_angle))-rot_axis[2]*np.sin(rot_angle),
                    rot_axis[0]*rot_axis[2]*(1-np.cos(rot_angle))+rot_axis[1]*np.sin(rot_angle)],
                   [rot_axis[0]*rot_axis[1]*(1-np.cos(rot_angle))+rot_axis[2]*np.sin(rot_angle),
                    rot_axis[1]**2+(1-rot_axis[1]**2)*np.cos(rot_angle),
                    rot_axis[1]*rot_axis[2]*(1-np.cos(rot_angle))-rot_axis[0]*np.sin(rot_angle)],
                   [rot_axis[0]*rot_axis[2]*(1-np.cos(rot_angle))-rot_axis[1]*np.sin(rot_angle),
                    rot_axis[1]*rot_axis[2]*(1-np.cos(rot_angle))+rot_axis[0]*np.sin(rot_angle),
                    rot_axis[2]**2+(1-rot_axis[2]**2)*np.cos(rot_angle)] ])

center_scaled_rotated = np.dot(Rn, center) * scale
M = align.build_xform(scale,Rn,-1*center_scaled_rotated)

print nz

srcR = reconstruct.Reconstructor(cal_source=orig_cal_file)
alignedR = srcR.get_aligned_copy(M)
alignedR.save_to_xml_filename(new_xml_file)


