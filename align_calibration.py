import numpy as np
import scipy.linalg
import flydra.align as align
import flydra.reconstruct as reconstruct



# the script allows you to align an existing calibration with an object of known size and orientation to create a new (more meaningful) calibration
# use flydra_analysis_plot_kalman_2d to get the necessary 3D points ('x' marks a spot, 'i' find the best fit intersection)


########### user entered values ######################################

orig_cal_file = '/home/floris/data/calibrations/20100609_arena/20100609_scaled.xml'
new_xml_file = '/home/floris/data/calibrations/20100609_arena/20100609_postaligned.xml'

# center: enter the 3d point that you would like to be the center of the new calibration
center = np.array([-0.04108834, -0.0129954 , -0.22327666])

# for scale: enter two points, s1 and s2. the distance between these two points will be scaled to equal the desired size
s1 = np.array([-0.04124752, 0.00477603, -0.2220248])
s2 = np.array([ -0.03948431, -0.03066134, -0.22075723])
desired_size = .01912

# zaxis: enter two points along the desired zaxis
p1 = np.array([-0.03357589, -0.0270953 , -0.17019208])
p2 = np.array([ 0.01346261, -0.03384385, 0.15798934])

#######################################################################

############ calculate rotation, scale, and translation ###############

size = scipy.linalg.norm(s2-s1)
scale = desired_size/size

nz = (p2 - p2) / scipy.linalg.norm((p1 - p2))
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

srcR = reconstruct.Reconstructor(cal_source=orig_cal_file)
alignedR = srcR.get_aligned_copy(M)
alignedR.save_to_xml_filename(new_xml_file)


