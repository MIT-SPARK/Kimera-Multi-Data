import numpy as np
from scipy.spatial.transform import Rotation as Rot

T_baselink_cameralink = np.eye(4)

##
## From D455 factory calibration
##

# Extrinsic from "Depth"      To      "Infrared 1" :
T_infra1optical_depthoptical = np.eye(4)

# Extrinsic from "Infrared 2"     To      "Infrared 1" :
T_infra1optical_infra2optical = np.eye(4)
T_infra1optical_infra2optical[:3,3] = (0.0948528572916985, 0, 0)

# Extrinsic from "Gyro"   To      "Infrared 1" :
T_infra1optical_imuoptical = np.eye(4)
T_infra1optical_imuoptical[:3,3] = (0.0302200000733137, -0.00740000000223517, -0.0160199999809265)

# Extrinsic from "Color"      To      "Infrared 1" :
T_infra1optical_coloroptical = np.eye(4)
T_infra1optical_coloroptical[:3,:3] = np.array([[ 0.999997, 0.00163706, -0.00158481],
                                                [-0.00163422, 0.999997, 0.00179475],
                                                [ 0.00158774, -0.00179215, 0.999997]])
T_infra1optical_coloroptical[:3,3] = (0.0589082464575768, -3.4272645279998e-05, -0.000188634483492933)

##
## Optical frames
##

# {optical frame} w.r.t {front-left-up frame}
T_flu_optical = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

T_depth_depthoptical = T_flu_optical
T_infra1_infra1optical = T_flu_optical
T_infra2_infra2optical = T_flu_optical
T_color_coloroptical = T_flu_optical
T_gyro_imuoptical = T_flu_optical


##
## Recipes for TF tree
##

T_cameralink_infra1 = np.eye(4)
T_cameralink_infra2 = T_cameralink_infra1 @ T_infra1_infra1optical @ T_infra1optical_infra2optical @ np.linalg.inv(T_infra2_infra2optical)
T_cameralink_depth = T_cameralink_infra1 @ T_infra1_infra1optical @ T_infra1optical_depthoptical @ np.linalg.inv(T_depth_depthoptical)
T_cameralink_color = T_cameralink_infra1 @ T_infra1_infra1optical @ T_infra1optical_coloroptical @ np.linalg.inv(T_color_coloroptical)
T_cameralink_gyro = T_cameralink_infra1 @ T_infra1_infra1optical @ T_infra1optical_imuoptical @ np.linalg.inv(T_gyro_imuoptical)


##
## lidar - camera calibration
##

T_coloroptical_lidar = np.eye(4)
T_coloroptical_lidar[:3,:3] = Rot.from_quat((-0.522868, 0.525613, -0.473277, -0.475762)).as_matrix()
T_coloroptical_lidar[:3,3] = (0.00923078, -0.147035, -0.0397743)

# recipe for TF tree
T_baselink_lidar = T_baselink_cameralink @ T_cameralink_color @ T_color_coloroptical @ T_coloroptical_lidar
