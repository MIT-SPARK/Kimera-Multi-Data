#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rospy
import tf2_ros

import geometry_msgs.msg as geometry_msgs

import kmd_tools


def build_static_tfmsg(T, frame_id, child_frame_id):
    """
    Builds a static geometry_msgs/TransformStamped message.

    Note that because it is static, we just use rospy.Time.now().

    Parameters
    ----------
    T : (4,4) np.array -- SE3 transform of {child_frame_id} w.r.t {frame_id}
    frame_id : str -- parent frame, sometimes called target frame
    child_frame_id : str -- child frame, sometimes called source frame

    Returns
    -------
    tfmsg : geometry_msgs.TransformStamped
    """
    tfmsg = geometry_msgs.TransformStamped()
    tfmsg.header.stamp = rospy.Time.now()
    tfmsg.header.frame_id = frame_id
    tfmsg.child_frame_id = child_frame_id
    tfmsg.transform.translation.x = T[0,3]
    tfmsg.transform.translation.y = T[1,3]
    tfmsg.transform.translation.z = T[2,3]
    q = Rot.from_matrix(T[:3,:3]).as_quat()
    tfmsg.transform.rotation.x = q[0]
    tfmsg.transform.rotation.y = q[1]
    tfmsg.transform.rotation.z = q[2]
    tfmsg.transform.rotation.w = q[3]

    return tfmsg


if __name__ == '__main__':
    rospy.init_node('robot_extrinsics_tf_broadcaster')
    tf = tf2_ros.StaticTransformBroadcaster()

    camera = rospy.get_param('~camera', 'forward')
    robot = rospy.get_param('~robot', 'acl_jackal')
    # robots = ["acl_jackal", "acl_jackal2", "apis", "hathor", "sobek", "sparkal1", "sparkal2", "thoth"]
    lidar_links = {
        'acl_jackal': 'velodyne_link',
        'acl_jackal2': 'velodyne_link',
        'apis': 'ouster_link',
        'hathor': 'base',
        'sobek': 'ouster_link',
        'sparkal1': 'velodyne',
        'sparkal2': 'velodyne',
        'thoth': 'ouster_link',
    }

    tfs = []

    frame_id = f"{robot}/base_link"
    child_frame_id = f"{robot}/{camera}_link"
    T = kmd_tools.extrinsics.get(robot).T_baselink_cameralink
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/{camera}_link"
    child_frame_id = f"{robot}/{camera}_infra1_frame"
    T = kmd_tools.extrinsics.get(robot).T_cameralink_infra1
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))

    frame_id = f"{robot}/{camera}_infra1_frame"
    child_frame_id = f"{robot}/{camera}_infra1_optical_frame"
    T = kmd_tools.extrinsics.get(robot).T_infra1_infra1optical
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/{camera}_link"
    child_frame_id = f"{robot}/{camera}_infra2_frame"
    T = kmd_tools.extrinsics.get(robot).T_cameralink_infra2
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))

    frame_id = f"{robot}/{camera}_infra2_frame"
    child_frame_id = f"{robot}/{camera}_infra2_optical_frame"
    T = kmd_tools.extrinsics.get(robot).T_infra2_infra2optical
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/{camera}_link"
    child_frame_id = f"{robot}/{camera}_color_frame"
    T = kmd_tools.extrinsics.get(robot).T_cameralink_color
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))

    frame_id = f"{robot}/{camera}_color_frame"
    child_frame_id = f"{robot}/{camera}_color_optical_frame"
    T = kmd_tools.extrinsics.get(robot).T_color_coloroptical
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/{camera}_link"
    child_frame_id = f"{robot}/{camera}_depth_frame"
    T = kmd_tools.extrinsics.get(robot).T_cameralink_depth
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))

    frame_id = f"{robot}/{camera}_depth_frame"
    child_frame_id = f"{robot}/{camera}_depth_optical_frame"
    T = kmd_tools.extrinsics.get(robot).T_depth_depthoptical
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/{camera}_link"
    child_frame_id = f"{robot}/{camera}_gyro_frame"
    T = kmd_tools.extrinsics.get(robot).T_cameralink_gyro
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))

    frame_id = f"{robot}/{camera}_gyro_frame"
    child_frame_id = f"{robot}/{camera}_imu_optical_frame"
    T = kmd_tools.extrinsics.get(robot).T_gyro_imuoptical
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))



    frame_id = f"{robot}/base_link"
    child_frame_id = f"{robot}/{lidar_links[robot]}"
    T = kmd_tools.extrinsics.get(robot).T_baselink_lidar
    tfs.append(build_static_tfmsg(T, frame_id, child_frame_id))


    tf.sendTransform(tfs)
    rospy.spin()
