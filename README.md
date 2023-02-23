# Kimera-Multi-Data: A large-Scale Multi-Robot Dataset for Multi-Robot SLAM 

## Description:

<div align="center">

|  Sequence        |  # Robots  |  Traversal (m)    |  Duration (min)  | 
| ---------------- | ---------- | ----------------- | ---------------- |
|  Campus-Hanger   |  7         |                   |                  |
|  Campus-Outdoor  |  6         |  6044             |  19              | 
|  Campus-Tunnels  |  8         |  6753             |  28              | 
|  Campus-Hybrid   |  8         |  7785             |  27              |

</div>

## Platforms

<p align="center"> <img src="figures/jackal_figure.jpg" title="" alt="" data-align="center"> </p>

### Data format

<div align="center">

| Topic                                         | Type                        | Description                        |
| --------------------------------------------- | --------------------------- | ---------------------------------- |
| /xxx/forward/color/image_raw/compressed       | sensor_msgs/CompressedImage | RGB Image from D455                |
| /xxx/forward/color/camera_info                | sensor_msgs/CameraInfo      | RGB Image Camera Info              |
| /xxx/forward/depth/image_rect_raw             | sensor_msgs/Image           | Depth Image from D455              |
| /xxx/forward/depth/camera_info                | sensor_msgs/CameraInfo      | Depth Image Camera Info            |
| /xxx/forward/infra1/image_rect_raw/compressed | sensor_msgs/CompressedImage | Compressed Gray Scale Stereo Left  |
| /xxx/forward/infra1/camera_info               | sensor_msgs/CameraInfo      | Stereo Left Camera Info            |
| /xxx/forward/infra2/image_rect_raw/compressed | sensor_msgs/CompressedImage | Compressed Gray Scale Stereo Right |
| /xxx/forward/infra2/camera_info               | sensor_msgs/CameraInfo      | Stereo Right Camera Info           |
| /xxx/forward/imu                              | sensor_msgs/Imu             | IMU from D455                      |
| /xxx/jackal_velocity_controller/odom          | nav_msgs/Odometry           | Wheel Odometry                     |
| /xxx/lidar_points                             | sensor_msgs/PointCloud2     | Lidar Point Cloud                  |

</div>

## Ground Truth

<p align="center"> <img src="figures/gt_map_mit.jpg" title="" alt="" data-align="center"> </p>

</div>

## Calibration

We provide the calibration between the sensors.


## Citation
