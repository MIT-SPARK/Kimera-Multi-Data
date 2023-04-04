# Kimera-Multi-Data: A large-Scale Multi-Robot Dataset for Multi-Robot SLAM 
> **Note**
> Due to privacy issues, the datasets are currently being processed to remove any faces or private information.
The datasets are scheduled to be ready by April 5th, 2023.

## Description:

<div align="center">

|  Sequence        |  # Robots  |  Traversal (m)    |  Duration (min)  | 
| ---------------- | ---------- | ----------------- | ---------------- |
|  Campus-Outdoor  |  6         |  6044             |  19              | 
|  Campus-Tunnels  |  8         |  6753             |  28              | 
|  Campus-Hybrid   |  8         |  7785             |  27              |

</div>

## Platforms

<p align="center"> <img src="figures/jackal_figure.jpg" title="" alt="" data-align="center"> </p>

We use a single set of camera intrinsic and extrinsic parameters for all the robots.
The parameters follow the [Kimera-VIO](https://github.com/MIT-SPARK/Kimera) format and can be downloaded below.

### Data format

The datasets are in compressed [rosbag](http://wiki.ros.org/rosbag) format.
For best results, [decompress](http://wiki.ros.org/rosbag/Commandline#decompress) the rosbags before usage.
```bash
rosbag decompress *.bag
```

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

The ground truth trajectory is generated using GPS and total-station assisted LiDAR SLAM based on [LOCUS](https://github.com/NeBula-Autonomy/LOCUS) and [LAMP](https://github.com/NeBula-Autonomy/LAMP).
The process is described in further detail in our paper.
You can download the ground truth trajectory and reference point cloud below.

## Citation
If you found the dataset to be useful, we would appreciate it if you can cite the following paper:

> **Note**
> Arxiv upload coming soon.

## Download

| Name | Rosbags | GT | Photos | Trajectory | 
|:-:|:-:|:-:|:-:|:-:|
| Campus-Outdoor | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1LKUC7wLhlVuoxYRhSCZYUVAAffA9EpDy?usp=share_link) | <img src="figures/photos_outdoor.jpg" alt="drawing" width="400"/> | <img src="figures/1014_gt.png" alt="drawing" width="400"/> |
| Campus-Tunnels | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1iDibVlkZLyK856O7X8lEUInWK-Z0TXG3?usp=share_link) | <img src="figures/photos_tunnels.jpg" alt="drawing" width="400"/> | <img src="figures/1207_gt.png" alt="drawing" width="400"/> |
| Campus-Hybrid  | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1YQnJn8z_yGku-wkw8X_cYd8v5PABSbS7?usp=share_link) | <img src="figures/photos_hybrid.jpg" alt="drawing" width="400"/>  | <img src="figures/1208_gt.png" alt="drawing" width="400"/> |

The camera calibration parameters used for our experiments can be found [here](https://drive.google.com/drive/folders/1YlVl2hoqWNwi6GGX6n_MqeNG-aWmeh9r?usp=share_link).

The point cloud of the reference ground truth map can be downloaded [here](https://drive.google.com/file/d/1u5BC8rEQlA0BKoobgmP5GCCi2vJf58oz/view?usp=share_link).
