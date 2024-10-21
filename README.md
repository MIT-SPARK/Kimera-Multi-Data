# Kimera-Multi-Data: A large-Scale Multi-Robot Dataset for Multi-Robot SLAM 

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

- Y. Tian, Y. Chang, L. Quang, A. Schang, C. Nieto-Granda, J. P. How, and L. Carlone, "Resilient and Distributed Multi-Robot Visual SLAM: Datasets, Experiments, and Lessons Learned," arXiv preprint arXiv:2304.04362, 2023.
```bibtex
@ARTICLE{tian23arxiv_kimeramultiexperiments,
  author={Yulun Tian and Yun Chang and Long Quang and Arthur Schang and Carlos Nieto-Granda and Jonathan P. How and Luca Carlone},
  title={Resilient and Distributed Multi-Robot Visual SLAM: Datasets, Experiments, and Lessons Learned}, 
  year={2023},
  eprint={2304.04362},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```

## Download

| Name | Rosbags | GT | Photos | Trajectory | 
|:-:|:-:|:-:|:-:|:-:|
| Campus-Outdoor | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1LKUC7wLhlVuoxYRhSCZYUVAAffA9EpDy?usp=share_link) | <img src="figures/photos_outdoor.jpg" alt="drawing" width="400"/> | <img src="figures/1014_gt.png" alt="drawing" width="400"/> |
| Campus-Tunnels | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1iDibVlkZLyK856O7X8lEUInWK-Z0TXG3?usp=share_link) | <img src="figures/photos_tunnels.jpg" alt="drawing" width="400"/> | <img src="figures/1207_gt.png" alt="drawing" width="400"/> |
| Campus-Hybrid  | [request](https://forms.gle/EBHJE3LEKkTsnABu7)  | [link](https://drive.google.com/drive/folders/1YQnJn8z_yGku-wkw8X_cYd8v5PABSbS7?usp=share_link) | <img src="figures/photos_hybrid.jpg" alt="drawing" width="400"/>  | <img src="figures/1208_gt.png" alt="drawing" width="400"/> |

The camera calibration parameters used for our experiments can be found [here](https://drive.google.com/drive/folders/1YlVl2hoqWNwi6GGX6n_MqeNG-aWmeh9r?usp=share_link).

The point cloud of the reference ground truth map can be downloaded [here](https://drive.google.com/file/d/1u5BC8rEQlA0BKoobgmP5GCCi2vJf58oz/view?usp=share_link).

<details>
  <summary><strong>Descriptions of LiDAR Sensor Configuration</strong></summary>

  For `10_14` sequences (i.e., `campus_outdoor_1014_compressed` in the shared drive), LiDAR point clouds are acquired by Velodyne VLP-16.

  For `12_07` and `12_08` sequences (i.e., `campus_tunnels_1207_compressed` and `campus_hybrid_1208_compressed`, respectively), some of the robots have different LiDAR setups.
  `apis`, `sobek`, and `thoth` sequences are acquired by [OS1-64 Gen1 LiDAR sensors](https://data.ouster.io/downloads/datasheets/datasheet-gen1-v2p0-os1.pdf), which have a different hardware configuration from the [recent OS1-64 sensors](https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p0-os1.pdf), while other robots have Velodyne VLP-16 sensors.

  The extrinsics can be found [here](https://github.com/plusk01/Kimera-Multi-Data/tree/parker/kmd_tools) (but we appreciate your kind understanding that these extrinsics are not *perfect*. We are always open to contributions to our Kimera-Multi dataset!).
</details>
