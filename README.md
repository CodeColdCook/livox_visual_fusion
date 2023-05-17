## livox_visual_fusion

#### livox激光点云与图像数据的融合，实时点云着色

### 介绍

​	实时点云着色

![三维信息采集与处理](./${image}/三维信息采集与处理-1635996193395.png)

### 依赖

- ros-melodic
- [Eigen3](https://github.com/artsy/eigen)
- [OpenCV3.2+](https://github.com/opencv/opencv)
- [livox_SDK](https://github.com/Livox-SDK/Livox-SDK)
- [livox_driver](https://github.com/Livox-SDK/livox_ros_driver)
- [camera_models](./Thirdparty/camera_models)

### Build

```shell
mkdir -p cloud_image_fusion_ws/src
cd cloud_image_fusion_ws/src
cp -r ./livox_visual_fusion/Thirdparty/camera_models ./
cd ..
catkin_make
```

### Usage

- [相机标定](https://github.com/ethz-asl/kalibr)
- [鱼眼相机标定](https://gitee.com/sensors_and_external_devices_drive/camera_calib)
- [相机雷达标定](https://gitee.com/csc105/dashboard/projects/csc105_slam_group/lidar_camera_calib/tree/calib_ui)
- [鱼眼相机与激光雷达标定](https://edu.gitee.com/csc105/repos/csc105_slam_group/lidar_camera_calib/tree/fisheye)

### Run

- 针孔相机点云着色

```shell
source devel/setup.zsh
roslaunch livox_visual_fusion test_fusion.launch
```

- 鱼眼相机点云着色

  [测试数据](./test_data/NC_1.1_001)

```shell
# 将launch中载入参数路径该为目标参数配置后
source devel/setup.zsh
roslaunch fusion_with_fisheye_offline.launch
```

- 着色点云查看

  依赖[pcl_ros](http://wiki.ros.org/pcl_ros)

  `sudo apt-get install ros-melodic-pcl-ros`

```shell
source devel/setup.zsh
roslaunch  livox_visual_fusion pcd_to_pointcloud_topics_ingle.launch
```

