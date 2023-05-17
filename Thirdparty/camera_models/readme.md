# Introduction

多相机模型内参标定和去畸变工具
part of [camodocal](https://github.com/hengli/camodocal)

## Dependence

- [OpenCV](https://github.com/opencv/opencv) : 3.4,4.X
- [ceres-solver](<http://ceres-solver.org>) : 2.0.0

## Usage

- 支持相机模型：
  - kannala-brandt
  - mei
  - scaramuzza
  - pinhole
  - pinhole_full

### Calibration

Use [intrinsic_calib.cc](src/intrinsic_calib.cc) to calibrate your camera.

- 注意：
  - 请仔细阅读[intrinsic_calib.cc](src/intrinsic_calib.cc)以清楚各个选项的含义
  - 摒弃了`prefix`和`file-extension`选项进行文件筛选

``` bash

cd camera_calib_example
rosrun camera_models Calibrations -w 11 -h 8 -s 20 -i instadata/right -m mei

```

### Undistortion

See [Camera.h](include/camodocal/camera_models/Camera.h) for general interface:

- liftProjective: Lift points from the image plane to the projective space.
- spaceToPlane: Projects 3D points to the image plane (Pi function)

搭配[fisheye-flattener](https://gitee.com/sensors_and_external_devices_drive/fisheye-flattener)使用
