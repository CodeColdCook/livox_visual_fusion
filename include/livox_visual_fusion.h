#ifndef LIVOX_VISUAL_FUSION_H
#define LIVOX_VISUAL_FUSION_H

#include <fstream>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

struct Config {
  std::string intrinsic_file_path;
  std::string extrinsic_file_path;

  double distance_valid = 5;
};

class LivoxVisualFusion {
 public:
  LivoxVisualFusion(Config config);

  bool Init();

  bool ProcessData(cv::Mat input_image, VPointCloud::Ptr input_cloud,
                   ColorPointCloud::Ptr output_cloud);

  void UndistortImage(cv::Mat image, cv::Mat &rectify_image);
  bool SpaceToPlane(Eigen::Vector3d P_w, Eigen::Vector2d &P_cam,
                    double max_distance);

  bool Txt2extrinsic(std::string filepath);
  bool LoadIntrinsic(std::string filepath);
  bool LoadExtrinsic(std::string filepath);

  Config m_config;

 private:
  bool start_flag;

  Eigen::Matrix3d m_R;
  Eigen::Vector3d m_t;
  Eigen::Isometry3d T_lidar2cam;

  cv::Mat camK, distort_param;
  double m_fx, m_fy, m_cx, m_cy;
  cv::Size m_image_size;
};

#endif
