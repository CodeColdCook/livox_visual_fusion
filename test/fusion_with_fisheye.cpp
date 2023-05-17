#include <pcl/io/pcd_io.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "livox_visual_fusion/pcl_common.h"
#include "livox_visual_fusion/util.h"

camodocal::CataCameraPtr p_cam_;

bool try_load_image(const std::string &file_name, cv::Mat &img) {
  try {
    img = cv::imread(file_name);
  } catch (const cv::Exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  if (img.empty()) {
    std::cout << "cv::imread " << file_name.c_str() << " failed.\n";
    return false;
  }
  return true;
}

bool try_load_pointcloud(const std::string &file_name, IPointCloud &cloud) {
  if (file_name.empty()) {
    std::cerr << "Can't load pointcloud: no file name provided" << std::endl;
    return false;
  } else if (pcl::io::loadPCDFile(file_name, cloud) < 0) {
    std::cerr << "Failed to parse pointcloud from file ('" << file_name
              << std::endl;
    return false;
  }
  return true;
}

void ImageReproject2Cloud(const cv::Mat image_undistort,
                          IPointCloud::Ptr cloud_src,
                          ColorPointCloud::Ptr color_cloud,
                          const Eigen::Matrix4d &T_lidar2cam) {
  std::cout << p_cam_->getParameters() << std::endl;
  if (p_cam_->getParameters().xi() != 0) {
    std::cerr << "camera config is not undistorted: " << std::endl;
  }
  Eigen::Matrix3d R = T_lidar2cam.block<3, 3>(0, 0);
  Eigen::Vector3d t = T_lidar2cam.block<3, 1>(0, 3);

  for (unsigned int index = 0; index < cloud_src->size(); index++) {
    IPoint ptmp = cloud_src->points[index];
    Eigen::Vector3d Pw(ptmp.x, ptmp.y, ptmp.z);
    Eigen::Vector3d Pwcamera = R * Pw + t;
    Eigen::Vector2d Pcam;
    p_cam_->spaceToPlane(Pwcamera, Pcam);
    // std::cout << "pcam: " << Pcam.transpose() << std::endl;
    if (Pcam(0) < 0 || Pcam(0) > p_cam_->getParameters().imageWidth() &&
        Pcam(1) < 0 && Pcam(1) > p_cam_->getParameters().imageHeight()) {
      continue;
    }

    int x = static_cast<int>(Pcam[0]);
    int y = static_cast<int>(Pcam[1]);
    ColorPoint rgb_p;
    rgb_p.b = image_undistort.ptr<uchar>(y)[x * 3];
    rgb_p.g = image_undistort.ptr<uchar>(y)[x * 3 + 1];
    rgb_p.r = image_undistort.ptr<uchar>(y)[x * 3 + 2];

    rgb_p.x = ptmp.x;
    rgb_p.y = ptmp.y;
    rgb_p.z = ptmp.z;
    color_cloud->push_back(rgb_p);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_with_fisheye");
  ros::NodeHandle nh;

  std::string path_camera_intrinsic, path_extrinsic;
  double distance;
  int flag_image_undistorted = -1;
  GPARAM(nh, "fusion_common/path_intrinsic", path_camera_intrinsic);
  GPARAM(nh, "fusion_common/path_extrinsic", path_extrinsic);
  GPARAM(nh, "fusion_common/distance", distance);
  GPARAM(nh, "fusion_common/flag_image_undistorted", flag_image_undistorted);

  std::string path_image, path_pcd, path_pcd_reslut;
  double scale = 1.0;
  GPARAM(nh, "fusion_with_fisheye/path_image", path_image);
  GPARAM(nh, "fusion_with_fisheye/path_pcd", path_pcd);
  GPARAM(nh, "fusion_with_fisheye/scale", scale);
  GPARAM(nh, "fusion_with_fisheye/path_pcd_reslut", path_pcd_reslut);
  cv::Mat image_src;
  if (!try_load_image(path_image, image_src)) return 0;

  IPointCloud icloud_src;
  if (!try_load_pointcloud(path_pcd, icloud_src)) return 0;

  p_cam_ = camodocal::CataCameraPtr(new camodocal::CataCamera);
  camodocal::CataCamera::Parameters params_raw = p_cam_->getParameters();
  params_raw.readFromYamlFile(path_camera_intrinsic);
  p_cam_->setParameters(params_raw);
  std::cout << "params_raw: " << p_cam_->getParameters() << std::endl;
  cv::Mat remap_mat_1, remap_mat_2;
  cv::Mat K_new = p_cam_->initUndistortRectifyMap(
      remap_mat_1, remap_mat_2, params_raw.gamma1() / scale,
      params_raw.gamma2() / scale, cv::Size(0, 0), params_raw.u0(),
      params_raw.v0());

  camodocal::CataCamera::Parameters param_new;
  param_new.gamma1() = K_new.at<float>(0, 0);
  param_new.gamma2() = K_new.at<float>(1, 1);
  param_new.u0() = K_new.at<float>(0, 2);
  param_new.v0() = K_new.at<float>(1, 2);
  param_new.imageWidth() = params_raw.imageWidth();
  param_new.imageHeight() = params_raw.imageHeight();
  p_cam_->setParameters(param_new);
  std::cout << "param_new: " << p_cam_->getParameters() << std::endl;

  Eigen::Matrix4d T_lidar2cam;
  LoadExtrinsic(T_lidar2cam, path_extrinsic);

  cv::imshow("image_src", image_src);
  cv::waitKey(0);
  cv::Mat image_undistorted;
  if (flag_image_undistorted < 0) {
    if (image_src.cols != param_new.imageWidth() ||
        image_src.rows != param_new.imageHeight()) {
      cv::resize(image_src, image_src,
                 cv::Size(param_new.imageWidth(), param_new.imageHeight()));
      cv::remap(image_src, image_undistorted, remap_mat_1, remap_mat_2,
                cv::INTER_LINEAR);
    }

  } else if (image_src.cols != param_new.imageWidth() ||
             image_src.rows != param_new.imageHeight()) {
    std::cerr << "image size is error: " << image_src.size() << std::endl;
    return 0;
  } else {
    image_undistorted = image_src.clone();
  }
  cv::imshow("image_undistorted", image_undistorted);
  cv::waitKey(0);

  ColorPointCloud::Ptr color_cloud_dst(new ColorPointCloud);
  ImageReproject2Cloud(image_undistorted, icloud_src.makeShared(),
                       color_cloud_dst, T_lidar2cam);

  if (color_cloud_dst->size() < 1) {
    std::cout << "cloud is empty: " << std::endl;
  } else {
    pcl::io::savePCDFileASCII(path_pcd_reslut, *color_cloud_dst);
    std::cout << "Success to add pointcloud, save to file" << path_pcd_reslut
              << std::endl;
  }

  return 0;
}