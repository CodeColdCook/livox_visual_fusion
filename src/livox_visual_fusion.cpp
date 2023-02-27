#include <livox_visual_fusion.h>

LivoxVisualFusion::LivoxVisualFusion(Config config)
    : m_config(config), start_flag(false) {}

bool LivoxVisualFusion::Init() {
  if (start_flag == false) {
    Txt2extrinsic(m_config.extrinsic_file_path);
    std::cout <<"1" << std::endl;
    LoadIntrinsic(m_config.intrinsic_file_path);
  std::cout <<"2" << std::endl;
    start_flag = true;
    return true;
  }
  return false;
}

bool LivoxVisualFusion::ProcessData(cv::Mat input_image,
                                    VPointCloud::Ptr input_cloud,
                                    ColorPointCloud::Ptr output_cloud) {
  if (!start_flag) {
    std::cout << "please load config first ..." << std::endl;
    return false;
  }

  cv::Mat rectify_image = input_image.clone();
  UndistortImage(input_image, rectify_image);
  // cv::imshow("rectify_image",rectify_image);
  // cv::imshow("input_image",input_image);
  // cv::waitKey(1);

  for (size_t index = 0; index < input_cloud->points.size(); index++) {
    VPoint tmp_p = input_cloud->points[index];

    Eigen::Vector3d Pw(static_cast<double>(tmp_p.x),
                       static_cast<double>(tmp_p.y),
                       static_cast<double>(tmp_p.z));
    Eigen::Vector2d Pcam;
    if (SpaceToPlane(Pw, Pcam, m_config.distance_valid)) {
      int x = static_cast<int>(Pcam[0]);
      int y = static_cast<int>(Pcam[1]);

      ColorPoint rgb_p;
      rgb_p.b = rectify_image.ptr<uchar>(y)[x * 3];
      rgb_p.g = rectify_image.ptr<uchar>(y)[x * 3 + 1];
      rgb_p.r = rectify_image.ptr<uchar>(y)[x * 3 + 2];

      rgb_p.x = tmp_p.x;
      rgb_p.y = tmp_p.y;
      rgb_p.z = tmp_p.z;

      output_cloud->push_back(rgb_p);
    }
  }

  return true;
}

void LivoxVisualFusion::UndistortImage(cv::Mat image, cv::Mat& rectify_image) {
//  cv::undistort(image, rectify_image, camK, distort_param, camK);
//  cv::undistort(image, rectify_image, camK, distort_param);
//  rectify_image = image.clone();
  cv::Mat map1, map2;
  cv::Size imageSize;
  imageSize = image.size();
  initUndistortRectifyMap(camK, distort_param, cv::Mat(),
      getOptimalNewCameraMatrix(camK, distort_param, imageSize, 1, imageSize, 0),
  imageSize, CV_16SC2, map1, map2);

  remap(image, rectify_image, map1, map2, cv::INTER_LINEAR);
}

bool LivoxVisualFusion::SpaceToPlane(Eigen::Vector3d P_w,
                                     Eigen::Vector2d& P_cam,
                                     double max_distance) {
  // 将点转到相机坐标系
  Eigen::Vector3d p_c = m_R * P_w + m_t;

  if (p_c(2) < 0 || p_c(2) > max_distance) {
    return false;
  }
  double u = p_c[0] / p_c[2];
  double v = p_c[1] / p_c[2];

  P_cam(0) = m_fx * u + m_cx;
  P_cam(1) = m_fy * v + m_cy;

  if (P_cam(0) > 0 && P_cam(0) < m_image_size.width && P_cam(1) > 0 &&
      P_cam(1) < m_image_size.height)
    return true;

  return false;
}

bool LivoxVisualFusion::Txt2extrinsic(std::string filepath) {
  Eigen::Matrix4d lidar2cam;
  std::ifstream infile(filepath, std::ios_base::binary);
  infile.read((char*)&lidar2cam, sizeof(Eigen::Matrix4d));
  infile.close();
  std::cout << "lidar2cam\n" << lidar2cam << std::endl;

  Eigen::Isometry3d T_lidar2cam_temp = Eigen::Isometry3d::Identity();
  T_lidar2cam_temp.rotate(lidar2cam.block<3, 3>(0, 0));
  T_lidar2cam_temp.pretranslate(lidar2cam.block<3, 1>(0, 3));

  m_R = lidar2cam.block<3, 3>(0, 0);
  m_t = lidar2cam.block<3, 1>(0, 3);
  T_lidar2cam = T_lidar2cam_temp;

  std::cout << "cam2lidar: \n"
            << T_lidar2cam_temp.inverse().matrix() << std::endl;

  return true;
}

bool LivoxVisualFusion::LoadExtrinsic(std::string filepath) {
  cv::FileStorage fs;
  fs.open(filepath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "can not open " << filepath << std::endl;
    return false;
  }
  // cam
  cv::Mat E;
  fs["extrinsic_matrix"] >> E;
  std::cout << "extrinsic_matrix: \n" << E << std::endl;
  fs.release();

  Eigen::Matrix4d lidar2cam;
  lidar2cam(0, 0) = E.at<double>(0, 0);
  lidar2cam(0, 1) = E.at<double>(0, 1);
  lidar2cam(0, 2) = E.at<double>(0, 2);
  lidar2cam(0, 3) = E.at<double>(0, 3);

  lidar2cam(1, 0) = E.at<double>(1, 0);
  lidar2cam(1, 1) = E.at<double>(1, 1);
  lidar2cam(1, 2) = E.at<double>(1, 2);
  lidar2cam(1, 3) = E.at<double>(1, 3);

  lidar2cam(2, 0) = E.at<double>(2, 0);
  lidar2cam(2, 1) = E.at<double>(2, 1);
  lidar2cam(2, 2) = E.at<double>(2, 2);
  lidar2cam(2, 3) = E.at<double>(2, 3);

  lidar2cam(3, 0) = E.at<double>(3, 0);
  lidar2cam(3, 1) = E.at<double>(3, 1);
  lidar2cam(3, 2) = E.at<double>(3, 2);
  lidar2cam(3, 3) = E.at<double>(3, 3);

  std::cout << "lidar2cam\n" << lidar2cam << std::endl;

  Eigen::Isometry3d T_lidar2cam_temp = Eigen::Isometry3d::Identity();
  T_lidar2cam_temp.rotate(lidar2cam.block<3, 3>(0, 0));
  T_lidar2cam_temp.pretranslate(lidar2cam.block<3, 1>(0, 3));

  m_R = lidar2cam.block<3, 3>(0, 0);
  m_t = lidar2cam.block<3, 1>(0, 3);
  T_lidar2cam = T_lidar2cam_temp;

  std::cout << "cam2lidar: \n"
            << T_lidar2cam_temp.inverse().matrix() << std::endl;

  return true;
}

bool LivoxVisualFusion::LoadIntrinsic(std::string filepath) {

  cv::FileStorage fs;
  fs.open(filepath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "can not open " << filepath << std::endl;
    return false;
  }
  // cam
  fs["K"] >> camK;
  fs["d"] >> distort_param;
  m_image_size.width = static_cast<int>(fs["Camera.width"]);
  m_image_size.height = static_cast<int>(fs["Camera.height"]);
  std::cout << "camK: \n" << camK << std::endl;
  std::cout << "dist: \n" << distort_param << std::endl;
  fs.release();

  m_fx = camK.at<double>(0, 0);
  m_cx = camK.at<double>(0, 2);
  m_fy = camK.at<double>(1, 1);
  m_cy = camK.at<double>(1, 2);

  return true;


  /*
  double a[3][3] = {838.656,0.,626.285,
   0.,838.841,363.969,
   0.,0.,1.};
  double d[5][1] = {-0.030168379, -0.1065128379,
  0, -0.000906385, 0};
  camK = cv::Mat(3,3,CV_64F, a);
  distort_param = cv::Mat(5,1,CV_64F, d);

  m_image_size.width = 1280;
  m_image_size.height = 720;

  m_fx = camK.at<double>(0, 0);
  m_cx = camK.at<double>(0, 2);
  m_fy = camK.at<double>(1, 1);
  m_cy = camK.at<double>(1, 2);

  std::cout << "camK: \n" << camK << std::endl;
  std::cout << "dist: \n" << distort_param << std::endl;
  return true;
  */

	
}
