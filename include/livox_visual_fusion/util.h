#pragma once

#include <ros/ros.h>
#include <sys/stat.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <string>

#define PMTD_CV_OPEN_YAML(fs, path, type)                \
  try {                                                  \
    fs.open(path, type);                                 \
    if (!fs.isOpened()) {                                \
      std::cerr << "--- Config: "                        \
                << "can not open " << path << std::endl; \
      fs.release();                                      \
      return false;                                      \
    }                                                    \
  } catch (const cv::Exception &e) {                     \
    std::cerr << "cfg path: " << path << std::endl;      \
    std::cerr << e.what() << std::endl;                  \
  }

template <class T>
void GPARAM(const ros::NodeHandle &n, std::string param_path, T &param) {
  if (!n.getParam(param_path, param))
    ROS_ERROR_STREAM("Load param from " << param_path << " failed...");
}

inline bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

inline bool LoadExtrinsic(Eigen::Matrix4d &e_output,
                          const std::string &cfg_path) {
  cv::FileStorage fs;
  PMTD_CV_OPEN_YAML(fs, cfg_path, cv::FileStorage::READ);

  cv::Mat cv_matrix;
  try {
    fs["extrinsic_matrix"] >> cv_matrix;
    e_output = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        int index = i * 4 + j;
        e_output(i, j) = cv_matrix.at<double>(i, j);
      }
    }
    std::cout << "extrinsic_matrix" << std::endl << cv_matrix << std::endl;
  } catch (const cv::Exception &e) {
    std::cerr << "cfg path: " << cfg_path << std::endl;
    std::cerr << e.what() << std::endl;
    fs.release();
    return false;
  }

  fs.release();
  return true;
}

inline bool LoadExtrinsic(Eigen::Matrix4d &e_output,
                          const std::string &cfg_path,
                          const std::string &node_name) {
  cv::FileStorage fs;
  PMTD_CV_OPEN_YAML(fs, cfg_path, cv::FileStorage::READ);
  try {
    cv::FileNode yaml_node = fs[node_name];
    cv::Mat cv_matrix;
    yaml_node["extrinsic_matrix"] >> cv_matrix;
    e_output = Eigen::Matrix4d::Identity();
    cv::cv2eigen(cv_matrix, e_output);
    std::cout << "extrinsic_matrix" << std::endl << cv_matrix << std::endl;
  } catch (const cv::Exception &e) {
    std::cerr << "cfg path: " << cfg_path << std::endl;
    std::cerr << e.what() << std::endl;
    fs.release();
    return false;
  }

  fs.release();
  return true;
}

inline bool WriteExtrinsic(Eigen::Matrix4d &pose, const std::string &cfg_path,
                           const std::string &node_name) {
  cv::FileStorage fs;
  PMTD_CV_OPEN_YAML(fs, cfg_path, cv::FileStorage::WRITE);
  try {
    cv::Mat extrinsic_matrix = cv::Mat::zeros(4, 4, CV_64FC1);
    cv::eigen2cv(pose, extrinsic_matrix);

    fs << node_name;
    fs << "{";
    fs << "extrinsic_matrix" << extrinsic_matrix;
    fs << "}";

  } catch (const cv::Exception &e) {
    std::cerr << "cfg path: " << cfg_path << std::endl;
    std::cerr << e.what() << std::endl;
    fs.release();
    return false;
  }

  fs.release();
  return true;
}

inline bool WriteExtrinsic(Eigen::Matrix4d &pose, const std::string &cfg_path) {
  cv::FileStorage fs;
  PMTD_CV_OPEN_YAML(fs, cfg_path, cv::FileStorage::WRITE);
  try {
    cv::Mat extrinsic_matrix = cv::Mat::zeros(4, 4, CV_64FC1);
    cv::eigen2cv(pose, extrinsic_matrix);
    fs << "extrinsic_matrix" << extrinsic_matrix;

  } catch (const cv::Exception &e) {
    std::cerr << "cfg path: " << cfg_path << std::endl;
    std::cerr << e.what() << std::endl;
    fs.release();
    return false;
  }

  fs.release();
  return true;
}

inline bool LoadTxtExtrinsic(std::string filepath, Eigen::Matrix4d &pose) {
  try {
    std::ifstream infile(filepath, std::ios_base::binary);
    infile.read((char *)&pose, sizeof(Eigen::Matrix4d));
    infile.close();
    std::cout << "pose\n" << pose << std::endl;
    return true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}
