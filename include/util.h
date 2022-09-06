#ifndef UTILS_H
#define UTILS_H

#include <sys/stat.h>
#include <fstream>

#include <ros/ros.h>
#include <string>

template <class T>
void GPARAM(const ros::NodeHandle &n, std::string param_path, T &param) {
  if (!n.getParam(param_path, param))
    ROS_ERROR_STREAM("Load param from " << param_path << " failed...");
}

inline bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

#endif
