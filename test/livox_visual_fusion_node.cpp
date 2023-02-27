#include <ros/package.h>
#include <ros/ros.h>

#include <livox_visual_fusion.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <util.h>

std::deque<cv::Mat> img_buffer;
std::deque<VPointCloud> cloud_buffer;

std::string image_topic, lidar_topic, fusion_pub_topic;
Config config;

void ImageHandler(const sensor_msgs::Image::ConstPtr& img_msg) {
  cv::Mat img = cv_bridge::toCvCopy(img_msg)->image;
  if (!img.empty()) {
    img_buffer.push_back(img);
    if (img_buffer.size() > 3) {
      img_buffer.pop_front();
    }
  }
}

void LiDARHandler(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
  VPointCloud cloud;
  pcl::fromROSMsg(*lidar_msg, cloud);

  cloud_buffer.push_back(cloud);
  if (cloud_buffer.size() > 3) {
    cloud_buffer.pop_front();
  }
}

bool LoadConfig(ros::NodeHandle nh_) {
  GPARAM(nh_, "/basic/intrinsic_file", config.intrinsic_file_path);
  GPARAM(nh_, "/basic/extrinsic_file", config.extrinsic_file_path);
  GPARAM(nh_, "/basic/distance_valid", config.distance_valid);

  GPARAM(nh_, "/basic/image_topic", image_topic);
  GPARAM(nh_, "/basic/lidar_topic", lidar_topic);
  GPARAM(nh_, "/basic/fusion_pub_topic", fusion_pub_topic);

  std::string package_path = ros::package::getPath("livox_visual_fusion");
  config.intrinsic_file_path = package_path + "/" + config.intrinsic_file_path;
  config.extrinsic_file_path = package_path + "/" + config.extrinsic_file_path;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_visual_fusion_node");
  ros::NodeHandle n;

  LoadConfig(n);

  LivoxVisualFusion lv_fusion(config);
  lv_fusion.Init();

  ros::Subscriber sub_livox =
      n.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, LiDARHandler);
  ros::Subscriber sub_image =
      n.subscribe<sensor_msgs::Image>(image_topic, 20, ImageHandler);

  ros::Publisher pub_fusion_cloud =
      n.advertise<sensor_msgs::PointCloud2>(fusion_pub_topic, 10);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    if (img_buffer.size() > 0 && cloud_buffer.size() > 0) {
      cv::Mat tmp_image = img_buffer.back();
      VPointCloud tmp_cloud = cloud_buffer.back();
      img_buffer.clear();

      ColorPointCloud::Ptr fusion_cloud_ptr(new ColorPointCloud);

      lv_fusion.ProcessData(tmp_image, tmp_cloud.makeShared(),
                            fusion_cloud_ptr);

      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*fusion_cloud_ptr, cloud_msg);
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = "livox_frame";

      pub_fusion_cloud.publish(cloud_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
