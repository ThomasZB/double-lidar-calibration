/*
 * @Author: Ziming.Liu
 * @Date: 2021-06-Fr 03:28:38
 * @Last Modified by:   Hang Chen
 * @Last Modified time: 2022-14-Dec 22:04
 */

#include "multi_lidar_calibration.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_lidar_calibration");
  ros::NodeHandle n;

  MultiLidarCalibration multi_lidar_calibration(n);
  ros::Rate rate(10);
  while (ros::ok()) {
    // update laserscan
    ros::spinOnce();

    multi_lidar_calibration.Run();

    rate.sleep();
  }

  return 0;
}
