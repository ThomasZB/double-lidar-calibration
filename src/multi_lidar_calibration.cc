/*
 * @Author: Ziming.Liu
 * @Date: 2021-06-Fr 03:28:56
 * @Last Modified by:   Hang Chen
 * @Last Modified time: 2022-14-Dec 22:04
 */

#include "multi_lidar_calibration.h"

#include <geometry_msgs/TransformStamped.h>

#include <chrono>

MultiLidarCalibration::MultiLidarCalibration(ros::NodeHandle &n)
    : pointcloud_frame_str_("calibration_pointcloud"),
      nh_(n),
      time_constant_(10),
      last_linear_acceleration_time_(ros::Time(0)),
      laset_eulerAngle_(Eigen::Vector3f::Identity()),
      last_transform_(Eigen::Vector3f::Identity()) {
  ROS_INFO_STREAM("\033[1;32m----> Multi Lidar Calibration Use ICP...\033[0m");

  nh_.param<std::string>("/multi_lidar_calibration_node/source_lidar_topic",
                         source_lidar_topic_str_, "/sick_back/scan");
  nh_.param<std::string>("/multi_lidar_calibration_node/target_lidar_topic",
                         target_lidar_topic_str_, "/sick_front/scan");
  nh_.param<std::string>("/multi_lidar_calibration_node/source_lidar_frame",
                         source_lidar_frame_str_, "sub_laser_link");
  nh_.param<std::string>("/multi_lidar_calibration_node/target_lidar_frame",
                         target_lidar_frame_str_, "main_laser_link");
  nh_.param<float>("/multi_lidar_calibration_node/icp_score", icp_score_,
                   5.5487);

  // 发布转换后的激光点云
  final_point_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/final_point_cloud", 10);

  // 订阅多个激光话题
  scan_front_subscriber_ =
      new message_filters::Subscriber<sensor_msgs::LaserScan>(
          nh_, source_lidar_topic_str_, 1);
  scan_back_subscriber_ =
      new message_filters::Subscriber<sensor_msgs::LaserScan>(
          nh_, target_lidar_topic_str_, 1);
  scan_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *scan_front_subscriber_, *scan_back_subscriber_);
  scan_synchronizer_->registerCallback(
      boost::bind(&MultiLidarCalibration::ScanCallBack, this, _1, _2));

  // 参数赋值
  is_first_run_ = true;

  // 在main_laser_link下sub_laser_link的坐标
  transform_martix_ = Eigen::Matrix4f::Identity();  // 4 * 4 齐次坐标
  // 在base_link坐标系下main_laser_link的坐标
  front_to_base_link_ = Eigen::Matrix4f::Identity();

  // 点云指针赋值
  main_scan_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
      new pcl::PointCloud<pcl::PointXYZ>());
  sub_scan_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
      new pcl::PointCloud<pcl::PointXYZ>());
  final_registration_scan_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
      new pcl::PointCloud<pcl::PointXYZ>());
  // 使用在main_laser_link下sub_laser_link的坐标，把sub_laser_link下的激光转换到main_laser_link下
  main_scan_pointcloud_init_transformed_ =
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
          new pcl::PointCloud<pcl::PointXYZ>());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
}

MultiLidarCalibration::~MultiLidarCalibration() {}

/**
 * @brief 获取激光雷达间的坐标变换
 *
 * @param transform_martix_ 激光雷达间的转换矩阵
 * @param front_to_base_link_ 在main_laser_link下sub_laser_link的坐标
 */
void MultiLidarCalibration::GetFrontLasertoBackLaserTf() {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl(buffer);

  ros::Time time = ros::Time::now();
  ros::Duration timeout(0.1);

  geometry_msgs::TransformStamped tfGeom;
  ros::Duration(3).sleep();
  /* 得到source到target雷达的坐标变换 */
  try {
    tfGeom =
        buffer.lookupTransform(source_lidar_frame_str_, target_lidar_frame_str_,
                               ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException &e) {
    ROS_ERROR_STREAM("Lidar Transform Error ... s:" << source_lidar_frame_str_
                                                    << " t:"
                                                    << target_lidar_frame_str_);
  }
  {
    // tf2矩阵转换成Eigen::Matrix4f
    Eigen::Quaternionf qw(tfGeom.transform.rotation.w,
                          tfGeom.transform.rotation.x,
                          tfGeom.transform.rotation.y,
                          tfGeom.transform.rotation.z);  // tf 获得的四元数
    Eigen::Vector3f qt(tfGeom.transform.translation.x,
                       tfGeom.transform.translation.y,
                       tfGeom.transform.translation.z);  // tf获得的平移向量
    transform_martix_.block<3, 3>(0, 0) = qw.toRotationMatrix();
    transform_martix_.block<3, 1>(0, 3) = qt;
  }

  /* 得到source到base_link的坐标变换 */
  try {
    tfGeom = buffer.lookupTransform("base_link", source_lidar_frame_str_,
                                    ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException &e) {
    ROS_ERROR_STREAM("Lidar Transform Error ...");
  }
  {
    // tf2矩阵转换成Eigen::Matrix4f
    Eigen::Quaternionf qw(tfGeom.transform.rotation.w,
                          tfGeom.transform.rotation.x,
                          tfGeom.transform.rotation.y,
                          tfGeom.transform.rotation.z);  // tf 获得的四元数
    Eigen::Vector3f qt(tfGeom.transform.translation.x,
                       tfGeom.transform.translation.y,
                       tfGeom.transform.translation.z);  // tf获得的平移向量
    front_to_base_link_.block<3, 3>(0, 0) = qw.toRotationMatrix();
    front_to_base_link_.block<3, 1>(0, 3) = qt;
  }

  ROS_INFO_STREAM("main_laser_link in base_link matrix=\n"
                  << front_to_base_link_);
}

/**
 * @brief 激光雷达发布点云
 * @param in_cloud_to_publish_ptr 输入icp转换后的激光点云数据
 */
void MultiLidarCalibration::PublishCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = pointcloud_frame_str_;
  final_point_cloud_pub_.publish(cloud_msg);
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "base_link";
  t.child_frame_id = pointcloud_frame_str_;

  /* 平移 */
  t.transform.translation.x = last_transform_[0];
  t.transform.translation.y = last_transform_[1];
  t.transform.translation.z = last_transform_[2];
  /* 旋转 */
  tf2::Quaternion q;
  q.setRPY(0, 0, laset_eulerAngle_[2]);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}

/**
 * @brief 激光雷达消息类型转换 sensor_msg::Laser to
 * pcl::PointCloud<pcl::PointXYZ>
 *
 * @param scan_msg 输入sensor_msgs
 * @return pcl::PointCloud<pcl::PointXYZ> 输出pcl格式点云
 */
pcl::PointCloud<pcl::PointXYZ> MultiLidarCalibration::ConvertScantoPointCloud(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud_points;
  pcl::PointXYZ points;

  for (int i = 0; i < scan_msg->ranges.size(); ++i) {
    float range = scan_msg->ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }

    if (range > scan_msg->range_min && range < scan_msg->range_max) {
      float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      points.x = range * cos(angle);
      points.y = range * sin(angle);
      points.z = 0.0;
      cloud_points.push_back(points);
    }
  }
  return cloud_points;
}

void MultiLidarCalibration::LaserScanToLDP(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp) {
  unsigned int n = scan_msg->ranges.size();
  // 调用csm里的函数进行申请空间
  if (ldp != nullptr) {
    ROS_INFO("maybe memory bug");
  }
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++) {
    // calculate position in laser frame
    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max) {
      // 填充雷达数据
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    } else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->estimate[0] = 0.0;
  ldp->estimate[1] = 0.0;
  ldp->estimate[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void MultiLidarCalibration::PclToLDP(
    const pcl::PointCloud<pcl::PointXYZ> &pcl_src, LDP &ldp) {
  unsigned int n = pcl_src.size();
  // 调用csm里的函数进行申请空间
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++) {
    // 填充雷达数据
    ldp->valid[i] = 1;
    ldp->readings[i] = GetRange(pcl_src[i]);

    ldp->theta[i] = GetTheta(pcl_src[i]);
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->estimate[0] = 0.0;
  ldp->estimate[1] = 0.0;
  ldp->estimate[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

/**
 * @brief 多个激光雷达数据同步
 *
 * @param in_main_scan_msg 激光雷达topic 1
 * @param in_sub_scan_msg 激光雷达topic 2
 */
void MultiLidarCalibration::ScanCallBack(
    const sensor_msgs::LaserScan::ConstPtr &in_main_scan_msg,
    const sensor_msgs::LaserScan::ConstPtr &in_sub_scan_msg) {
  main_scan_pointcloud_ =
      ConvertScantoPointCloud(in_main_scan_msg).makeShared();
  if (sub_scan_ldp_ != nullptr) {
    ld_free(sub_scan_ldp_);
    sub_scan_ldp_ = nullptr;
  }
  LaserScanToLDP(in_sub_scan_msg, sub_scan_ldp_);
}

/**
 * @brief 两个激光雷达数据进行pl_icp匹配
 *
 */
bool MultiLidarCalibration::ScanRegistration() {
  if (0 == main_scan_pointcloud_->points.size() || nullptr == sub_scan_ldp_) {
    return false;
  }

  // 对点云进行初始化旋转，back_link to front_link
  pcl::transformPointCloud(*main_scan_pointcloud_,
                           *main_scan_pointcloud_init_transformed_,
                           transform_martix_);

  // 最大欧式距离差值
  icp_.setMaxCorrespondenceDistance(0.2);
  // 迭代阈值，当前变换矩阵和当前迭代矩阵差异小于阈值，认为收敛
  icp_.setTransformationEpsilon(1e-10);
  // 均方误差和小于阈值停止迭代
  icp_.setEuclideanFitnessEpsilon(0.01);
  // 最多迭代次数
  icp_.setMaximumIterations(200);

  icp_.setInputSource(sub_scan_pointcloud_);
  icp_.setInputTarget(main_scan_pointcloud_init_transformed_);

  icp_.align(*final_registration_scan_);

  if (icp_.hasConverged() == false && icp_.getFitnessScore() > 1.0) {
    ROS_WARN_STREAM("Not Converged ... ");
    return false;
  }
  return true;
}

/**
 * @brief 打印结果
 *
 */
void MultiLidarCalibration::GetResult() {
  if (icp_.getFitnessScore() > icp_score_) {
    return;
  }

  // sub激光雷达到main雷达的icp的计算结果（这个结果是差值）
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T = icp_.getFinalTransformation();
  Eigen::Matrix3f R3 = T.block<3, 3>(0, 0);
  Eigen::Vector3f t3 = T.block<3, 1>(0, 3);
  /* 将这个结果叠加到对应的tf上去（这样两个雷达的相对变换才是正确的） */
  // Eigen::Matrix4f new_T = T * transform_martix_;

  /* 知道main激光和base_link的坐标变换，知道main激光和sub激光的坐标变换，可以求出sub和baselink
   */
  // Eigen::Matrix4f O_B_T = front_to_base_link_ * new_T;
  Eigen::Matrix4f O_B_T = front_to_base_link_ * transform_martix_ * T;
  Eigen::Vector3f transform = O_B_T.block<3, 1>(0, 3);
  Eigen::Vector3f eulerAngle = O_B_T.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

  /* 这里加一个指数滑动平均（其实角度这里由于周期模糊不能加指数平均的） */
  ros::Duration duration = ros::Time::now() - last_linear_acceleration_time_;
  last_linear_acceleration_time_ = ros::Time::now();
  const double delta_t = last_linear_acceleration_time_ > ros::Time(0)
                             ? duration.toSec()
                             : std::numeric_limits<double>::infinity();
  const double alpha = 1. - std::exp(-delta_t / time_constant_);
  laset_eulerAngle_ = (1. - alpha) * laset_eulerAngle_ + alpha * eulerAngle;
  last_transform_ = (1. - alpha) * last_transform_ + alpha * transform;

  // 输出转换关系
  ROS_INFO_STREAM("eulerAngle=\n" << laset_eulerAngle_);
  ROS_INFO_STREAM("transform vector=\n" << last_transform_);
}

/**
 * @brief  点云可视化
 *
 */
void MultiLidarCalibration::View() {
  // 点云可视化
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);  // 背景色设置

  // 显示源点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(
      main_scan_pointcloud_, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(main_scan_pointcloud_, source_color,
                                       "source");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

  // 显示目标点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(
      sub_scan_pointcloud_, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ>(sub_scan_pointcloud_, target_color,
                                       "target");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

  // 显示变换后的源点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      source_trans_color(final_registration_scan_, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ>(final_registration_scan_,
                                       source_trans_color, "source trans");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source trans");

  // 保存变换结果
  pcl::io::savePLYFile("final_registration_scan.pcd", *final_registration_scan_,
                       false);
  viewer->spin();
}

/**
 * @brief 运行主函数
 *
 */
void MultiLidarCalibration::Run() {
  if (is_first_run_) {
    GetFrontLasertoBackLaserTf();
    is_first_run_ = false;
    return;
  }

  // 进行icp匹配，匹配失败返回
  if (!ScanRegistration()) {
    return;
  }

  GetResult();

  PublishCloud(sub_scan_pointcloud_);
}
