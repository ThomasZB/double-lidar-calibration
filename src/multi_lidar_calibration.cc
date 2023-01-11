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
  sub_to_base_link_ = Eigen::Matrix4f::Identity();

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
  InitParams();
}

void MultiLidarCalibration::InitParams() {
  // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_.getParam("max_angular_correction_deg",
                    input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 180;

  // Maximum translation between scans (m)
  if (!nh_.getParam("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 1.0;

  // Maximum ICP cycle iterations
  if (!nh_.getParam("max_iterations", input_.max_iterations))
    input_.max_iterations = 100;

  // A threshold for stopping (m)
  if (!nh_.getParam("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_.getParam("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 1.0;

  // Noise in the scan (m)
  if (!nh_.getParam("sigma", input_.sigma)) input_.sigma = 0.10;

  // Use smart tricks for finding correspondences.
  if (!nh_.getParam("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_.getParam("restart", input_.restart)) input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_.getParam("restart_threshold_mean_error",
                    input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_.getParam("restart_dt", input_.restart_dt)) input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_.getParam("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_.getParam("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_.getParam("orientation_neighbourhood",
                    input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_.getParam("use_point_to_line_distance",
                    input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_.getParam("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_.getParam("do_alpha_test_thresholdDeg",
                    input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!nh_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!nh_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar
  // angle of the points of one scan in the new position. If the polar angle is
  // not a monotone function of the readings index, it means that the surface is
  // not visible in the next position. If it is not visible, then we don't use
  // it for matching.
  if (!nh_.getParam("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method
  // http://purl.org/censi/2006/icpcov
  if (!nh_.getParam("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to
  // compute the incidence beta, and the factor (1/cos^2(beta)) used to weight
  // the correspondence.");
  if (!nh_.getParam("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!nh_.getParam("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

MultiLidarCalibration::~MultiLidarCalibration() {}

/**
 * @brief 获取激光雷达间的坐标变换
 *
 * @param transform_martix_ 激光雷达间的转换矩阵
 * @param sub_to_base_link_ 在base_link下sub_laser_link的坐标
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
    tfGeom = buffer.lookupTransform("base_link", target_lidar_frame_str_,
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
    sub_to_base_link_.block<3, 3>(0, 0) = qw.toRotationMatrix();
    sub_to_base_link_.block<3, 1>(0, 3) = qt;
  }

  ROS_INFO_STREAM("sub laser in base_link matrix=\n" << sub_to_base_link_);
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
  std::map<float, float> laser_scan;
  // 调用csm里的函数进行申请空间
  ldp = ld_alloc_new(n);

  /* 对激光按theta排序 */
  for (unsigned int i = 0; i < n; i++) {
    laser_scan.emplace(GetTheta(pcl_src[i]), GetRange(pcl_src[i]));
  }
  int i = 0;
  for (const auto &each_ray : laser_scan) {
    // 填充雷达数据
    ldp->valid[i] = 1;
    ldp->readings[i] = each_ray.second;

    ldp->theta[i] = each_ray.first;
    ldp->cluster[i] = -1;
    i++;
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
  sub_scan_pointcloud_ = ConvertScantoPointCloud(in_sub_scan_msg).makeShared();
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

  LDP main_scan_ldp;
  PclToLDP(*main_scan_pointcloud_init_transformed_, main_scan_ldp);

  input_.laser_ref = main_scan_ldp;
  input_.laser_sens = sub_scan_ldp_;

  // 位姿的预测值为0，就是不进行预测
  input_.first_guess[0] = 0;
  input_.first_guess[1] = 0;
  input_.first_guess[2] = 0;

  // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
  ROS_INFO("begin pl-icp");
  sm_icp(&input_, &output_);

  if (output_.valid) {
    std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", "
              << output_.x[2] * 180 / M_PI << ")" << std::endl;
  } else {
    ROS_INFO("error~~~");
  }
  ld_free(main_scan_ldp);

  return true;
}

/**
 * @brief 打印结果
 *
 */
void MultiLidarCalibration::GetResult() {
  // sub激光雷达到main雷达的icp的计算结果（这个结果是差值）
  Eigen::AngleAxisf rotation_vecotr(output_.x[2], Eigen::Vector3f(0, 0, 1));
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = rotation_vecotr.toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3f(output_.x[0], output_.x[1], 0);

  Eigen::Matrix4f O_B_T = sub_to_base_link_ * T;
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
