#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class LidarImuOdometryNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  LidarImuOdometryNode()
  : Node("lidar_imu_odometry"),
    initialized_(false),
    pose_x_(0.0),
    pose_y_(0.0),
    pose_yaw_(0.0),
    imu_yaw_delta_(0.0),
    previous_imu_time_(0.0),
    have_imu_(false)
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("imu_topic", "/imu/data");

    declare_parameter<std::string>("odom_topic", "/odom_lidar_imu");

    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("min_range", 0.15);
    declare_parameter<double>("max_range", 8.0);

    declare_parameter<double>("max_correspondence_distance", 0.60);
    declare_parameter<int>("max_iterations", 40);

    declare_parameter<double>("transformation_epsilon", 1e-5);
    declare_parameter<double>("euclidean_fitness_epsilon", 1e-4);

    declare_parameter<double>("imu_yaw_weight", 0.30);

    scan_topic_ =
      get_parameter("scan_topic").as_string();

    imu_topic_ =
      get_parameter("imu_topic").as_string();

    odom_topic_ =
      get_parameter("odom_topic").as_string();

    odom_frame_ =
      get_parameter("odom_frame").as_string();

    base_frame_ =
      get_parameter("base_frame").as_string();

    min_range_ =
      get_parameter("min_range").as_double();

    max_range_ =
      get_parameter("max_range").as_double();

    max_correspondence_distance_ =
      get_parameter("max_correspondence_distance").as_double();

    max_iterations_ =
      get_parameter("max_iterations").as_int();

    transformation_epsilon_ =
      get_parameter("transformation_epsilon").as_double();

    euclidean_fitness_epsilon_ =
      get_parameter("euclidean_fitness_epsilon").as_double();

    imu_yaw_weight_ =
      get_parameter("imu_yaw_weight").as_double();

    odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::QoS(20));

    scan_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &LidarImuOdometryNode::scanCallback,
          this,
          _1));

    imu_sub_ =
      create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &LidarImuOdometryNode::imuCallback,
          this,
          _1));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "LiDAR + IMU odometry node initialized.");

    RCLCPP_INFO(
      get_logger(),
      "Scan topic: %s",
      scan_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "IMU topic: %s",
      imu_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Publishing: %s",
      odom_topic_.c_str());
  }

private:

  // ==========================================================
  // IMU CALLBACK
  // ==========================================================

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    const double t =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) *
      1e-9;

    if (!have_imu_)
    {
      previous_imu_time_ = t;
      have_imu_ = true;
      return;
    }

    double dt = t - previous_imu_time_;

    previous_imu_time_ = t;

    if (dt <= 0.0 || dt > 0.20)
    {
      return;
    }

    // Our stabilized WT901 publishes calibrated gyro-Z.
    //
    // rad/s * sec = rad
    //
    imu_yaw_delta_ +=
      msg->angular_velocity.z * dt;

    imu_yaw_delta_ =
      normalizeAngle(imu_yaw_delta_);
  }

  // ==========================================================
  // SCAN CALLBACK
  // ==========================================================

  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    CloudT::Ptr current_cloud =
      laserScanToCloud(*msg);

    if (current_cloud->size() < 30)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Not enough usable LiDAR points: %zu",
        current_cloud->size());

      return;
    }

    // --------------------------------------------------------
    // First scan initializes the local map.
    // --------------------------------------------------------

    if (!initialized_)
    {
      previous_cloud_ = current_cloud;

      initialized_ = true;

      last_scan_time_ =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) *
        1e-9;

      RCLCPP_INFO(
        get_logger(),
        "Initialized with %zu LiDAR points.",
        current_cloud->size());

      publishOdometry(msg->header.stamp);

      return;
    }

    // --------------------------------------------------------
    // Read and reset the IMU delta since the previous scan.
    // --------------------------------------------------------

    double predicted_yaw = 0.0;

    {
      std::lock_guard<std::mutex> lock(imu_mutex_);

      predicted_yaw = imu_yaw_delta_;

      imu_yaw_delta_ = 0.0;
    }

    // --------------------------------------------------------
    // Build initial transform from IMU yaw.
    //
    // Translation is initialized to zero because translation
    // will be estimated by LiDAR registration.
    // --------------------------------------------------------

    Eigen::Matrix4f initial_guess =
      Eigen::Matrix4f::Identity();

    const float c =
      static_cast<float>(std::cos(predicted_yaw));

    const float s =
      static_cast<float>(std::sin(predicted_yaw));

    initial_guess(0, 0) = c;
    initial_guess(0, 1) = -s;
    initial_guess(1, 0) = s;
    initial_guess(1, 1) = c;

    // --------------------------------------------------------
    // PCL ICP
    // --------------------------------------------------------

    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setInputSource(current_cloud);
    icp.setInputTarget(previous_cloud_);

    icp.setMaximumIterations(max_iterations_);

    icp.setMaxCorrespondenceDistance(
      max_correspondence_distance_);

    icp.setTransformationEpsilon(
      transformation_epsilon_);

    icp.setEuclideanFitnessEpsilon(
      euclidean_fitness_epsilon_);

    CloudT aligned;

    icp.align(
      aligned,
      initial_guess);

    if (!icp.hasConverged())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "ICP did not converge.");

      return;
    }

    const double fitness =
      icp.getFitnessScore();

    if (!std::isfinite(fitness))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "ICP produced invalid fitness.");

      return;
    }

    // --------------------------------------------------------
    // Reject obviously bad registrations.
    // --------------------------------------------------------

    if (fitness > 0.50)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Poor ICP match. Fitness = %.4f",
        fitness);

      return;
    }

    const Eigen::Matrix4f T =
      icp.getFinalTransformation();

    double dx =
      static_cast<double>(T(0, 3));

    double dy =
      static_cast<double>(T(1, 3));

    double lidar_dyaw =
      std::atan2(
        static_cast<double>(T(1, 0)),
        static_cast<double>(T(0, 0)));

    // --------------------------------------------------------
    // The ICP transform maps source -> target.
    //
    // For our accumulated vehicle pose we use the inverse
    // relative displacement convention.
    // --------------------------------------------------------

    dx = -dx;
    dy = -dy;
    lidar_dyaw = -lidar_dyaw;

    // --------------------------------------------------------
    // IMU-assisted yaw fusion.
    //
    // The IMU gives us a short-term yaw prediction.
    // LiDAR gives the geometric correction.
    //
    // For this first implementation we blend the two rather
    // than making the IMU overpower the scan matcher.
    // --------------------------------------------------------

    double fused_dyaw =
      (1.0 - imu_yaw_weight_) * lidar_dyaw +
      imu_yaw_weight_ * predicted_yaw;

    fused_dyaw =
      normalizeAngle(fused_dyaw);

    // --------------------------------------------------------
    // Rotate local LiDAR translation into the odom frame.
    // --------------------------------------------------------

    const double cos_yaw =
      std::cos(pose_yaw_);

    const double sin_yaw =
      std::sin(pose_yaw_);

    pose_x_ +=
      cos_yaw * dx -
      sin_yaw * dy;

    pose_y_ +=
      sin_yaw * dx +
      cos_yaw * dy;

    pose_yaw_ += fused_dyaw;

    pose_yaw_ =
      normalizeAngle(pose_yaw_);

    previous_cloud_ =
      current_cloud;

    last_scan_time_ =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) *
      1e-9;

    publishOdometry(msg->header.stamp);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "ICP fitness=%.4f  "
      "dx=%.3f  dy=%.3f  dyaw=%.2f deg  "
      "pose=(%.3f, %.3f, %.2f deg)",
      fitness,
      dx,
      dy,
      fused_dyaw * 180.0 / M_PI,
      pose_x_,
      pose_y_,
      pose_yaw_ * 180.0 / M_PI);
  }

  // ==========================================================
  // LASER SCAN -> 2D POINT CLOUD
  // ==========================================================

  CloudT::Ptr laserScanToCloud(
    const sensor_msgs::msg::LaserScan & scan)
  {
    CloudT::Ptr cloud =
      std::make_shared<CloudT>();

    cloud->reserve(
      scan.ranges.size());

    for (
      std::size_t i = 0;
      i < scan.ranges.size();
      ++i)
    {
      const float r =
        scan.ranges[i];

      if (!std::isfinite(r))
      {
        continue;
      }

      if (r < min_range_ || r > max_range_)
      {
        continue;
      }

      const double angle =
        scan.angle_min +
        static_cast<double>(i) *
        scan.angle_increment;

      // ------------------------------------------------------
      // Strong persistent rover self-mask established from
      // the S3 30-scan analysis.
      //
      // 0-20 deg
      // 240-270 deg
      // 340-360 deg
      // ------------------------------------------------------

      double deg =
        angle * 180.0 / M_PI;

      deg =
        std::fmod(
          deg + 360.0,
          360.0);

      if (
        (deg >= 0.0 && deg < 20.0) ||
        (deg >= 240.0 && deg < 270.0) ||
        (deg >= 340.0 && deg < 360.0))
      {
        continue;
      }

      PointT p;

      p.x =
        r *
        static_cast<float>(
          std::cos(angle));

      p.y =
        r *
        static_cast<float>(
          std::sin(angle));

      p.z = 0.0f;

      cloud->push_back(p);
    }

    cloud->width =
      static_cast<std::uint32_t>(
        cloud->size());

    cloud->height = 1;

    cloud->is_dense = true;

    return cloud;
  }

  // ==========================================================
  // ODOMETRY PUBLISH
  // ==========================================================

  void publishOdometry(
    const builtin_interfaces::msg::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;

    odom.header.frame_id =
      odom_frame_;

    odom.child_frame_id =
      base_frame_;

    odom.pose.pose.position.x =
      pose_x_;

    odom.pose.pose.position.y =
      pose_y_;

    odom.pose.pose.position.z =
      0.0;

    tf2::Quaternion q;

    q.setRPY(
      0.0,
      0.0,
      pose_yaw_);

    odom.pose.pose.orientation.x =
      q.x();

    odom.pose.pose.orientation.y =
      q.y();

    odom.pose.pose.orientation.z =
      q.z();

    odom.pose.pose.orientation.w =
      q.w();

    // Conservative covariance.
    // We are not claiming millimeter-level accuracy.
    odom.pose.covariance[0] =
      0.05;

    odom.pose.covariance[7] =
      0.05;

    odom.pose.covariance[35] =
      0.05;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = stamp;

    tf_msg.header.frame_id =
      odom_frame_;

    tf_msg.child_frame_id =
      base_frame_;

    tf_msg.transform.translation.x =
      pose_x_;

    tf_msg.transform.translation.y =
      pose_y_;

    tf_msg.transform.translation.z =
      0.0;

    tf_msg.transform.rotation.x =
      q.x();

    tf_msg.transform.rotation.y =
      q.y();

    tf_msg.transform.rotation.z =
      q.z();

    tf_msg.transform.rotation.w =
      q.w();

    tf_broadcaster_->sendTransform(
      tf_msg);
  }

  // ==========================================================
  // ANGLE NORMALIZATION
  // ==========================================================

  static double normalizeAngle(
    double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

  // ==========================================================
  // MEMBERS
  // ==========================================================

  std::string scan_topic_;
  std::string imu_topic_;
  std::string odom_topic_;

  std::string odom_frame_;
  std::string base_frame_;

  double min_range_;
  double max_range_;

  double max_correspondence_distance_;
  int max_iterations_;

  double transformation_epsilon_;
  double euclidean_fitness_epsilon_;

  double imu_yaw_weight_;

  bool initialized_;

  double pose_x_;
  double pose_y_;
  double pose_yaw_;

  double last_scan_time_;

  CloudT::Ptr previous_cloud_;

  rclcpp::Subscription<
    sensor_msgs::msg::LaserScan
  >::SharedPtr scan_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::Imu
  >::SharedPtr imu_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry
  >::SharedPtr odom_pub_;

  std::unique_ptr<
    tf2_ros::TransformBroadcaster
  > tf_broadcaster_;

  std::mutex imu_mutex_;

  bool have_imu_;
  double previous_imu_time_;
  double imu_yaw_delta_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<LidarImuOdometryNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
