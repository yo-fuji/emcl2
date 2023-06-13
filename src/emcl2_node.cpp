// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
// Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl.

#include "emcl/emcl2_node.hpp"
#include "emcl/Pose.hpp"

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(emcl2::EMcl2Node)

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace emcl2
{

EMcl2Node::EMcl2Node(const rclcpp::NodeOptions& options)
  : LifecycleNode("emcl2_node", "", options)
{
  if (!this->has_parameter("odom_freq")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.description = "frequency of odometry update";
    descriptor.read_only = true;
    this->declare_parameter("odom_freq", 20, descriptor);
  }

  if (!this->has_parameter("global_frame_id")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.description = "the frame for localization";
    descriptor.read_only = true;
    this->declare_parameter("global_frame_id", std::string("map"), descriptor);
  }
  if (!this->has_parameter("footprint_frame_id")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.description = "the frame of the localized robot's base";
    descriptor.read_only = true;
    this->declare_parameter("footprint_frame_id", std::string("base_footprint"), descriptor);
  }
  if (!this->has_parameter("odom_frame_id")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.description = "the frame for odometry";
    descriptor.read_only = true;
    this->declare_parameter("odom_frame_id", std::string("odom"), descriptor);
  }
  if (!this->has_parameter("base_frame_id")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.description = "the frame of the robot's base. It is used for calculating the position and orientation of the LiDAR";
    descriptor.read_only = true;
    this->declare_parameter("base_frame_id", std::string("base_link"), descriptor);
  }
  if (!this->has_parameter("use_map_topic")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.description = "when set to true, EMCL will subscribe to the map topic rather than making a service call to receive its map";
    descriptor.read_only = true;
    this->declare_parameter("use_map_topic", true, descriptor);
  }
  if (!this->has_parameter("transform_tolerance")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "time with which to post-date the transform that is published, to indicate that this transform is valid into the future";
    descriptor.read_only = true;
    this->declare_parameter("transform_tolerance", 0.2, descriptor);
  }

  if (!this->has_parameter("laser_min_range")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "threshold for discarding scans whose ranges are smaller than this value";
    descriptor.read_only = true;
    this->declare_parameter("laser_min_range", 0.0, descriptor);
  }
  if (!this->has_parameter("laser_max_range")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "threshold for discarding scans whose ranges are larger than this value";
    descriptor.read_only = true;
    this->declare_parameter("laser_max_range", 100000000.0, descriptor);
  }
  if (!this->has_parameter("scan_increment")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.description = "increment number when beams are picked from their sequence; the larger this number is, the fewer number of beams are used for calculation of likelihood";
    descriptor.read_only = true;
    this->declare_parameter("scan_increment", 1, descriptor);
  }

  if (!this->has_parameter("initial_pose_x")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "initial x coordinate of particles";
    descriptor.read_only = true;
    this->declare_parameter("initial_pose_x", 0.0, descriptor);
  }
  if (!this->has_parameter("initial_pose_y")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "initial y coordinate of particles";
    descriptor.read_only = true;
    this->declare_parameter("initial_pose_y", 0.0, descriptor);
  }
  if (!this->has_parameter("initial_pose_a")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "initial yaw coordinate of particles";
    descriptor.read_only = true;
    this->declare_parameter("initial_pose_a", 0.0, descriptor);
  }

  if (!this->has_parameter("num_particles")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.description = "number of particles";
    descriptor.read_only = true;
    this->declare_parameter("num_particles", 0, descriptor);
  }
  if (!this->has_parameter("alpha_threshold")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "threshold of the alpha value for expansion resetting";
    this->declare_parameter("alpha_threshold", 0.0, descriptor);
  }
  if (!this->has_parameter("expansion_radius_position")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "maximum change of the position on the xy-plane when the reset replaces a particle";
    this->declare_parameter("expansion_radius_position", 0.1, descriptor);
  }
  if (!this->has_parameter("expansion_radius_orientation")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "maximum change of the yaw angle when the reset replaces a particle";
    this->declare_parameter("expansion_radius_orientation", 0.2, descriptor);
  }

  if (!this->has_parameter("extraction_rate")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "rate of particles that are checked by the node";
    this->declare_parameter("extraction_rate", 0.1, descriptor);
  }
  if (!this->has_parameter("range_threshold")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "threshold of the range of lasers; if all lasers on this range penetrate occupancy cells, the pose of the particle is judged as wrong";
    this->declare_parameter("range_threshold", 0.1, descriptor);
  }
  if (!this->has_parameter("sensor_reset")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "flag for sensor resettings";
    this->declare_parameter("sensor_reset", false, descriptor);
  }

  if (!this->has_parameter("odom_fw_dev_per_fw")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "standard deviation of forward motion noise by forward motion";
    descriptor.read_only = true;
    this->declare_parameter("odom_fw_dev_per_fw", 0.19, descriptor);
  }
  if (!this->has_parameter("odom_fw_dev_per_rot")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "standard deviation of forward motion noise by rotational motion";
    descriptor.read_only = true;
    this->declare_parameter("odom_fw_dev_per_rot", 0.0001, descriptor);
  }
  if (!this->has_parameter("odom_rot_dev_per_fw")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "standard deviation of rotational motion noise by forward motion";
    descriptor.read_only = true;
    this->declare_parameter("odom_rot_dev_per_fw", 0.13, descriptor);
  }
  if (!this->has_parameter("odom_rot_dev_per_rot")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "standard deviation of rotational motion noise by rotational motion";
    descriptor.read_only = true;
    this->declare_parameter("odom_rot_dev_per_rot", 0.2, descriptor);
  }

  if (!this->has_parameter("laser_likelihood_max_dist")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.description = "maximum distance to inflate occupied cells on the likelihood field map";
    descriptor.read_only = true;
    this->declare_parameter("laser_likelihood_max_dist", 0.2, descriptor);
  }
}

EMcl2Node::~EMcl2Node()
{
}

nav2_util::CallbackReturn
EMcl2Node::on_configure(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Configuring");

  initCommunication();

  int odom_freq;
  odom_freq = this->get_parameter("odom_freq").as_int();

  loop_timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(1e6 / odom_freq)),
    std::bind(&EMcl2Node::loop, this));

  scan_stamp_ = this->now();
  init_request_ = false;
  simple_reset_request_ = false;
  map_request_ = false;
  first_init_ = true;

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
EMcl2Node::on_activate(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Activating");

  particlecloud_pub_->on_activate();
  pose_pub_->on_activate();
  alpha_pub_->on_activate();

  alpha_threshold_ = this->get_parameter("alpha_threshold").as_double();
  expansion_radius_position_ = this->get_parameter("expansion_radius_position").as_double();
  expansion_radius_orientation_ = this->get_parameter("expansion_radius_orientation").as_double();
  extraction_rate_ = this->get_parameter("extraction_rate").as_double();
  range_threshold_ = this->get_parameter("range_threshold").as_double();
  sensor_reset_ = this->get_parameter("sensor_reset").as_bool();

  auto node = shared_from_this();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&EMcl2Node::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
EMcl2Node::on_deactivate(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Deactivating");

  dyn_params_handler_.reset();

  particlecloud_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  alpha_pub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
EMcl2Node::on_cleanup(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Cleaning up");

  loop_timer_.reset();

  tfl_.reset();
  tf_.reset();
  tfb_.reset();

  static_map_srv_.reset();

  global_loc_srv_.reset();

  map_sub_.reset();
  initial_pose_sub_.reset();
  laser_scan_sub_.reset();

  alpha_pub_.reset();
  pose_pub_.reset();
  particlecloud_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
EMcl2Node::on_shutdown(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
EMcl2Node::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  bool update_pf = false;

  for (auto parameter : parameters) {
    const auto& type = parameter.get_type();
    const auto& name = parameter.get_name();

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      double value = parameter.as_double();
      if (name == "alpha_threshold") {
        alpha_threshold_ = value;
        update_pf = true;
      } else if (name == "expansion_radius_position") {
        expansion_radius_position_ = value;
        update_pf = true;
      } else if (name == "expansion_radius_orientation") {
        expansion_radius_orientation_ = value;
        update_pf = true;
      } else if (name == "extraction_rate") {
        extraction_rate_ = value;
        update_pf = true;
      } else if (name == "range_threshold") {
        range_threshold_ = value;
        update_pf = true;
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown name: %s: %f", name.c_str(), value);
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      bool value = parameter.as_bool();
      if (name == "sensor_reset") {
        sensor_reset_ = value;
        update_pf = true;
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown name: %s: %s", name.c_str(), boolString(value).c_str());
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Not supported type");
    }
  }

  if (update_pf) {
    RCLCPP_INFO(this->get_logger(), "alpha_threshold: %f", alpha_threshold_);
    RCLCPP_INFO(this->get_logger(), "expansion_radius_position: %f", expansion_radius_position_);
    RCLCPP_INFO(this->get_logger(), "expansion_radius_orientation: %f", expansion_radius_orientation_);
    RCLCPP_INFO(this->get_logger(), "extraction_rate: %f", extraction_rate_);
    RCLCPP_INFO(this->get_logger(), "range_threshold: %f", range_threshold_);
    RCLCPP_INFO(this->get_logger(), "sensor_reset: %s", boolString(sensor_reset_).c_str());

    pf_->paramsUpdate(alpha_threshold_,
                      expansion_radius_position_, expansion_radius_orientation_,
                      extraction_rate_, range_threshold_, sensor_reset_);
  }

  result.successful = true;
  return result;
}

void EMcl2Node::initCommunication()
{
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "particlecloud", rclcpp::SensorDataQoS());
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  alpha_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "alpha", rclcpp::SensorDataQoS());

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&EMcl2Node::cbScan, this, _1));
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&EMcl2Node::initialPoseReceived, this, _1));

  bool use_map_topic = this->get_parameter("use_map_topic").as_bool();
  if (use_map_topic) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&EMcl2Node::mapReceived, this, _1));
  } else {
    static_map_srv_ = this->create_client<nav_msgs::srv::GetMap>("static_map");
  }

  global_loc_srv_ = this->create_service<std_srvs::srv::Empty>(
    "global_localization", std::bind(&EMcl2Node::cbSimpleReset, this, _1, _2, _3));

  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  footprint_frame_id_ = this->get_parameter("footprint_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();

  tfb_.reset(new tf2_ros::TransformBroadcaster(this));
  tf_.reset(new tf2_ros::Buffer(this->get_clock()));
  tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

std::shared_ptr<OdomModel> EMcl2Node::initOdometry()
{
  double ff, fr, rf, rr;
  ff = this->get_parameter("odom_fw_dev_per_fw").as_double();
  fr = this->get_parameter("odom_fw_dev_per_rot").as_double();
  rf = this->get_parameter("odom_rot_dev_per_fw").as_double();
  rr = this->get_parameter("odom_rot_dev_per_rot").as_double();

  return std::make_shared<OdomModel>(ff, fr, rf, rr);
}

void EMcl2Node::cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  scan_frame_id_ = msg->header.frame_id;
  scan_stamp_ = msg->header.stamp;

  if (!pf_) {
    return;
  }
  pf_->setScan(msg);
}

void EMcl2Node::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  init_request_ = true;
  init_x_ = msg->pose.pose.position.x;
  init_y_ = msg->pose.pose.position.y;
  init_t_ = tf2::getYaw(msg->pose.pose.orientation);
}

void EMcl2Node::mapReceived(const nav_msgs::msg::OccupancyGrid& msg)
{
  double likelihood_range;
  likelihood_range = this->get_parameter("laser_likelihood_max_dist").as_double();

  auto map = std::make_shared<LikelihoodFieldMap>(msg, likelihood_range);
  auto om = std::move(initOdometry());

  Scan scan;
  scan.range_min_ = this->get_parameter("laser_min_range").as_double();
  scan.range_max_ = this->get_parameter("laser_max_range").as_double();
  scan.scan_increment_ = this->get_parameter("scan_increment").as_int();

  Pose init_pose;
  init_pose.x_ = this->get_parameter("initial_pose_x").as_double();
  init_pose.y_ = this->get_parameter("initial_pose_y").as_double();
  init_pose.t_ = this->get_parameter("initial_pose_a").as_double();
  if (first_init_) {
    first_init_ = false;
  } else {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_->lookupTransform(global_frame_id_, footprint_frame_id_,
                                               this->now(), rclcpp::Duration::from_seconds(transform_tolerance_));
      init_pose.x_ = transform_stamped.transform.translation.x;
      init_pose.y_ = transform_stamped.transform.translation.y;
      init_pose.t_ = tf2::getYaw(transform_stamped.transform.rotation);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to get current pose as initial pose (%s)", e.what());
    }
  }

  int num_particles;
  num_particles = this->get_parameter("num_particles").as_int();

  pf_.reset(new ExpResetMcl2(init_pose, num_particles, scan, om, map,
                             alpha_threshold_, expansion_radius_position_, expansion_radius_orientation_,
                             extraction_rate_, range_threshold_, sensor_reset_));
}

void EMcl2Node::loop()
{
  if (static_map_srv_ && (!map_request_)) {
    static_map_srv_->wait_for_service();

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto service_callback = [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
      mapReceived(future.get()->map);
    };
    static_map_srv_->async_send_request(request, service_callback);
    map_request_ = true;
  }
  if (!pf_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "map not received.");
    return;
  }

  if (init_request_) {
    pf_->initialize(init_x_, init_y_, init_t_);
    init_request_ = false;
  } else if (simple_reset_request_) {
    pf_->simpleReset();
    simple_reset_request_ = false;
  }

  double x, y, t;
  if (not getOdomPose(x, y, t)) {
    RCLCPP_INFO(this->get_logger(), "can't get odometry info");
    return;
  }
  pf_->motionUpdate(x, y, t);

  double lx, ly, lt;
  bool inv;
  if (not getLidarPose(lx, ly, lt, inv)) {
    RCLCPP_INFO(this->get_logger(), "can't get lidar pose info");
    return;
  }

  /*
  struct timespec ts_start, ts_end;
  clock_gettime(CLOCK_REALTIME, &ts_start);
  */
  pf_->sensorUpdate(lx, ly, lt, inv);
  /*
  clock_gettime(CLOCK_REALTIME, &ts_end);
  struct tm tm;
  localtime_r( &ts_start.tv_sec, &tm);
  printf("START: %02d.%09ld\n", tm.tm_sec, ts_start.tv_nsec);
  localtime_r( &ts_end.tv_sec, &tm);
  printf("END: %02d.%09ld\n", tm.tm_sec, ts_end.tv_nsec);
  */

  rclcpp::Time stamp;
  double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
  if (!pf_->meanPose(stamp, x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov)) {
    RCLCPP_INFO(this->get_logger(), "can't get particle mean pose");
    return;
  }

  publishOdomFrame(stamp, x, y, t);
  publishPose(stamp, x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
  publishParticles(stamp);

  std_msgs::msg::Float32 alpha_msg;
  alpha_msg.data = static_cast<float>(pf_->alpha_);
  alpha_pub_->publish(alpha_msg);
}

void EMcl2Node::publishPose(const rclcpp::Time& stamp,
                            double x, double y, double t,
                            double x_dev, double y_dev, double t_dev,
                            double xy_cov, double yt_cov, double tx_cov)
{
  geometry_msgs::msg::PoseWithCovarianceStamped p;
  p.header.frame_id = global_frame_id_;
  p.header.stamp = stamp;
  p.pose.pose.position.x = x;
  p.pose.pose.position.y = y;

  p.pose.covariance[6 * 0 + 0] = x_dev;
  p.pose.covariance[6 * 1 + 1] = y_dev;
  p.pose.covariance[6 * 2 + 2] = t_dev;

  p.pose.covariance[6 * 0 + 1] = xy_cov;
  p.pose.covariance[6 * 1 + 0] = xy_cov;
  p.pose.covariance[6 * 0 + 2] = tx_cov;
  p.pose.covariance[6 * 2 + 0] = tx_cov;
  p.pose.covariance[6 * 1 + 2] = yt_cov;
  p.pose.covariance[6 * 2 + 1] = yt_cov;

  tf2::Quaternion q;
  q.setRPY(0, 0, t);
  tf2::convert(q, p.pose.pose.orientation);

  pose_pub_->publish(p);
}

void EMcl2Node::publishOdomFrame(const rclcpp::Time& stamp,
                                 double x, double y, double t)
{
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, t);
    tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = footprint_frame_id_;
    tmp_tf_stamped.header.stamp = stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);

  } catch (const tf2::TransformException& e) {
    RCLCPP_DEBUG(this->get_logger(), "Failed to subtract base to odom transform (%s)", e.what());
    return;
  }
  tf2::convert(odom_to_map.pose, latest_tf_);

  rclcpp::Time transform_expiration = scan_stamp_ + rclcpp::Duration::from_seconds(transform_tolerance_);
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = transform_expiration;
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

  tfb_->sendTransform(tmp_tf_stamped);
}

void EMcl2Node::publishParticles(const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseArray cloud_msg;
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(pf_->particles_.size());

  for (size_t i = 0; i < pf_->particles_.size(); i++) {
    cloud_msg.poses[i].position.x = pf_->particles_[i].p_.x_;
    cloud_msg.poses[i].position.y = pf_->particles_[i].p_.y_;
    cloud_msg.poses[i].position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pf_->particles_[i].p_.t_);
    tf2::convert(q, cloud_msg.poses[i].orientation);
  }
  particlecloud_pub_->publish(cloud_msg);
}

bool EMcl2Node::getOdomPose(double& x, double& y, double& yaw)
{
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = footprint_frame_id_;
  ident.header.stamp = rclcpp::Time(0);
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  geometry_msgs::msg::PoseStamped odom_pose;
  try {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

bool EMcl2Node::getLidarPose(double& x, double& y, double& yaw, bool& inv)
{
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = scan_frame_id_;
  ident.header.stamp = rclcpp::Time(0);
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  geometry_msgs::msg::PoseStamped lidar_pose;
  try {
    this->tf_->transform(ident, lidar_pose, base_frame_id_);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to compute lidar pose, skipping scan (%s)", e.what());
    return false;
  }
  x = lidar_pose.pose.position.x;
  y = lidar_pose.pose.position.y;

  double roll, pitch;
  tf2::getEulerYPR(lidar_pose.pose.orientation, yaw, pitch, roll);
  inv = (fabs(pitch) > M_PI / 2 || fabs(roll) > M_PI / 2) ? true : false;

  return true;
}

bool EMcl2Node::cbSimpleReset(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                              const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                              std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  return simple_reset_request_ = true;
}

} // namespace emcl2
