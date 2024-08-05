// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef INTERFACE_EMCL2_HPP__
#define INTERFACE_EMCL2_HPP__

#include "emcl/ExpResetMcl2.hpp"
#include "emcl/visibility_control.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

namespace emcl2
{

class EMcl2Node : public nav2_util::LifecycleNode
{
public:
  EMcl2Node(const rclcpp::NodeOptions& options);
  ~EMcl2Node();

  void loop();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
  std::shared_ptr<ExpResetMcl2> pf_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr alpha_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::string footprint_frame_id_;
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string scan_frame_id_;
  std::string base_frame_id_;
  double transform_tolerance_;

  double alpha_threshold_;
  double expansion_radius_position_;
  double expansion_radius_orientation_;
  double extraction_rate_;
  double range_threshold_;
  bool sensor_reset_;
  bool tf_fix_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::Time scan_stamp_;

  bool init_request_;
  bool simple_reset_request_;
  double init_x_, init_y_, init_t_;
  bool first_init_;
  bool map_request_;
  geometry_msgs::msg::TransformStamped map_to_odom_stamped_;

  void publishPose(const rclcpp::Time& stamp,
                   double x, double y, double t,
                   double x_dev, double y_dev, double t_dev,
                   double xy_cov, double yt_cov, double tx_cov);
  void publishOdomFrame(const rclcpp::Time& stamp,
                        double x, double y, double t);
  void publishParticles(const rclcpp::Time& stamp);
  bool getOdomPose(double& x, double& y, double& yaw);
  bool getLidarPose(double& x, double& y, double& yaw, bool& inv);

  void initCommunication();
  std::shared_ptr<OdomModel> initOdometry();

  void cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  bool cbSimpleReset(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                     const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void mapReceived(const nav_msgs::msg::OccupancyGrid& msg);

  static inline std::string boolString(bool value)
  {
    return (value) ? "true" : "false";
  }
};

} // namespace emcl2

#endif
