// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef PF_HPP__
#define PF_HPP__

#include <random>
#include <sstream>
#include <vector>

#include "emcl/LikelihoodFieldMap.hpp"
#include "emcl/OdomModel.hpp"
#include "emcl/Particle.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace emcl2
{

class Mcl
{
public:
  Mcl() {}
  Mcl(const Pose& p, int num, const Scan& scan,
      const std::shared_ptr<OdomModel>& odom_model,
      const std::shared_ptr<LikelihoodFieldMap>& map);
  ~Mcl();

  std::vector<Particle> particles_;
  double alpha_;

  void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);
  void motionUpdate(double x, double y, double t);

  void initialize(double x, double y, double t);

  void setScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  bool meanPose(rclcpp::Time& stamp,
                double& x_mean, double& y_mean, double& t_mean,
                double& x_var, double& y_var, double& t_var,
                double& xy_cov, double& yt_cov, double& tx_cov);

  void simpleReset();

protected:
  Pose* last_odom_;
  Pose* prev_odom_;

  Scan scan_;
  int processed_seq_;
  int msg_seq_;

  double normalizeAngle(double t);
  void resampling();
  double normalizeBelief();
  void resetWeight();

  std::shared_ptr<OdomModel> odom_model_;
  std::shared_ptr<LikelihoodFieldMap> map_;
};

extern double cos_[];
extern double sin_[];

} // namespace emcl2

#endif
