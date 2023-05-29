// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef POSE_HPP__
#define POSE_HPP__

#include <sstream>

namespace emcl2
{

class Pose
{
public:
  Pose() {}
  Pose(double x, double y, double t);
  Pose(const Pose& p);

  void set(double x, double y, double t);
  void set(const Pose& p);
  std::string to_s();

  void normalizeAngle();
  void move(double length, double direction, double rotation,
            double fw_noise, double rot_noise);

  const Pose operator-(const Pose& p) const;
  Pose& operator=(const Pose& p);

  bool nearlyZero();

  double x_, y_, t_;

  uint16_t get16bitRepresentation();
  static uint16_t get16bitRepresentation(double);
};

} // namespace emcl2

#endif
