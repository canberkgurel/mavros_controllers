/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller Node
 *
 * Geometric controller ROS Node Implementation
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  geometricCtrl* geometricController = new geometricCtrl(nh, nh_private);

  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig> srv;
  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig>::CallbackType f;
  f = boost::bind(&geometricCtrl::dynamicReconfigureCallback, geometricController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}

Eigen::Vector4d geometricCtrl::jerkcontroller(Eigen::Vector3d &ref_jerk, Eigen::Vector3d &ref_acc, Eigen::Vector3d &ref_vel, Eigen::Vector4d &curr_att){
  //Feedforward control from Lopez(2016)
  Eigen::Vector4d ratecmd;
  Eigen::Vector3d jerk, jerk_fb, acc_fb, jerk_vector, ratecmd_pre;
  Eigen::Matrix3d R;

  //TODO: calculate jerk_fb from acc_reference
  // jerk_fb = calc(ref_acc, ref_vel, ref_pos);
  jerk = ref_jerk + jerk_fb;
  jerk_vector = jerk / jerk.norm() - ref_acc*ref_acc.dot(jerk) / std::pow(jerk.norm(), 3); //TODO: is ref_acc ?

  R = quat2RotMatrix(curr_att);
  ratecmd_pre = R.transpose() * jerk_vector;

  ratecmd(0) =  (-1.0)* ratecmd_pre(2);
  ratecmd(2) = ratecmd_pre(1);
  ratecmd(3) = 0.0;

  return ratecmd;
}