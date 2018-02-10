///////////////////////////////////////////////////////////////////////////////
//      Title     : vr_cam_controller.cpp
//      Project   : vr_cam_controller
//      Created   : 2/8/2018
//      Author    : Veiko Vunder
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2017-2018. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "vr_cam_controller/vr_cam_controller.h"

#include <exception>
#include <iostream>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

CamController::CamController()
  : nh_("~"), last_time_(0)
{
  // subscribe to spacenav joy events
  std::string topic = "/spacenav_demuxer/cam_joy";
  nh_.getParam("spacenav_topic", topic);
  sub_ = nh_.subscribe(topic, 5, &CamController::joyCallback, this);

  my_parent_frame_ = "world";
  nh_.getParam("parent_frame", my_parent_frame_);

  //start with an identity transform
  integrated_tf_.setIdentity();
}

CamController::~CamController()
{
  sub_.shutdown();
}

void CamController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  double dt = (ros::Time::now() - last_time_).toSec();

  // continue only if dt is available (more than 1 msg is arrived)
  if (last_time_ == ros::Time(0))
  {
    last_time_ = ros::Time::now();
    return;
  }
  last_time_ = ros::Time::now();

  tf2::Quaternion delta_rotation;
  delta_rotation.setRPY(0, msg->axes[2]*MAX_ANG_VEL*dt, msg->axes[1]*MAX_ANG_VEL*dt);
  //delta_rotation *= MAX_ANG_VEL * dt;
  tf2::Vector3 delta_origin(msg->axes[0], 0, 0);
  delta_origin *= MAX_LIN_VEL * dt;
  tf2::Transform delta_tf(delta_rotation, delta_origin);

  integrated_tf_ *= delta_tf;
  tf2::Transform tmp_tf(integrated_tf_.getRotation().inverse(), tf2::Vector3(0,0,0));

 // // Translate first and then rotate along the sphere
 // tf2::Transform trans1(integrated_tf_.getRotation(), tf2::Vector3(0,0,0));
 // tf2::Vector3 rotated_pos = trans1 * integrated_tf_.getOrigin();
 // tf2::Transform cam_tf(integrated_tf_.getRotation(), rotated_pos);
//-------------------------------------------
//   tf2::Vector3 speed_vec(msg->axes[0] * MAX_LIN_SPEED, msg->axes[1] * MAX_ANG_SPEED,
//                          msg->axes[2] * MAX_ANG_SPEED);
//   integrated_vec_ += speed_vec * dt;
// 
//   tf2::Quaternion q;
//   q.setRPY(0, integrated_vec_.z(), -integrated_vec_.y());
//   tf2::Transform trans1(q,tf2::Vector3());
//   tf2::Vector3 pos = trans1 * tf2::Vector3(integrated_vec_.x(), 0, 0);
//   tf2::Transform trans(q, pos);

  geometry_msgs::TransformStamped ts_msg;
  ts_msg.header.stamp = ros::Time::now();
  ts_msg.header.frame_id = my_parent_frame_;
  ts_msg.child_frame_id = "vr_cam";
  ts_msg.transform = toMsg(tmp_tf*integrated_tf_);

  br_.sendTransform(ts_msg);
}

int main(int argc, char* argv[])
{
  // ROS init
  ros::init(argc, argv, "cam_controller");

  try
  {
    CamController cam_controller;
    ros::spin();
  }
  catch(std::exception& e)
  {
    std::cout << "Got exception from CamController: " << e.what()<< std::endl;
  }
}
