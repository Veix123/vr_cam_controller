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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

CamController::CamController()
  : nh_("~")
{
  // subscribe to spacenav joy events
  std::string topic = "/spacenav_demuxer/cam_joy";
  nh_.getParam("spacenav_topic", topic);
  sub_ = nh_.subscribe(topic, 5, &CamController::joyCallback, this);

  my_parent_frame_ = "world";
  nh_.getParam("parent_frame", my_parent_frame_);
}

CamController::~CamController()
{
  sub_.shutdown();
}

void CamController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::TransformStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = my_parent_frame_;
  ts.child_frame_id = "vr_cam";
  tf2::Quaternion q;
  q.setRPY(0.1,0,0);
  ts.transform.translation.x = msg->axes[0];
  ts.transform.rotation = tf2::toMsg(q);


  br_.sendTransform(ts);

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
