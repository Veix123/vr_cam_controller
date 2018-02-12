///////////////////////////////////////////////////////////////////////////////
//      Title     : vr_cam_controller.h
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

#ifndef VR_CAM_CONTROLLER_H
#define VR_CAM_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Joy.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MAX_LIN_VEL 2.0
#define MAX_ANG_VEL 1.0

// Close-up limit to prevent moving through the tracked object
#define R_LIMIT 0.2

// Limit up and down motions to be less that PI from the horizon
// This will prevent singularities at upper and lower poles
#define THETA_LIMIT 0.9 * M_PI / 2

class CamController
{
public:
  CamController();
  ~CamController();

private:

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  ros::NodeHandle nh_;

  // subscriber and muxed publishers of joy messages
  ros::Subscriber sub_;

  tf2_ros::TransformBroadcaster br_;

  std::string my_parent_frame_;
  ros::Time last_time_;

  double integrated_r_;
  double integrated_phi_;
  double integrated_theta_;
};
#endif
