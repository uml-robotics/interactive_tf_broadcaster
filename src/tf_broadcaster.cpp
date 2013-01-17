/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <geometry_msgs/Pose.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;
interactive_markers::MenuHandler menu_handler;

ros::Publisher pub_pose;
void publishMarkerPose()
{
  InteractiveMarker marker;
  server->get("interactive_tf", marker);
  tf::Quaternion qt;
  tf::quaternionMsgToTF(marker.pose.orientation, qt);
  pub_pose.publish(marker.pose);
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void frameCallback(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;
  tf::Transform t;
  ros::Time time = ros::Time::now();

  InteractiveMarker marker;
  server->get("interactive_tf", marker);
  tf::poseMsgToTF(marker.pose, t);
  br.sendTransform(tf::StampedTransform(t, time, marker.header.frame_id, "base_link"));
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
        << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      ROS_INFO_STREAM(
          s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
      if (feedback->menu_entry_id == 1)
      {
        publishMarkerPose();
      }

      break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      ROS_INFO_STREAM(
          s.str() << ": pose changed" << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: " << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
      {
        InteractiveMarker marker;
        server->get(feedback->marker_name, marker);
        tf::Quaternion qt;
        tf::quaternionMsgToTF(feedback->pose.orientation, qt);
        std::stringstream ss;
        ss //
        << " " << feedback->pose.position.x //
            << ", " << feedback->pose.position.y //
            << ", " << tf::getYaw(qt);
        marker.description = ss.str();
        server->insert(marker);
      }
      break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
  }

  server->applyChanges();
}

void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  ROS_INFO_STREAM(
      feedback->marker_name << ":" << " aligning position = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << " to " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z);

  server->setPose(feedback->marker_name, pose);
  server->applyChanges();
}

double rand(double min, double max)
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t * (max - min);
}

void saveMarker(InteractiveMarker int_marker)
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

void make6DofMarker(bool fixed)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.pose.position.y = -3.0 * marker_pos++;
  ;
  int_marker.scale = 1;

  int_marker.name = "interactive_tf";
  int_marker.description = "Interactive TF Publisher";

  // insert a box
//  makeBoxControl(int_marker);

  {
    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);
  }

  InteractiveMarkerControl control;

  if (fixed)
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
//  control.name = "move_z";
//  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  int_marker.pose.orientation.w = 1;
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}
//
//void makeMenuMarker()
//{
//  InteractiveMarker int_marker;
//  int_marker.header.frame_id = "base_link";
//  int_marker.pose.position.y = -3.0 * marker_pos++;
//  ;
//  int_marker.scale = 1;
//
//  int_marker.name = "context_menu";
//  int_marker.description = "Context Menu\n(Right Click)";
//
//  InteractiveMarkerControl control;
//
//  control.interaction_mode = InteractiveMarkerControl::MENU;
//  control.name = "menu_only_control";
//
//  Marker marker = makeBox(int_marker);
//  control.markers.push_back(marker);
//  control.always_visible = true;
//  int_marker.controls.push_back(control);
//
//  server->insert(int_marker);
//  server->setCallback(int_marker.name, &processFeedback);
//  menu_handler.apply(*server, int_marker.name);
//}

//void makeButtonMarker()
//{
//  InteractiveMarker int_marker;
//  int_marker.header.frame_id = "base_link";
//  int_marker.pose.position.y = -3.0 * marker_pos++;;
//  int_marker.scale = 1;
//
//  int_marker.name = "button";
//  int_marker.description = "Button\n(Left Click)";
//
//  InteractiveMarkerControl control;
//
//  control.interaction_mode = InteractiveMarkerControl::BUTTON;
//  control.name = "button_control";
//
//  Marker marker = makeBox( int_marker );
//  control.markers.push_back( marker );
//  control.always_visible = true;
//  int_marker.controls.push_back(control);
//
//  server->insert(int_marker);
//  server->setCallback(int_marker.name, &processFeedback);
//}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;
  pub_pose = n.advertise<geometry_msgs::Pose>("/measurement/pose_truth", 1);

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("Publish", &processFeedback);
//  menu_handler.insert("Second Entry", &processFeedback);
//  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Submenu");
//  menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
//  menu_handler.insert(sub_menu_handle, "Second Entry", &processFeedback);

  make6DofMarker(false);
//  make6DofMarker( true );
//  makeMenuMarker();

  server->applyChanges();

  ros::spin();

  server.reset();
}
