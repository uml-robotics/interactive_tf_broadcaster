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
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <angles/angles.h>

#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;
interactive_markers::MenuHandler menu_handler;

ros::Publisher pub_marker_pose;
ros::Publisher pub_pose_diff;
ros::Publisher pub_distance;
ros::Publisher pub_yaw_diff;

tf::TransformListener * tf_listener;
void publishMeasurements()
{
  InteractiveMarker marker;
  server->get("interactive_tf", marker);
  geometry_msgs::PoseStamped marker_pose;
  marker_pose.pose = marker.pose;
  marker_pose.header = marker.header;
  pub_marker_pose.publish(marker_pose);
  {
    ros::Time transform_time = ros::Time::now();

    geometry_msgs::PoseStamped base_link_pose;
    base_link_pose.header.stamp = transform_time;
    base_link_pose.header.frame_id = "/base_link";
    base_link_pose.pose.orientation.w = 1;
    // Transform camera pose into visual_odom frame
    try
    {
      tf_listener->waitForTransform("/map", "/base_link", transform_time, ros::Duration(1));
      tf_listener->transformPose("/map", base_link_pose, base_link_pose);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    tf::Quaternion qt1;
    tf::quaternionMsgToTF(marker.pose.orientation, qt1);
    tf::Quaternion qt2;
    tf::quaternionMsgToTF(base_link_pose.pose.orientation, qt2);

    ROS_DEBUG_STREAM("Robot Position: "<< base_link_pose.pose.position.x <<" "<<base_link_pose.pose.position.y);
    base_link_pose.pose.position.x -= marker.pose.position.x;
    base_link_pose.pose.position.y -= marker.pose.position.y;
    base_link_pose.pose.position.z -= marker.pose.position.z;
    base_link_pose.pose.orientation.x = 0;
    base_link_pose.pose.orientation.y = 0;
    base_link_pose.pose.orientation.z = 0;
    base_link_pose.pose.orientation.w = 1;

    pub_pose_diff.publish(base_link_pose);
    std_msgs::Float32 distance;
    distance.data = pow(pow(base_link_pose.pose.position.x, 2) + pow(base_link_pose.pose.position.y, 2), 0.5);
    pub_distance.publish(distance);

    std_msgs::Float32 yaw_diff;
    yaw_diff.data = fabs(angles::normalize_angle(tf::getYaw(qt1) - tf::getYaw(qt2)));
    pub_yaw_diff.publish(yaw_diff);
  }
}

void alignWithEstimate()
{
  ROS_INFO("Aligning");
  InteractiveMarker marker;
  server->get("interactive_tf", marker);

  geometry_msgs::PoseStamped base_link_pose;
  {
    ros::Time transform_time = ros::Time::now();

    base_link_pose.header.stamp = transform_time;
    base_link_pose.header.frame_id = "/base_link";
    base_link_pose.pose.orientation.w = 1;
    // Transform camera pose into visual_odom frame
    try
    {
      tf_listener->waitForTransform("/map", "/base_link", transform_time, ros::Duration(1));
      tf_listener->transformPose("/map", base_link_pose, base_link_pose);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }

  marker.pose = base_link_pose.pose;
  server->insert(marker);
}

Marker makeBox(InteractiveMarker & msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.3;

  return marker;
}

InteractiveMarkerControl & makeBoxControl(InteractiveMarker & msg)
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
      ROS_DEBUG_STREAM( s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      ROS_DEBUG_STREAM(
          s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
      switch (feedback->menu_entry_id)
      {
        case (3):
          publishMeasurements();
          break;
        case (1):
          alignWithEstimate();
          break;
      }

      break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      ROS_DEBUG_STREAM(
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
      ROS_DEBUG_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_DEBUG_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
  }

  server->applyChanges();
}

void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x - 0.5) + 0.5;
  pose.position.y = round(pose.position.y - 0.5) + 0.5;

  ROS_DEBUG_STREAM(
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
  int_marker.scale = 5;

  int_marker.name = "interactive_tf";
  int_marker.description = "Interactive TF Publisher";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move";
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  int_marker.pose.orientation.w = 1;
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  tf_listener = new tf::TransformListener();
  ros::NodeHandle n;
  pub_marker_pose = n.advertise<geometry_msgs::PoseStamped>("/measurement/pose_truth", 1);
  pub_pose_diff = n.advertise<geometry_msgs::PoseStamped>("/measurement/pose_diff", 1);
  pub_distance = n.advertise<std_msgs::Float32>("/measurement/distance", 1);
  pub_yaw_diff = n.advertise<std_msgs::Float32>("/measurement/yaw_diff", 1);

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("Align with pose estimate", &processFeedback);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Save");
  menu_handler.insert(sub_menu_handle, "Publish the measurement", &processFeedback);

  make6DofMarker(false);

  server->applyChanges();

  ros::spin();

  server.reset();
}
