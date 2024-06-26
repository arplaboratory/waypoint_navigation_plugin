/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: JEFFREY MAO - MODIFIED TO ROS2 */

#ifndef WAYPOINT_NAV_FLAG_TOOL_H
#define WAYPOINT_NAV_FLAG_TOOL_H

#include <rviz_common/panel.hpp>
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include <rcutils/logging_macros.h>
#include <rviz_common/tool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

//#include <OGRE/OgreVector3.h>
#include <OgrePrerequisites.h>

#include <thread>
/*
#include "rviz_common/render_panel.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include <rviz_common/panel.hpp>


**/
using std::placeholders::_1;
#include "waypoint_nav_frame_ros2.hpp"
namespace rviz_common::properties
{
class VectorProperty;
}

namespace rviz_common
{
class VisualizationManager;
class ViewportMouseEvent;
class PanelDockWidget;
class render_panel;
class display_context;
class panel;
class WindowManagerInterface;
}



namespace Ui
{
class QuadrotorSteeringWidget;
}

namespace waypoint_nav_plugin
{

class WaypointNavTool: public rviz_common::Tool
{

Q_OBJECT
public:
  WaypointNavTool();
  ~WaypointNavTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;
  void makeIm(const Ogre::Vector3& position, const Ogre::Quaternion& quat);
  void spin();
  bool setServerPose(int index, Eigen::Vector3f pos_eigen, Eigen::Vector4f quat_eigen);
  //CLEAR
  void clearAllWaypoints();
  //SAVE
  void savePoints(std::string filn);
  //Load
  void loadPoints(std::string filn);
  //GET GEOMETRY MSGS PATH
  nav_msgs::msg::Path getPath();

private:
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  void getMarkerPoses();

  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;

  // the waypoint nav Qt frame
  WaypointFrame *frame_;
  //
  bool first_time_ = true;
  rviz_common::PanelDockWidget* frame_dock_;

  interactive_markers::InteractiveMarkerServer * server_;
  interactive_markers::MenuHandler menu_handler_;

  //map that stores waypoints based on unique names
  typedef std::map<int, Ogre::SceneNode* > M_StringToSNPtr;
  M_StringToSNPtr sn_map_;
  rviz_common::properties::VectorProperty * current_flag_property_;
  rclcpp::Node::SharedPtr nh_;
  //index used for creating unique marker names
  int unique_ind_; 
  std::shared_ptr<std::thread>  thread_;
  rclcpp::executors::StaticSingleThreadedExecutor exec_;
};

} // end namespace waypoint_nav_plugin

#endif // WAYPOINT_NAV_TOOL_H
