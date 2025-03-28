  /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

//READ THIS NOTICE BEFORE YOU DO ANYTHING RIGHT NOW!!!!!!!

//WE CAN NOT USE OGRE VECTOR operations in waypoint_nav_frame.cpp
//DO ALL OGRE OPERATIONS IN waypoint_nav_tool.cpp FIRST OR COMMAND IN TOOL
//I AM SERIOUS IT IS YOUR OWN FAULT IF YOU DON'T READ THIS NOTICES


#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include <QWidget>
#include <iostream>
#include "gnuplot.h"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <Eigen/Sparse>
#include "ui_WaypointNavigation.h"
//#include <OGRE/OgreVector3.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <OgrePrerequisites.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Geometry>
#include <mav_manager_srv/srv/vec4.hpp>

typedef struct {
    int derivOrder, vertexNum;
    Eigen::Vector4d lower, upper;
    bool enable; //Declares wether this constraint is active or not
} waypoint_ineq_const;

static const std::string Deriv_title[5] = { "'Pos'", "'Vel'", "'accel'", "'jerk'" , "'snap'"};
static const std::string append[5] = { "Pos", "Vel", "accel", "jerk" , "snap"};

namespace Ogre
{
//class SceneNode;
//class Vector3;
//class SceneManager;
//class Quaternion;
}

namespace rviz_common
{
class DisplayContext;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace Ui
{
class WaypointNavigationWidget;
}

namespace waypoint_nav_plugin
{
class WaypointNavTool;
}

namespace waypoint_nav_plugin
{

class WaypointFrame : public QWidget
{
  friend class WaypointNavTool;
  Q_OBJECT

public:
  WaypointFrame(rviz_common::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr,
   interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent = 0, WaypointNavTool* wp_tool=0);
  ~WaypointFrame();
  interactive_markers::InteractiveMarkerServer* server_;

  void enable();
  void disable();

  void setWpCount(int size);
  void setConfig(QString topic, QString frame, float height);
  void setWpLabel();
  void setSelectedMarkerName(std::string name);
  void setPose(Eigen::Vector3f position, Eigen::Vector4f quat);


  bool getTopicOveride();
  bool getBernEnable();
  
  double getDefaultHeight();
  double getTime();
  bool get2Ddisplay();
  QString getFrameId();
  QString getOutputTopic();
  void getPose(Eigen::Vector3f * position, Eigen::Vector4f * quat);
  void display_corridros();
  void setLimit(Eigen::Vector4d& upper, Eigen::Vector4d& lower, bool enable);
  std::vector<waypoint_ineq_const> ineq_list;
  void push_newIneq_const();
  void ineqChanged(double val,int mode, int axis);
  Eigen::Quaternionf getQuatTransform();
  bool getLocalFrameStatus();

protected:

  Ui::WaypointNavigationWidget *ui_;
  rviz_common::DisplayContext* context_;

private Q_SLOTS:
  void publishButtonClicked();
  void clearAllWaypoints();
  void heightChanged(double h);
  void timeChanged(double t);
  void bool2DChanged(int b);
  void bern_enable(int b);
  void replan_enable(int b);
  void topic_enable(int b);

  void frameChanged();
  void topicChanged();
  void poseChanged(double val);


  //void rollChanged(double val);
  //void pitchChanged(double val);

  void saveButtonClicked();
  void loadButtonClicked();
  void perchClicked();

  //Buttons RQT MAV MANAGER
  void motors_on_push_button();
  void motors_off_push_button();
  void land_push_button();
  void takeoff_push_button();
  void goto_push_button();
  void relativeChanged(int b);
  void originChanged(int b);
  void robotChanged();
  void serviceChanged();
  void goHome_push_button();
  void hover_push_button();
  void clear_map();
  void clear_path();
  void tf2_callback();
   //Bernstein Check boxes


  //Inequality cahgned for each double box
  /*
  void pl_ineqChanged(double val);
  void xl_ineqChanged(double val);
  void yl_ineqChanged(double val);
  void zl_ineqChanged(double val);
  void pu_ineqChanged(double val);
  void xu_ineqChanged(double val);
  void yu_ineqChanged(double val);
  void zu_ineqChanged(double val);
  void en_ineqChanged(double val);
  void do_ineqChanged(double val);*/

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_corridor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_clear_pub_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motors_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  hover_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_map_client_;
  rclcpp::Client<mav_manager_srv::srv::Vec4>::SharedPtr go_to_client_;
  rclcpp::Client<mav_manager_srv::srv::Vec4>::SharedPtr go_to_relative_client_;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ path_listen_;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ vel_listen_;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ acc_listen_;

 // void display(const nav_msgs::msg::Path &msg, int index);

  rclcpp::Serialization<nav_msgs::msg::Path> serialization_;
  WaypointNavTool* wp_nav_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;


  //default height the waypoint must be placed at
  double default_height_;
  double total_time_ = 0.0; //Jeff Addition Total Time of waypopints
  double yaw_init_ = 0.0;
  bool display_2D = false; // Jeff additional boolean 
  bool relative_ = true;
  bool bern_enable_ = false;
  bool replan_enable_ = false;
  bool topicOverride = false;
  double roll_=0.0;
  double pitch_=0.0;

  // The current name of the output topic.
  QString output_topic_;
  QString frame_id_;


  // The current name of the output topic.
  std::string robot_name = "quadrotor";
  std::string mav_node_name = "mav_services";

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;
  visualization_msgs::msg::MarkerArray marker_array;
  std::string selected_marker_name_;

  Eigen::Quaternionf quat_transform_;
  bool local_frame_ = false;
};

}

#endif
