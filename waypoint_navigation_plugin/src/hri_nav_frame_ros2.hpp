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
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <Eigen/Sparse>
#include "ui_HRI_coworker.h"
//#include <OGRE/OgreVector3.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <OgrePrerequisites.h>

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
class HRI_coworkerWidget;
}

namespace hri_nav_plugin
{
class HriNavTool;
}

namespace hri_nav_plugin
{

class HriFrame : public QWidget
{
  friend class HriNavTool;
  Q_OBJECT

  public:
  HriFrame(rviz_common::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr,
   interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent = 0, HriNavTool* wp_tool=0);
  ~HriFrame();
  interactive_markers::InteractiveMarkerServer* server_;

  void enable();
  void disable();


  void setWpCount(int size);
  //void setConfig(QString topic, QString frame, float height);
  void setWpLabel();
  void setSelectedMarkerName(std::string name);
  void setPose(Eigen::Vector3f position, Eigen::Vector4f quat);


  bool getTopicOveride();
  bool getBernEnable();
  
  double getDefaultHeight();
  double getTime();
  bool get2Ddisplay();
//   QString getFrameId();
//   QString getOutputTopic();
  void getPose(Eigen::Vector3f * position, Eigen::Vector4f * quat);
  void display_corridros();
  void setLimit(Eigen::Vector4d& upper, Eigen::Vector4d& lower, bool enable);
  std::vector<waypoint_ineq_const> ineq_list;
  void push_newIneq_const();
  void ineqChanged(double val,int mode, int axis);


  protected:

  Ui::HRI_coworkerWidget *ui_;
  rviz_common::DisplayContext* context_;

  private Q_SLOTS:

  void motors_on_push_button();
  void motors_off_push_button();
  void land_push_button();
  void takeoff_push_button();
  void goto_push_button();
  void relativeChanged(int b);
  void robotChanged();
  void serviceChanged();
  void goHome_push_button();
  void hover_push_button();
  void clear_map();
  void clear_path();
  void rqt_change_mode_();
  void start_FPVI_task();
  void start_APVI_task();
  void exit_task();
  void rotate_180_yaw();


  private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_clear_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rqt_change_mode;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rqt_input_start_FPVI_task;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rqt_input_start_APVI_task;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rqt_input_rotate_180_yaw;
 
  HriNavTool* hri_nav_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;

  //default height the waypoint must be placed at
  bool relative_ = true;

  //interactive_markers::InteractiveMarkerServer* server_; 
  // The current name of the output topic.
  std::string robot_name = "quadrotor";
  std::string mav_node_name = "mav_services";

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;

};
}

#endif