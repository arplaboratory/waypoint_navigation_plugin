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

#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include <QWidget>
#include <iostream>
#include "gnuplot.h"
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <mav_manager_srv/Vec4.h>
#include <Eigen/Sparse>
#include "ui_WaypointNavigation.h"
#include <visualization_msgs/MarkerArray.h>


typedef struct {
    int derivOrder, vertexNum;
    Eigen::Vector4d lower, upper;
    bool enable; //Declares wether this constraint is active or not
} waypoint_ineq_const;

  static const std::string Deriv_title[5] = { "'Pos'", "'Vel'", "'accel'", "'jerk'" , "'snap'"};
  static const std::string append[5] = { "Pos", "Vel", "accel", "jerk" , "snap"};

namespace Ogre
{
class SceneNode;
class Vector3;
class SceneManager;
class Quaternion;
}

namespace rviz
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
  WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent = 0, WaypointNavTool* wp_tool=0);
  ~WaypointFrame();

  void enable();
  void disable();

  void setWpCount(int size);
  void setConfig(QString topic, QString frame, float height);
  void setWpLabel(Ogre::Vector3 position);
  void setSelectedMarkerName(std::string name);
  void setPose(Ogre::Vector3& position, Ogre::Quaternion& quat);


  bool getTopicOveride();
  bool getBernEnable();
  
  double getDefaultHeight();
  double getTime();
  bool get2Ddisplay();
  QString getFrameId();
  QString getOutputTopic();
  void getPose(Ogre::Vector3& position, Ogre::Quaternion& quat);
  void display_corridros();
  void setLimit(Eigen::Vector4d& upper, Eigen::Vector4d& lower, bool enable);
  std::vector<waypoint_ineq_const> ineq_list;
  void push_newIneq_const();
  void ineqChanged(double val,int mode, int axis);
protected:

  Ui::WaypointNavigationWidget *ui_;
  rviz::DisplayContext* context_;

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
  void robotChanged();
  void serviceChanged();
  void goHome_push_button();
  void hover_push_button();
  void clear_map();
  void clear_path();
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

  ros::NodeHandle nh_;
  ros::Publisher wp_pub_;
  ros::Publisher pub_corridor_;
  ros::Publisher path_clear_pub_;
  
  ros::Subscriber path_listen_;
  ros::Subscriber vel_listen_;
  ros::Subscriber acc_listen_;

  void pos_listen(const nav_msgs::Path &msg);
  void vel_listen(const nav_msgs::Path &msg);
  void acc_listen(const nav_msgs::Path &msg);
  void display(const nav_msgs::Path &msg, int index);


  WaypointNavTool* wp_nav_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;

  interactive_markers::InteractiveMarkerServer* server_;

  //default height the waypoint must be placed at
  double default_height_;
  double total_time_ = 0.0; //Jeff Addition Total Time of waypopints
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
  visualization_msgs::MarkerArray marker_array;
  std::string selected_marker_name_;

};

}

#endif
