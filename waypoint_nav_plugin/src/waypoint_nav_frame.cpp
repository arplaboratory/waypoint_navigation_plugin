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
/* Author: JEFFREY MAO - MODIFIED TO ROS2 */

#include <OGRE/OgreSceneManager.h>
//#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.hpp>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <fstream>
#include "waypoint_nav_tool.hpp"
//#include "waypoint_nav_frame.h"
#include <OGRE/OgreVector3.h>

#include <QFileDialog>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace waypoint_nav_plugin
{

WaypointFrame::WaypointFrame(rviz_common::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, 
interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent, WaypointNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::WaypointNavigationWidget())
  , sn_map_ptr_(map_ptr)
  , unique_ind_(unique_ind)
  , server_(server)
  , frame_id_("world")
  , default_height_(0.0)
  , selected_marker_name_("waypoint1")
  , wp_nav_tool_(wp_tool)
{
  //scene_manager_ = context_->getSceneManager();

  // set up the GUI 
  node = rclcpp::Node::make_shared("wp_node");
  ui_->setupUi(this);
  pub_corridor_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("corridors", 1); 
  wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("waypoints", 1);
  path_clear_pub_ = node->create_publisher<std_msgs::msg::Bool>("/clear", 1);
  //connect the Qt signals and slots
  connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
  connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
  connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
  connect(ui_->TimeBox, SIGNAL(valueChanged(double)), this, SLOT(timeChanged(double)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));
  connect(ui_->display2D, SIGNAL(stateChanged(int)), this, SLOT(bool2DChanged(int)));

  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));

  connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));
  
  connect(ui_->perch, SIGNAL(clicked()), this, SLOT(perchClicked()));

  //ROSRUN RQT Mav Manager Line topics 
  connect(ui_->robot_name_line_edit, SIGNAL(editingFinished()), this, SLOT(robotChanged()));
  connect(ui_->node_name_line_edit, SIGNAL(editingFinished()), this, SLOT(serviceChanged()));
  //Buttons
  connect(ui_->motors_on_push_button, SIGNAL(clicked()), this, SLOT(motors_on_push_button()));
  connect(ui_->motors_off_push_button, SIGNAL(clicked()), this, SLOT(motors_off_push_button()));
  connect(ui_->land_push_button, SIGNAL(clicked()), this, SLOT(land_push_button()));
  connect(ui_->takeoff_push_button, SIGNAL(clicked()), this, SLOT(takeoff_push_button()));
  connect(ui_->goto_push_button, SIGNAL(clicked()), this, SLOT(goto_push_button()));
  connect(ui_->relative_checkbox, SIGNAL(stateChanged(int)), this, SLOT(relativeChanged(int)));
  connect(ui_->go_home_button, SIGNAL(clicked()), this, SLOT(goHome_push_button()));
  connect(ui_->hover, SIGNAL(clicked()), this, SLOT(hover_push_button()));
  connect(ui_->bern_enable, SIGNAL(stateChanged(int)), this, SLOT(bern_enable(int)));
  connect(ui_->replan_enable, SIGNAL(stateChanged(int)), this, SLOT(replan_enable(int)));
  connect(ui_->topic_overide, SIGNAL(stateChanged(int)), this, SLOT(topic_enable(int)));
  connect(ui_->motors_off_push_button, SIGNAL(clicked()), this, SLOT(motors_off_push_button()));
  connect(ui_->clear_path, SIGNAL(clicked()), this, SLOT(clear_path()));
  
  connect(ui_->reset_map, SIGNAL(clicked()), this, SLOT(clear_map()));
  
  node->declare_parameter("/"+ robot_name+"/"+"replan",false);
  node->declare_parameter("/"+ robot_name+"/"+"bern_enable",false);
	//path_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_pos", 10, &WaypointFrame::pos_listen, this);
	//vel_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_vel", 10, &WaypointFrame::vel_listen, this);
	//acc_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_acc", 10, &WaypointFrame::acc_listen, this);
}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}

void WaypointFrame::enable()
{
  // activate the frame
  show();
}

void WaypointFrame::disable()
{
  //wp_pub_.shutdown();
  hide();
}

void WaypointFrame::clear_path()
{
   std_msgs::msg::Bool thing;
   path_clear_pub_->publish(thing);
}


void WaypointFrame::bern_enable(int b)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  bern_enable_ = true;
  }
  else{
	  bern_enable_ = false;
  }

}


void WaypointFrame::saveButtonClicked()
{
  QString filename = QFileDialog::getSaveFileName(0,tr("Save Bag"), "waypoints", tr("Bag Files (*.txt)"));
  if(filename == "")
   std::cout << " NO FILE NAME GIVEN!!!" <<std::endl;
  else
  {
    QFileInfo info(filename);
    std::string filn = info.absolutePath().toStdString() + "/" + info.baseName().toStdString() + ".txt";
    wp_nav_tool_->savePoints(filn);
  }   
}

void WaypointFrame::loadButtonClicked()
{
  QString filename = QFileDialog::getOpenFileName(0,tr("Open Bag"), "~/", tr("Bag Files (*.txt)"));
  if(filename == "")
   std::cout << " NO FILE NAME GIVEN!!!" <<std::endl;
  else
  {
    wp_nav_tool_->loadPoints(filename.toStdString());
  }
}


void WaypointFrame::push_newIneq_const(){
  //inequality helper function
  waypoint_ineq_const ineq_const;
  ineq_const.derivOrder = 0;
  Eigen::Vector4d lower, upper;
  lower << 0.0, 0.0, 0.0, 0;
  upper << 0.0, 0.0, 0.0, 0;
  ineq_const.lower = lower;
  ineq_const.upper = upper;
  ineq_const.enable = false;
  ineq_list.push_back(ineq_const);
  setLimit( upper,  lower, false);
}



void WaypointFrame::setLimit(Eigen::Vector4d& upper, Eigen::Vector4d& lower, bool enable)
{

}


void WaypointFrame::publishButtonClicked()
{
  if(!getTopicOveride()){
    std::string topic_name = "/waypoints";
    /*if(getBernEnable()){
      topic_name = "/b_waypoints";
    }*/
    wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+ robot_name +topic_name, 1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  nav_msgs::msg::Path path;
  path = wp_nav_tool_->getPath();
  path.header.frame_id = frame_id_.toStdString();
  wp_pub_->publish(path);
}


void WaypointFrame::perchClicked()
{
  nav_msgs::msg::Path path;
  if(!getTopicOveride()){
    std::string topic_name = "/perch";
    if(getBernEnable()){
      topic_name = "/b_perch";
    }
    wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+ robot_name +topic_name, 1);
  }
  path = wp_nav_tool_->getPath();
  path.header.frame_id = frame_id_.toStdString();
  wp_pub_->publish(path);
}



void WaypointFrame::clearAllWaypoints()
{
  //destroy the ogre scene nodes
  //clear the waypoint map and reset index
  //sn_map_ptr_->clear();
  wp_nav_tool_->clearAllWaypoints();
  *unique_ind_=0;

  //clear the interactive markers
  //server_->clear();
  //server_->applyChanges();
}

void WaypointFrame::heightChanged(double h)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  default_height_ = h;
}

void WaypointFrame::timeChanged(double t)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  total_time_ = t;
}

void WaypointFrame::bool2DChanged(int b)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  display_2D = true;
  }
  else{
	 display_2D = false;
  }
}

void WaypointFrame::replan_enable(int b)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  replan_enable_ = true;

  }
  else{
	 replan_enable_ = false;
  }
  //    wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+ robot_name +topic_name, 1);
  node->set_parameters({rclcpp::Parameter("/"+ robot_name+"/"+"replan", replan_enable_)}); 
  //nh_.setParam("/"+ robot_name+"/"+"replan",replan_enable_);
}

void WaypointFrame::topic_enable(int b)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  topicOverride = true;
  }
  else{
	 topicOverride = false;
  }
}


void WaypointFrame::setSelectedMarkerName(std::string name)
{
  selected_marker_name_ = name;
}

void WaypointFrame::poseChanged(double val)
{
  Eigen::Vector3f pos_eigen;
  Eigen::Vector4f quat_eigen;
  getPose(&pos_eigen, &quat_eigen);
  bool succ = wp_nav_tool_->setServerPose(std::stoi(selected_marker_name_.substr(8)), pos_eigen, quat_eigen);
}


void WaypointFrame::display_corridros(){
}


void WaypointFrame::ineqChanged(double val, int mode, int axes)
{
  int index = std::stoi(selected_marker_name_.substr(8))-1;
  if (index >= ineq_list.size())
    RCLCPP_ERROR(node->get_logger(),"%s not found in map", selected_marker_name_.c_str());
  else
  {
    if(mode ==0){
      ineq_list[index].lower[axes] = val;
    }
    else if(mode==1){
      ineq_list[index].upper[axes] = val;

    }
    else if(mode==2){
      if(val > 0.4){
        ineq_list[index].enable = true;
      }
      else{
        ineq_list[index].enable = false;
      }

    }
    else if(mode == 3){
        ineq_list[index].derivOrder = val;
    }
    else{
      return;
    }
    display_corridros();
  }
}


void WaypointFrame::frameChanged()
{

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->frame_line_edit->text();
  // Only take action if the frame has changed.
  if((new_frame != frame_id_)  && (new_frame != ""))
  {
    wp_nav_tool_->changeFrame(new_frame.toStdString());
  }
}

void WaypointFrame::topicChanged()
{
  QString new_topic = ui_->topic_line_edit->text();

  // Only take action if the name has changed.
  if(new_topic != output_topic_)
  {
    //wp_pub_->shutdown();
    output_topic_ = new_topic;

    if((output_topic_ != "") && (output_topic_ != "/"))
    {
       wp_pub_ = node->create_publisher<nav_msgs::msg::Path>(output_topic_.toStdString(), 1);
    }
  }
}

void WaypointFrame::setWpCount(int size)
{
  std::ostringstream stringStream;
  stringStream << "num wp: " << size;
  std::string st = stringStream.str();

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(st));
}

void WaypointFrame::setConfig(QString topic, QString frame, float height)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->topic_line_edit->blockSignals(true);
  ui_->frame_line_edit->blockSignals(true);
  ui_->wp_height_doubleSpinBox->blockSignals(true);

  ui_->topic_line_edit->setText(topic);
  ui_->frame_line_edit->setText(frame);
  ui_->wp_height_doubleSpinBox->setValue(height);

  ui_->topic_line_edit->blockSignals(false);
  ui_->frame_line_edit->blockSignals(false);
  ui_->wp_height_doubleSpinBox->blockSignals(false);

  }
  topicChanged();
  frameChanged();
  heightChanged(height);
}

void WaypointFrame::getPose(Eigen::Vector3f * position, Eigen::Vector4f * quat)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  (*position)(0) = ui_->x_doubleSpinBox->value();
  (*position)(1) = ui_->y_doubleSpinBox->value();
  (*position)(2) = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();
  //extract quaternion from yaw simple conversion look Hopf Firbation paper
  (*quat)(0) = 0.0;
  (*quat)(1) = 0.0;
  (*quat)(2) = sin(yaw/2);
  (*quat)(3) = cos(yaw/2);
  //tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
  //quat.x = qt.x();
  //quat.y = qt.y();
  //quat.z = qt.z();
  //quat.w = qt.w();

  }
}

//Quaternion is scalar last 
void WaypointFrame::setPose(Eigen::Vector3f  position, Eigen::Vector4f  quat)
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  //block spinbox signals
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position(0));
  ui_->y_doubleSpinBox->setValue(position(1));
  ui_->z_doubleSpinBox->setValue(position(2));
  double yaw;
  double q_x = quat(0);
  double q_y =quat(1);
  double q_z =quat(2);
  double q_w = quat(3);

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q_x * q_x;
  sqy = q_y *q_y;
  sqz = q_z * q_z;
  sqw = q_w * q_w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  // normalization added from urdfom_headers
  double sarg = -2 * (q_x * q_z - q_w * q_y) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    yaw = -2 * atan2(q_y, q_x);
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q_y, q_x);
  } else {
    yaw = atan2(2 * (q_x * q_y + q_w * q_z), sqw + sqx - sqy - sqz);
  }
  ui_->yaw_doubleSpinBox->setValue(yaw);
  //enable the signals
  ui_->x_doubleSpinBox->blockSignals(false);
  ui_->y_doubleSpinBox->blockSignals(false);
  ui_->z_doubleSpinBox->blockSignals(false);
  ui_->yaw_doubleSpinBox->blockSignals(false);

  }
}

void WaypointFrame::setWpLabel()
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::ostringstream stringStream;
  stringStream.precision(2);
  stringStream << selected_marker_name_;
  //stringStream << " x: " << position.x << " y: " << position.y << " z: " << position.z;
  std::string label = stringStream.str();

  ui_->sel_wp_label->setText(QString::fromStdString(label));
  }
}

double WaypointFrame::getDefaultHeight()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

double WaypointFrame::getTime()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return total_time_;
}

bool WaypointFrame::getTopicOveride()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return topicOverride;
}

bool WaypointFrame::getBernEnable()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return bern_enable_;
}


bool WaypointFrame::get2Ddisplay()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return display_2D;
}

QString WaypointFrame::getFrameId()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointFrame::getOutputTopic()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}

  //Clear Map
void WaypointFrame::clear_map(){
	auto client = node->create_client<std_srvs::srv::Empty>("/voxblox_node/clear_map");
	auto request = std::make_shared<std_srvs::srv::Empty::Request>();
	auto result = client->async_send_request(request);
}


  //Buttons RQT MAV MANAGER
void WaypointFrame::motors_on_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";

}

void WaypointFrame::motors_off_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	auto client = node->create_client<std_srvs::srv::SetBool>(srvs_name);

}

void WaypointFrame::hover_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/hover";

}

void WaypointFrame::land_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/land";	

}

void WaypointFrame::takeoff_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/takeoff";
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
}

void WaypointFrame::goto_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::string srvs_name;// = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	if(relative_){
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goToRelative";
	}
	else{
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	}

}

void WaypointFrame::relativeChanged(int b){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  relative_ = true;
  }
  else{
	 relative_ = false;
  }
}


void WaypointFrame::robotChanged(){ 
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->robot_name_line_edit->text();
  robot_name =  new_frame.toStdString();
}

void WaypointFrame::serviceChanged(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->node_name_line_edit->text();
  mav_node_name =  new_frame.toStdString();
}

void WaypointFrame::goHome_push_button(){
    boost::mutex::scoped_lock lock(frame_updates_mutex_);

  }
}
