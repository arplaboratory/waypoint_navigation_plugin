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
#include "waypoint_nav_tool_ros2.hpp"
//#include "waypoint_nav_frame.h"
#include <mav_manager_srv/srv/vec4.hpp>
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
  , quat_transform_(Eigen::Quaternionf::Identity())
{
  //scene_manager_ = context_->getSceneManager();

  // set up the GUI 
  node = rclcpp::Node::make_shared("wp_node");
  ui_->setupUi(this);
  pub_corridor_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("corridors", 1); 
  wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("waypoints", 1);
  path_clear_pub_ = node->create_publisher<std_msgs::msg::Bool>("/clear", 1);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // quat_transform_ = Eigen::Quaternionf::Identity();

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
  connect(ui_->local_frame, SIGNAL(stateChanged(int)), this, SLOT(originChanged(int)));
  connect(ui_->kill_switch, SIGNAL(clicked()), this, SLOT(killSwitchEngage()));
  connect(ui_->hover, SIGNAL(clicked()), this, SLOT(hover_push_button()));
  connect(ui_->bern_enable, SIGNAL(stateChanged(int)), this, SLOT(bern_enable(int)));
  connect(ui_->replan_enable, SIGNAL(stateChanged(int)), this, SLOT(replan_enable(int)));
  connect(ui_->topic_overide, SIGNAL(stateChanged(int)), this, SLOT(topic_enable(int)));
  connect(ui_->clear_path, SIGNAL(clicked()), this, SLOT(clear_path()));
  
  connect(ui_->reset_map, SIGNAL(clicked()), this, SLOT(clear_map()));
  
  robot_name = std::getenv("MAV_NAME") ? std::string(std::getenv("MAV_NAME")) : "quadrotor";
  ui_->robot_name_line_edit->setText(QString::fromStdString(robot_name));

  node->declare_parameter("/"+ robot_name+"/"+"replan",false);
  node->declare_parameter("/"+ robot_name+"/"+"bern_enable",false);
  kill_publisher_ = node->create_publisher<px4_msgs::msg::VehicleCommand>("/" + robot_name + "/fmu/in/vehicle_command", 1);
	//path_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_pos", 10, &WaypointFrame::pos_listen, this);
	//vel_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_vel", 10, &WaypointFrame::vel_listen, this);
	//acc_listen_ = nh_.subscribe("/quadrotor/trackers_manager/qp_tracker/qp_trajectory_acc", 10, &WaypointFrame::acc_listen, this);

  // Call tf2_callback() function every second
  timer_ = node->create_wall_timer(std::chrono::seconds(1), std::bind(&WaypointFrame::tf2_callback, this));

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
  QFileInfo info(filename);
  std::string filn = info.absolutePath().toStdString() + "/" + info.baseName().toStdString() + ".txt";
  if(filename == "")
   std::cout << " NO FILE NAME GIVEN!!!" <<std::endl;
  else
  {
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
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  //block spinbox signals
  /*
  ui_->bern_pl->blockSignals(true);
  ui_->bern_xl->blockSignals(true);
  ui_->bern_yl->blockSignals(true);
  ui_->bern_zl->blockSignals(true);
  ui_->bern_pu->blockSignals(true);
  ui_->bern_xu->blockSignals(true);
  ui_->bern_yu->blockSignals(true);
  ui_->bern_zu->blockSignals(true);
  ui_->ineq_enable->blockSignals(true);


#include <chrono>
#include <thread>
  ui_->bern_xl->setValue(lower[0]);
  ui_->bern_yl->setValue(lower[1]);
  ui_->bern_zl->setValue(lower[2]);
  ui_->bern_pl->setValue(lower[3]);
  ui_->bern_xu->setValue(upper[0]);
  ui_->bern_yu->setValue(upper[1]);
  ui_->bern_zu->setValue(upper[2]);
  ui_->bern_pu->setValue(upper[3]);
  ui_->ineq_enable->setValue(enable);


  //enable the signals
  ui_->bern_pl->blockSignals(false);
  ui_->bern_xl->blockSignals(false);
  ui_->bern_yl->blockSignals(false);
  ui_->bern_zl->blockSignals(false);
  ui_->bern_pu->blockSignals(false);
  ui_->bern_xu->blockSignals(false);
  ui_->bern_yu->blockSignals(false);
  ui_->bern_zu->blockSignals(false);
  ui_->ineq_enable->blockSignals(false);*/

}


void WaypointFrame::publishButtonClicked()
{
  if(!getTopicOveride()){
    std::string topic_name = "/waypoints";
    if(getBernEnable()){
      topic_name = "/b_waypoints";
    }
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
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
  {
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();

    geometry_msgs::msg::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    path.poses.push_back(pos);
  }

  path.header.frame_id = frame_id_.toStdString();
  //nh_.setParam("/total_time", getTime());
  //nh_.setParam("/display_2D", get2Ddisplay());
  if(!getTopicOveride()){
    std::string topic_name = "/perch";
    if(getBernEnable()){
      topic_name = "/b_perch";
    }
    wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+ robot_name +topic_name, 1);
  }
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

  // if (local_frame_){

  //   rclcpp::get_logger("ROTATING FLAG");
  //   Eigen::Quaternionf quat(quat_eigen(3), quat_eigen(0),quat_eigen(1), quat_eigen(2));
  //   pos_eigen = quat_transform_ * pos_eigen;
  //   quat = quat_transform_ * quat;
  //   quat_eigen = {quat.x(), quat.y(), quat.z(), quat.w()};

  // }
  bool succ = wp_nav_tool_->setServerPose(std::stoi(selected_marker_name_.substr(8)), pos_eigen, quat_eigen);
}


void WaypointFrame::display_corridros(){
 /* marker_array.markers.clear();
  for(int i =0;i<ineq_list.size()-1;i++){
      visualization_msgs::Marker marker;
			marker.ns = "basic_shapes";Vec4
			marker.id = i;
			marker.type = 1; //CUBE
			// Set the marker action.  Options are ADD and DELETE
			marker.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(marker);
  }
  pub_corridor_.publish(marker_array);
  marker_array.markers.clear();
  for(int i =0;i<ineq_list.size()-1;i++){
    if(ineq_list[i].enable==1){
      visualization_msgs::Marker marker;
      Eigen::Vector4d l= ineq_list[i].lower;
      Eigen::Vector4d u= ineq_list[i].upper;
      marker.header.frame_id=frame_id_.toStdString();
      marker.pose.position.x = 0.5*(l[0]+u[0]);
      marker.pose.position.y = 0.5*(l[1]+u[1]);
      marker.pose.position.z = 0.5*(l[2]+u[2]);

      marker.pose.orientation.w = 1.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;

      marker.scale.x = abs(u[0]-l[0]);
      marker.scale.y = abs(u[1]-l[1]);
      marker.scale.z = abs(u[2]-l[2]);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.2;

			marker.ns = "basic_shapes";
			marker.id = i;
			// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			marker.type = 1; //CUBE
			// Set the marker action.  Options are ADD and DELETE
			marker.action = visualization_msgs::Marker::ADD;
      marker_array.markers.push_back(marker);
    }
  }
  pub_corridor_.publish(marker_array);*/
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
    frame_id_ = new_frame;
    //ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

    // update the frames for all interactive markers
    std::map<int, Ogre::SceneNode *>::iterator sn_it;
    for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    {
      std::stringstream wp_name;
      wp_name << "waypoint" << sn_it->first;
      std::string wp_name_str(wp_name.str());

      visualization_msgs::msg::InteractiveMarker int_marker;
      if(server_->get(wp_name_str, int_marker))
      {
        int_marker.header.frame_id = new_frame.toStdString();
        server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
        // Eigen::Vector3f pos(int_marker.pose.x,int_marker.pose.y,int_marker.pose.z);
        // Eigen::Quaternionf q(int_marker.header.w, int_marker.header.x, int_marker.header.y, int_marker.header.z)
        // bool succ = wp_nav_tool_->setServerPose(std::stoi(wp_name_str.substr(8)), pos, q);
      }
    }
    server_->applyChanges();
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

/*
void WaypointFrame::display(const nav_msgs::msg::Path &msg, int order){
	double dt = 0.01;
	//Record File 
	std::ofstream outFileX;
	std::ofstream outFileY;
	std::ofstream outFileZ;
	std::string title = Deriv_title[order];
	std::string der_app = append[order];
	outFileX.open("tempX"+der_app +".dat");
	outFileY.open("tempY"+der_app +".dat");
	outFileZ.open("tempZ"+der_app +".dat");
	for (int j=0; j< msg.poses.size(); j++){
    geometry_msgs::PoseStamped ps = msg.poses[j];	
    double time = ps.header.stamp.toSec();	
    outFileX << time;
		outFileX << " " << ps.pose.position.x << std::endl;
		outFileY << time;
		outFileY << " " <<ps.pose.position.y << std::endl;
		outFileZ << time;
		outFileZ << " " << ps.pose.position.z << std::endl;
	}
	outFileX.close();
	outFileY.close();
	outFileZ.close();
	// VISUALIZATION
  GnuplotPipe gp;
	gp.sendLine("set title " + title);
  gp.sendLine("plot 'tempX"+ der_app +".dat' , 'tempY"+ der_app +".dat', 'tempZ"+ der_app +".dat' ");
}
*/

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
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "Success callback Clear map");
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "Failed callback Clear map");  
  }
}


  //Buttons RQT MAV MANAGER
void WaypointFrame::motors_on_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	auto client = node->create_client<std_srvs::srv::SetBool>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = true;
	auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
  }

}

void WaypointFrame::motors_off_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	auto client = node->create_client<std_srvs::srv::SetBool>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = false;
	auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
  }
}

void WaypointFrame::hover_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/hover";
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
  }

}

void WaypointFrame::land_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/land";	
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
  }
}

void WaypointFrame::takeoff_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/takeoff";
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
  }
}

void WaypointFrame::tf2_callback(){

  RCLCPP_INFO(node->get_logger(), "in tf2_callback");
  if (local_frame_){
    RCLCPP_INFO(node->get_logger(), "GOTO in Local Frame");
    geometry_msgs::msg::TransformStamped t;
    
    try {
          t = tf_buffer_->lookupTransform(
            "FLU", "world" ,                       
            tf2::TimePointZero);
              RCLCPP_INFO(node->get_logger(), "FOUND TRANSFORM");

        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            node->get_logger(), "Could not transform %s to %s: %s",
            "world", "FLU", ex.what());
          return;
        }
  // tf_buffer_->clear();
  
  quat_transform_ = Eigen::Quaternionf(t.transform.rotation.w,
                                      t.transform.rotation.x,
                                      t.transform.rotation.y,
                                      t.transform.rotation.z);
  quat_transform_ = quat_transform_.inverse();
  
  RCLCPP_INFO(node->get_logger(), "Set TRANSFORM");
  yaw_init_ = std::atan2(2. * (quat_transform_.w() * quat_transform_.z() + quat_transform_.x() * quat_transform_.y()), 1. - 2. * (quat_transform_.y() * quat_transform_.y() + quat_transform_.z() * quat_transform_.z()));

  }
}

void WaypointFrame::goto_push_button(){
  
  Eigen::Vector3f goalPos(ui_->x_doubleSpinBox_gt->value(),
                          ui_->y_doubleSpinBox_gt->value(),
                          ui_->z_doubleSpinBox_gt->value());
  
  goalPos = quat_transform_ * goalPos;
  double yaw = ui_->yaw_doubleSpinBox_gt->value();
  
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::string srvs_name;// = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	if(relative_){
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goToRelative";
	}
	else{
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
    yaw-=yaw_init_;
	}
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<mav_manager_srv::srv::Vec4>(srvs_name);
	auto request = std::make_shared<mav_manager_srv::srv::Vec4::Request>();
  request->goal[0] = goalPos.x();
 	request->goal[1] = goalPos.y();
  request->goal[2] = goalPos.z();
  request->goal[3]  = yaw;

	auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent Service");
  if(rclcpp::spin_until_future_complete(node->get_node_base_interface(), result,
   std::chrono::duration</*TimeRepT*/int64_t, /*TimeT*/ std::milli>(300))==
  rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(node->get_logger(), "%s Success callback", srvs_name.c_str());
  }
  else{    
    RCLCPP_ERROR(node->get_logger(), "%s Failed callback", srvs_name.c_str());  
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

void WaypointFrame::originChanged(int b){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  local_frame_ = true;
    std::cout << " LOCAL FRAME is " << local_frame_ <<std::endl;
  }
  else{
	 local_frame_ = false;
    std::cout << " LOCAL FRAME is " << local_frame_ <<std::endl;

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

void WaypointFrame::killSwitchEngage(){
  auto message = px4_msgs::msg::VehicleCommand();
  message.command = 185;  // MAV_CMD_COMPONENT_ARM_DISARM
  message.param1 = 1.0;   // Engage kill switch
  message.target_system = 1;
  message.target_component = 1;
  message.source_system = 1;
  message.source_component = 1;
  message.from_external = true;
  
  kill_publisher_->publish(message);
  RCLCPP_WARN(node->get_logger(), "Kill Switch Engaged");
}

Eigen::Quaternionf WaypointFrame::getQuatTransform(){
  return (quat_transform_);
}

bool WaypointFrame::getLocalFrameStatus(){
  return (local_frame_);
}
}
