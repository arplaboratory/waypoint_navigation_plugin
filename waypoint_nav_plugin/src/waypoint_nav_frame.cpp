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

#include <OGRE/OgreSceneManager.h>
//#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.hpp>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>

#include "waypoint_nav_tool.hpp"
//#include "waypoint_nav_frame.h"

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
  , frame_id_("/map")
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
/*
  QString filename = QFileDialog::getSaveFileName(0,tr("Save Bag"), "waypoints", tr("Bag Files (*)"));
   if(filename == "")
   std::cout << " NO FILE NAME GIVEN!!!" <<std::endl;
   else
  {
    QFileInfo info(filename);
    std::string filn = info.absolutePath().toStdString() + "/" + info.baseName().toStdString();
    std::cout << "saving waypoints to " << filn.c_str() <<std::endl;
    //ROS_INFO("saving waypoints to %s", filn.c_str());
    std::unique_ptr<rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(filn);
    writer_->create_topic(
      {"waypoints",
      "nav_msgs/msg/Path",
      rmw_get_serialization_format(),
      ""});
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
    
    writer_->write(path, "waypoints", node->now());
  }*/
}

void WaypointFrame::loadButtonClicked()
{/*
  QString filename = QFileDialog::getExistingDirectory(0,"/home");
  if(filename == "")
   std::cout << " NO FILE NAME GIVEN!!!" <<std::endl;
  else
  {
    //Clear existing waypoints
    clearAllWaypoints();
    std::string filn = filename.toStdString();
    std::cout << " loading waypoints from " << filn <<std::endl;
    rosbag2_cpp::Reader reader_;
    reader_.open(filn);
    while (reader_.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
      if (msg->topic_name == "waypoints") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        nav_msgs::msg::Path ros_msg;
        serialization_.deserialize_message(&serialized_msg, &ros_msg);
        std::cout <<" ROS MESSAGE " << ros_msg.poses.size() <<std::endl;
        for(int i = 0; i < ros_msg.poses.size(); i++)
        {

          geometry_msgs::msg::PoseStamped pos = ros_msg.poses[i];
          Ogre::Vector3 position;
          position.x = pos.pose.position.x;
          position.y = pos.pose.position.y;
          position.z = pos.pose.position.z;

          Ogre::Quaternion quat;
          quat.x = pos.pose.orientation.x;
          quat.y = pos.pose.orientation.y;
          quat.z = pos.pose.orientation.z;
          quat.w = pos.pose.orientation.w;

          wp_nav_tool_->makeIm(position, quat);
        }
      }
    }
  }*/
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
  nav_msgs::msg::Path path;
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  if(!getTopicOveride()){
    std::string topic_name = "/waypoints";
    /*if(getBernEnable()){
      topic_name = "/b_waypoints";
    }*/
    wp_pub_ = node->create_publisher<nav_msgs::msg::Path>("/"+ robot_name +topic_name, 1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  
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
  //nh_.setParam("/"+ robot_name+"/"+"bern_enable",getBernEnable());
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
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  while (sn_map_ptr_->size()>0)
  {
    sn_it = sn_map_ptr_->begin();
    sn_it->second->detachAllObjects();
    std::stringstream wp_name;
    wp_name << "waypoint" << sn_it->first;
    std::string wp_name_str(wp_name.str());
    server_->erase(wp_name_str);
    server_->applyChanges();
    sn_map_ptr_->erase(sn_it);
  }

  //clear the waypoint map and reset index
  //sn_map_ptr_->clear();
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

  std::map<int, Ogre::SceneNode *>::iterator sn_entry =
      sn_map_ptr_->find(std::stoi(selected_marker_name_.substr(8)));

  if (sn_entry == sn_map_ptr_->end())
    RCLCPP_ERROR(node->get_logger(),"%s not found in map", selected_marker_name_.c_str());
    //ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);
    geometry_msgs::msg::Pose pos;
    pos.position.x = position.x;
    pos.position.y = position.y;
    pos.position.z = position.z;

    pos.orientation.x = quat.x;
    pos.orientation.y = quat.y;
    pos.orientation.z = quat.z;
    pos.orientation.w = quat.w;

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_entry->first;
    std::string wp_name_str(wp_name.str());
    std::cout <<" WAYPOINT " <<std::endl;
    visualization_msgs::msg::InteractiveMarker int_marker;
    std::cout <<" get start " <<std::endl;
    server_->setPose(wp_name_str, pos);
    std::cout <<" set pose " <<std::endl;

    /*if(!server_->get(wp_name_str, int_marker))
    {

      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }*/
    std::cout << " SERVER Set" <<std::endl;
    server_->applyChanges();
  }
}


void WaypointFrame::display_corridros(){
 /* marker_array.markers.clear();
  for(int i =0;i<ineq_list.size()-1;i++){
      visualization_msgs::Marker marker;
			marker.ns = "basic_shapes";
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

void WaypointFrame::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  position.x = ui_->x_doubleSpinBox->value();
  position.y = ui_->y_doubleSpinBox->value();
  position.z = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();
  //extract quaternion from yaw simple conversion look Hopf Firbation paper
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = sin(yaw/2);
  quat.w = cos(yaw/2);
  //tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
  //quat.x = qt.x();
  //quat.y = qt.y();
  //quat.z = qt.z();
  //quat.w = qt.w();

  }
}

void WaypointFrame::setPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  //block spinbox signals
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position.x);
  ui_->y_doubleSpinBox->setValue(position.y);
  ui_->z_doubleSpinBox->setValue(position.z);
  //extrat yaw from quaterinon

  double  yaw = atan2(2.0*(quat.y*quat.z + quat.w*quat.x), 
  quat.w*quat.w - quat.x*quat.x - quat.y*quat.y + quat.z*quat.z);
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

void WaypointFrame::setWpLabel(Ogre::Vector3 position)
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
	auto client = node->create_client<std_srvs::srv::SetBool>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = true;
	auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Sent Service");

}

void WaypointFrame::motors_off_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	auto client = node->create_client<std_srvs::srv::SetBool>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = false;
	auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Sent Service");

}

void WaypointFrame::hover_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/hover";
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Sent Service");

}

void WaypointFrame::land_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/land";	
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Sent Service");

}

void WaypointFrame::takeoff_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/takeoff";
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<std_srvs::srv::Trigger>(srvs_name);
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto result = client->async_send_request(request);
      RCLCPP_INFO(node->get_logger(), "Sent Service");

}

void WaypointFrame::goto_push_button(){
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
	/*ros::NodeHandle nh;
	std::string srvs_name;
	if(relative_){
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goToRelative";
	}
	else{
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	}
	ros::ServiceClient client = nh.serviceClient<mav_manager::Vec4>(srvs_name);
	mav_manager::Vec4 srv;
  	srv.request.goal [0] = ui_->x_doubleSpinBox_gt->value();
 	srv.request.goal [1] = ui_->y_doubleSpinBox_gt->value();
  	srv.request.goal [2] = ui_->z_doubleSpinBox_gt->value();
  	srv.request.goal [3] = ui_->yaw_doubleSpinBox_gt->value();
	if (client.call(srv))
	{
		ROS_INFO("GoTo Success");
	}
	else
	{	
		ROS_ERROR("Failed GoTo ");
	}		*/

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
	/*ros::NodeHandle nh;
	std::string srvs_name;
	srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	ros::ServiceClient client = nh.serviceClient<mav_manager::Vec4>(srvs_name);
	mav_manager::Vec4 srv;
  	srv.request.goal [0] = 0;
 	srv.request.goal [1] = 0;
  	srv.request.goal [2] = 0.5;
  	srv.request.goal [3] = 0;
	if (client.call(srv))
	{
		ROS_INFO("Go Home Success");
	}
	else
	{	
		ROS_ERROR("Failed Go Home ");
	}		
  }*/

}
}
