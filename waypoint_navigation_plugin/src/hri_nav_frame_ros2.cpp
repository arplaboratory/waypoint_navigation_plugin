/**********************************************************************
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
#include "hri_nav_tool_ros2.hpp"
//#include "waypoint_nav_frame.h"
#include <mav_manager_srv/srv/vec4.hpp>
#include <OGRE/OgreVector3.h>
#include <QFileDialog>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace hri_nav_plugin
{
    HriFrame::HriFrame(rviz_common::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, 
interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent, HriNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::HRI_coworkerWidget())
  , sn_map_ptr_(map_ptr)
{
   // scene_manager_ = context_->getSceneManager();

    // set up the GUI 
    node = rclcpp::Node::make_shared("wp_node");
    ui_->setupUi(this);

    path_clear_pub_ = node->create_publisher<std_msgs::msg::Bool>("/clear", 1);
    rqt_change_mode = node->create_publisher<std_msgs::msg::Int32>("/rqt_input/case_switcher", 1);
    rqt_input_start_FPVI_task = node->create_publisher<std_msgs::msg::Bool>("/rqt_input/start_FPVI_task", 1);
    rqt_input_start_APVI_task = node->create_publisher<std_msgs::msg::Bool>("/rqt_input/start_APVI_task", 1);
    rqt_input_rotate_180_yaw = node->create_publisher<std_msgs::msg::Bool>("/rqt_input/rotate_180_yaw", 1);


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
  connect(ui_->clear_path, SIGNAL(clicked()), this, SLOT(clear_path()));
  connect(ui_->reset_map, SIGNAL(clicked()), this, SLOT(clear_map()));
  connect(ui_->take_control_button, SIGNAL(clicked()), this, SLOT(rqt_change_mode_()));
  connect(ui_->start_FPVI_task_button, SIGNAL(clicked()), this, SLOT(start_FPVI_task()));
  connect(ui_->start_APVI_task_button, SIGNAL(clicked()), this, SLOT(start_APVI_task()));
  connect(ui_->exit_task_button, SIGNAL(clicked()), this, SLOT(exit_task()));
  connect(ui_->rotate180, SIGNAL(clicked()), this, SLOT(rotate_180_yaw()));


}

HriFrame::~HriFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}




void HriFrame::enable()
{
  // activate the frame
  show();
}

void HriFrame::disable()
{

  hide();
}

void HriFrame::clear_path()
{
   std_msgs::msg::Bool thing;
   path_clear_pub_->publish(thing);
}

void HriFrame::rqt_change_mode_(){
    int mode = 1;
	std_msgs::msg::Int32 mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_change_mode->publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::start_FPVI_task(){
    bool mode = true;
	std_msgs::msg::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_start_FPVI_task->publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::start_APVI_task(){
    bool mode = true;
	std_msgs::msg::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_start_APVI_task->publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::exit_task(){
    int mode = 2;
	std_msgs::msg::Int32 mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_change_mode->publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::rotate_180_yaw(){
    bool mode = true;
	std_msgs::msg::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_rotate_180_yaw->publish(mode_);
        sleep(0.01);
    }
}


 //Buttons RQT MAV MANAGER
void HriFrame::motors_on_push_button(){
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

void HriFrame::motors_off_push_button(){
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

void HriFrame::hover_push_button(){
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

void HriFrame::land_push_button(){
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

void HriFrame::takeoff_push_button(){
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

void HriFrame::goto_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::string srvs_name;// = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	if(relative_){
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goToRelative";
	}
	else{
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	}
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<mav_manager_srv::srv::Vec4>(srvs_name);
	auto request = std::make_shared<mav_manager_srv::srv::Vec4::Request>();
  request->goal[0]  = ui_->x_doubleSpinBox_gt->value();
 	request->goal[1] = ui_->y_doubleSpinBox_gt->value();
  request->goal[2] = ui_->z_doubleSpinBox_gt->value();
  request->goal[3]  = ui_->yaw_doubleSpinBox_gt->value();

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

void HriFrame::relativeChanged(int b){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  relative_ = true;
  }
  else{
	 relative_ = false;
  }
}

void HriFrame::robotChanged(){ 
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->robot_name_line_edit->text();
  robot_name =  new_frame.toStdString();
}

void HriFrame::serviceChanged(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->node_name_line_edit->text();
  mav_node_name =  new_frame.toStdString();
}

void HriFrame::goHome_push_button(){
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::string srvs_name;// = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	//ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	auto client = node->create_client<mav_manager_srv::srv::Vec4>(srvs_name);
	auto request = std::make_shared<mav_manager_srv::srv::Vec4::Request>();
  request->goal[0]  = 0.0;
 	request->goal[1] = 0.0;
  request->goal[2] = 0.5;
  request->goal[3]  = 0.0;
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





  //Clear Map
void HriFrame::clear_map(){
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
}


