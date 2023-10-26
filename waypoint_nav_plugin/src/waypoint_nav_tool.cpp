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

/* Author: Dinesh Thakur - Modified for waypoint navigation */

//#include <OGRE/OgreSceneNode.h>
//#include <OGRE/OgreSceneManager.h>
//#include <OGRE/OgreEntity.h>
#include <Ogre.h>

#include "waypoint_nav_tool.hpp"
#include <boost/bind.hpp>
namespace waypoint_nav_plugin
{

WaypointNavTool::WaypointNavTool()
  : moving_flag_node_(NULL)
  , frame_dock_(NULL)
  , frame_(NULL)
  , nh_(std::make_shared<rclcpp::Node>("w_tool"))
 // , server_("waypoint_nav", "", false)
  , unique_ind_(0)
  , current_flag_property_(NULL)
{
  shortcut_key_ = 'l';
}


WaypointNavTool::~WaypointNavTool()
{
  M_StringToSNPtr::iterator sn_it;
  for(sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
      scene_manager_->destroySceneNode(sn_it->second);
  }

  delete frame_;
  delete frame_dock_;
}

void WaypointNavTool::onInitialize()
{
  flag_resource_ = "package://waypoint_nav_plugin/media/flag.dae";

  if (!rviz_rendering::loadMeshFromResource(flag_resource_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("waypoint nav plugin"),
      "waypoint nav plugin: failed to load model resource '%s'.",
      flag_resource_.c_str());
    return;
  }
  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);

  moving_flag_node_->attachObject(entity);

  moving_flag_node_->setVisible(false);

  rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new WaypointFrame(context_, &sn_map_, server_, &unique_ind_, NULL, this);

  //if (window_context){
  //  std::cout << " ADD PANEL " <<std::endl;
  frame_dock_ = window_context->addPane("Waypoint Navigation", frame_);
  //}
  frame_->enable();

  //add Delete menu for interactive marker
  menu_handler_.insert("Delete", std::bind(&WaypointNavTool::processFeedback, this , std::placeholders::_1));
  menu_handler_.insert("Set Manual", std::bind(&WaypointNavTool::processFeedback, this,  std::placeholders::_1));
    server_ = new interactive_markers::InteractiveMarkerServer("waypoint", 
    nh_->get_node_base_interface(),
    nh_->get_node_clock_interface(),
    nh_->get_node_logging_interface(),
    nh_->get_node_topics_interface(),
    nh_->get_node_services_interface());
    thread_ = std::make_shared<std::thread>(std::bind(&WaypointNavTool::spin, this));
    frame_->server_ = server_;
    first_time_ = false;
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz_common::VectorProperty to show the user the position of the flag.
// Unlike rviz_common::Display, rviz_common::Tool is not a subclass of
// rviz_common::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void WaypointNavTool::activate()
{
  std::cout << " ACTIVATE START " <<std::endl;
  if(moving_flag_node_)
  {
    moving_flag_node_->setVisible(true);

    /*current_flag_property_ = new rviz_common::properties::VectorProperty(
      "waypoint" + QString::number(sn_map_.size()+1));
    getPropertyContainer()->addChild(current_flag_property_);
    server_->applyChanges();*/
  }
  server_->applyChanges();
}

//Spin the ROS NODE
 void WaypointNavTool::spin()
 {
    exec_.add_node(nh_);
    while(rclcpp::ok()){
       exec_.spin();
    }
 }

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of waypoints when
// we switch to another tool.
void WaypointNavTool::deactivate()
{
  if(moving_flag_node_)
  {
    moving_flag_node_->setVisible(false);
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }

}

// Handling mouse events

int WaypointNavTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  if(!moving_flag_node_)
  {
    return Render;
  }
  double height = frame_->getDefaultHeight();
  Ogre::Quaternion quat;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height);
  auto projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  auto projection = projection_finder->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  Ogre::Vector3 intersection = projection.second;
    if (projection.first) {

      moving_flag_node_->setVisible(true);
      moving_flag_node_->setPosition(intersection);

      frame_->setWpLabel(intersection);

      //check if mouse pointer is near existing waypoint
      M_StringToSNPtr::iterator sn_it;
      for(sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
      {
        Ogre::Vector3 stored_pos = sn_it->second->getPosition();
        double distance = std::sqrt(pow(stored_pos.x - intersection.x,2) + pow(stored_pos.y - intersection.y,2));

        if(distance < 0.4)
        {
          moving_flag_node_->setVisible(false);

          //delete the waypoint if right clicked
          if(event.rightDown())
          {
            sn_it->second->detachAllObjects();
            std::stringstream wp_name;
            wp_name << "waypoint" << sn_it->first;
            std::string wp_name_str(wp_name.str());
            server_->erase(wp_name_str);
            server_->applyChanges();
            sn_map_.erase(sn_it);

            moving_flag_node_->setVisible(true);
            return Render | Finished;
          }
        }
      }
    
    //add a waypoint
    if(event.leftDown())
    {
      makeIm(intersection, quat);
      return Render | Finished;
    }
    }
    else{
      moving_flag_node_->setVisible(false);
    }
  return Render;
}

void WaypointNavTool::makeIm(const Ogre::Vector3& position, const Ogre::Quaternion& quat)
{
    unique_ind_++; //increment the index for unique marker names

    std::stringstream wp_name;
    wp_name << "waypoint" << unique_ind_;
    std::string wp_name_str(wp_name.str());

    if (rviz_rendering::loadMeshFromResource(flag_resource_).isNull()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("waypoint nav plugin"),
        "waypoint nav plugin: failed to load model resource '%s'.",
        flag_resource_.c_str());
      return;
    }


    // create a new flag in the Ogre scene and save it in a std::map.
    Ogre::SceneNode* sn_ptr = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
    sn_ptr->attachObject(entity);
    sn_ptr->setVisible(true);
    sn_ptr->setPosition(position);
    sn_ptr->setOrientation(quat);

    M_StringToSNPtr::iterator sn_entry = sn_map_.find(unique_ind_);

    if (sn_entry == sn_map_.end())
      sn_map_.insert(std::make_pair(unique_ind_, sn_ptr));
    else{
    //  ROS_WARN("%s already in map", wp_name_str.c_str());
      return;
    }

    int num_wp = sn_map_.size();

    geometry_msgs::msg::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    frame_->setWpCount(num_wp);

    visualization_msgs::msg::InteractiveMarker int_marker;
    //int_marker.header.stamp = ros::Time::now();

    int_marker.header.frame_id = "world";
    //std::cout << " WE ARE IN FRAME ID " << int_marker.header.frame_id <<std::endl;
    int_marker.pose = pos.pose;
    int_marker.scale = 2;
    int_marker.name = wp_name_str;
    int_marker.description = wp_name_str;

    // create a cylinder marker
    visualization_msgs::msg::Marker cyn_marker;
    cyn_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    cyn_marker.scale.x = 2.0;
    cyn_marker.scale.y = 2.0;
    cyn_marker.scale.z = 0.2;
    cyn_marker.color.r = 0.5;
    cyn_marker.color.g = 0.5;
    cyn_marker.color.b = 0.5;
    cyn_marker.color.a = 0.5;

    // create a non-interactive control which contains the marker
    visualization_msgs::msg::InteractiveMarkerControl cyn_control;
    cyn_control.always_visible = true;
    cyn_control.markers.push_back(cyn_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(cyn_control);

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w =  0.707106781;
    control.orientation.x = 0;
    control.orientation.y =  0.707106781;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    control.name = "menu_delete";
    control.description = wp_name_str.substr(8);
    int_marker.controls.push_back(control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
    	std::bind(&WaypointNavTool::processFeedback, this, std::placeholders::_1));
    menu_handler_.apply(*server_, int_marker.name);

    //Set the current marker as selected
    Ogre::Vector3 p = position;
    Ogre::Quaternion q = quat;
    frame_->setSelectedMarkerName(wp_name_str);
    frame_->setWpLabel(p);
    frame_->setPose(p, q);
    frame_->push_newIneq_const();
    server_->applyChanges();
}

void WaypointNavTool::processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {

     M_StringToSNPtr::iterator sn_entry =
        sn_map_.find(std::stoi(feedback->marker_name.substr(8)));
      if (sn_entry == sn_map_.end())
        std::cout << feedback->marker_name.c_str() <<  "not found in map" <<std::endl;
       // ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
      else
      {

        if(feedback->menu_entry_id == 1)
        {
          //Delete selected waypoint
          std::stringstream wp_name;
          wp_name << "waypoint" << sn_entry->first;  
          
          std::string wp_name_str(wp_name.str());
          server_->erase(wp_name_str);

          menu_handler_.reApply(*server_);
          server_->applyChanges();
          sn_entry->second->detachAllObjects();
          sn_map_.erase(sn_entry);

		      int num_wp = sn_map_.size();
          frame_->setWpCount(num_wp);

        }
        else
        {
          //Set the pose manually from the line edits
          Ogre::Vector3 position;
          Ogre::Quaternion quat;

          frame_->getPose(position, quat);

          geometry_msgs::msg::Pose pos;
          pos.position.x = position.x;
          pos.position.y = position.y;
          pos.position.z = position.z;

          pos.orientation.x = quat.x;
          pos.orientation.y = quat.y;
          pos.orientation.z = quat.z;
          pos.orientation.w = quat.w;

          sn_entry->second->setPosition(position);
          sn_entry->second->setOrientation(quat);

          frame_->setWpLabel(position);

          server_->setPose(feedback->marker_name, pos);
          server_->applyChanges();
        }
      }
    }
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      M_StringToSNPtr::iterator sn_entry = sn_map_.find(std::stoi(feedback->marker_name.substr(8)));

      if (sn_entry == sn_map_.end())
        std::cout << feedback->marker_name.c_str() <<  "not found in map" <<std::endl;
        //ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
      else
      {
        geometry_msgs::msg::PoseStamped pos;
        pos.pose = feedback->pose;   

        Ogre::Vector3 position;
        position.x = pos.pose.position.x;
        position.y = pos.pose.position.y;
        position.z = pos.pose.position.z;

        sn_entry->second->setPosition(position);

        Ogre::Quaternion quat;
        quat.x = pos.pose.orientation.x;
        quat.y = pos.pose.orientation.y;
        quat.z = pos.pose.orientation.z;
        quat.w = pos.pose.orientation.w;

        sn_entry->second->setOrientation(quat);

        frame_->setWpLabel(position);
        frame_->setPose(position, quat);
        frame_->setSelectedMarkerName(feedback->marker_name);
        //waypoint_ineq_const ineq = frame_->ineq_list[std::stoi(feedback->marker_name.substr(8))-1];
        //frame_->setLimit(ineq.upper,ineq.lower,ineq.enable);
      }
    }
     break;
  }
}

void WaypointNavTool::getMarkerPoses()
{
  M_StringToSNPtr::iterator sn_it;
  for (sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_it->first;
    std::string wp_name_str(wp_name.str());
    server_->get(wp_name_str, int_marker);

    //ROS_ERROR("pos: %g %g %g", int_marker.pose.position.x,
  //  int_marker.pose.position.y,
    //int_marker.pose.position.z);
  }
}

void WaypointNavTool::clearAllWaypoints()
{
  M_StringToSNPtr::iterator sn_it;
  for (sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
      scene_manager_->destroySceneNode(sn_it->second);
  }
  sn_map_.clear();
  unique_ind_ = 0;
}

// Loading and saving the waypoints
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named waypoint positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz_common::ToolManager will know what to instantiate when the config
// file is read back in.
void WaypointNavTool::save(rviz_common::Config config) const
{
  config.mapSetValue("Class", getClassId());
  //rviz_common::Config waypoints_config = config.mapMakeChild("Waypoints");

  rviz_common::Config waypoints_config = config.mapMakeChild("WaypointsTool");

  waypoints_config.mapSetValue("topic", frame_->getOutputTopic());
  waypoints_config.mapSetValue("frame_id", frame_->getFrameId());
  waypoints_config.mapSetValue("default_height", frame_->getDefaultHeight());
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void WaypointNavTool::load(const rviz_common::Config& config)
{
  rviz_common::Config waypoints_config = config.mapGetChild("WaypointsTool");

  QString topic, frame;
  float height;
  if(!waypoints_config.mapGetString("topic", &topic))
    topic = "/waypoints";

  if(!waypoints_config.mapGetString("frame_id", &frame))
    frame = "world";

  waypoints_config.mapGetFloat("default_height", &height);

  frame_->setConfig(topic, frame, height);
}

} // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavTool,rviz_common::Tool)

