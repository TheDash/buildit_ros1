
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <buildit_ros/UpdateInteractiveMountPoint.h>
#include <buildit_ros/InteractiveMountPoint.h>
#include <buildit_ros/SetOrientation.h>
#include <buildit_ros/SetPosition.h>

#include <vector>
#include <algorithm>
#include <sstream>
#include <map>
#include <iostream>
#include <string>
#include <sstream>

#include <buildit_ros/GetInteractiveMarkers.h>
#include <buildit_ros/MountPointMarker.h>
#include <buildit_ros/buildit_config.h>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// GLOBAL VARS
#define ATTACH_MENU_ID 1
#define UNATTACH_MENU_ID 2
#define SET_POSITION_MENU_ID 3
#define SET_ORIENTATION_MENU_ID 4


// NAMESPACING
using namespace visualization_msgs;


// FORWARD DECLARATIONS
static void alignMarker(const InteractiveMarkerFeedbackConstPtr&);

static void processFeedback( const InteractiveMarkerFeedbackConstPtr&);

void makeChessPieceMarker(const tf::Vector3& );
InteractiveMarkerControl& makeBoxControl(InteractiveMarker&);

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );

void make6DofMarkerWithName(std::string& name, std::string& parent_name, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

interactive_markers::MenuHandler menu_handler;

visualization_msgs::Marker makeBox(InteractiveMarker&);  

void attach_marker_to_model(const InteractiveMarkerFeedbackConstPtr & feedback);

void unattach_marker_to_model(const InteractiveMarkerFeedbackConstPtr & feedback);

void set_marker_orientation(const InteractiveMarkerFeedbackConstPtr & feedback);

void set_marker_position(const InteractiveMarkerFeedbackConstPtr & feedback);

bool get_all_markers(buildit_ros::GetInteractiveMarkers::Request &req, buildit_ros::GetInteractiveMarkers::Response &res);

bool update_mount_point_marker(buildit_ros::UpdateInteractiveMountPoint::Request &req, buildit_ros::UpdateInteractiveMountPoint::Response &res);

bool clear_all_markers(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res);


bool load_mount_point_marker(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res);

std::map<std::string, geometry_msgs::Vector3> parent_positions;
std::map<std::string, MountPointMarker> mount_point_markers;
std::vector<std::string> marker_names;

int MountPointMarker::number_of_markers = 0;


// BEGIN FUNCTION DEFINITIONS 
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

Marker makeBox(InteractiveMarker &msg )
{
	Marker marker;
	marker.type = Marker::CUBE;
	marker.scale.x = msg.scale * 0.1;
	marker.scale.y = msg.scale * 0.1;
	marker.scale.z = msg.scale * 0.1;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;
	return marker;
}


//Taken from RViz tutorials 
void makeChessPieceMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}

void attach_marker_to_model(const InteractiveMarkerFeedbackConstPtr & feedback)
{


}

void unattach_marker_to_model(const InteractiveMarkerFeedbackConstPtr & feedback)
{


}

void set_marker_orientation(const InteractiveMarkerFeedbackConstPtr & feedback)
{
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<buildit_ros::SetOrientation>("set_marker_orientation_editor");

   buildit_ros::SetOrientation or_msg;
   or_msg.request.marker_info = *feedback.get();
   if (client.call(or_msg))
   {
     ROS_INFO("Server contacted, spawning editor.");
   } else
   {
      ROS_ERROR("Unable to contact service set_marker_orientation_editor");
   }
   
}

void set_marker_position(const InteractiveMarkerFeedbackConstPtr & feedback)
{
     ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<buildit_ros::SetPosition>("set_marker_position_editor");

   buildit_ros::SetPosition pos_msg;
   pos_msg.request.marker_info = *feedback.get();
   if (client.call(pos_msg))
   {
     ROS_INFO("Server contacted, spawning editor.");
   } else
   {
      ROS_ERROR("Unable to contact service set_marker_position_editor");
   }

}

void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );

      switch (feedback->menu_entry_id)
      {
          case SET_POSITION_MENU_ID:
              ROS_INFO("Setting Position. ");
              set_marker_position(feedback);
              break;
          case SET_ORIENTATION_MENU_ID:
              ROS_INFO("Setting Orientation.");
              set_marker_orientation(feedback);
              break;
          case ATTACH_MENU_ID:
              ROS_INFO("Attaching mount point. ");
              attach_marker_to_model(feedback);
              break;
          case UNATTACH_MENU_ID:
              ROS_INFO("Unattaching mount point.");
              unattach_marker_to_model(feedback);
              break;
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

      // Update pose DB
      mount_point_markers[feedback->marker_name].pose = feedback->pose;
 
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}


void alignMarker( const InteractiveMarkerFeedbackConstPtr &feedback )
{
	geometry_msgs::Pose pose = feedback->pose;
	pose.position.x = round(pose.position.x-0.5)+0.5;
	pose.position.y = round(pose.position.y-0.5)+0.5;
	ROS_INFO_STREAM( feedback->marker_name << ":"
	<< " aligning position = "
	<< feedback->pose.position.x
	<< ", " << feedback->pose.position.y
	<< ", " << feedback->pose.position.z
	<< " to "
	<< pose.position.x
	<< ", " << pose.position.y
	<< ", " << pose.position.z );

	server->setPose( feedback->marker_name, pose );
	server->applyChanges();
}


void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}

void make6DofMarkerWithName(std::string& name, std::string& parent_name, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{

   InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = name;
  int_marker.description = name + "mount point";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      //int_marker.name += "_" + mode_text;
      int_marker.description = name + " mount point";
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );

}

std::vector<std::string> marker_list;

bool get_all_markers(buildit_ros::GetInteractiveMarkers::Request &req, buildit_ros::GetInteractiveMarkers::Response &res)
{

   // Iterate the markers map
     typedef std::map<std::string, MountPointMarker>::iterator it_type;
     for (it_type iterator = mount_point_markers.begin(); iterator != mount_point_markers.end(); ++iterator)
     { 
          buildit_ros::MountPointMarker m;
          MountPointMarker marker;
          marker = iterator->second;
          ROS_INFO("Sending marker %s", marker.marker_name.c_str());
          m.link_name = marker.link_name;
          m.marker_name = marker.marker_name;
          m.pose = marker.pose;
          //buildit_ros::MountPointMarker m = iterator->second();
          res.markers.push_back(m);
     }   
     return true;
}


bool update_mount_point_marker(buildit_ros::UpdateInteractiveMountPoint::Request &req, buildit_ros::UpdateInteractiveMountPoint::Response &res)
{
   // ok ... so the new position needs to be relative to the parent position. so new_pose is specified relative to the parent. How to pass in that vector to set pose? Well, i'd need to specify it as a link on the model. 
   geometry_msgs::Vector3 parent_pos = parent_positions[req.marker_name];
   tf::Vector3 parent(parent_pos.x, parent_pos.y, parent_pos.z);
   tf::Vector3 curr(req.new_pose.position.x, req.new_pose.position.y, req.new_pose.position.z);
   tf::Vector3 new_pos = parent + curr;

   req.new_pose.position.x = new_pos.getX();
   req.new_pose.position.y = new_pos.getY();
   req.new_pose.position.z = new_pos.getZ();

   // Update the marker in the DB of markers.
   MountPointMarker mpm = mount_point_markers[req.marker_name];
   mpm.pose = req.new_pose;

   server->setPose(req.marker_name, req.new_pose);
   server->applyChanges();
   ROS_INFO("Successfully updated marker %s", req.marker_name.c_str());

   return true;
}

bool clear_all_markers(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{
    marker_names.clear();
    MountPointMarker::number_of_markers = 0;
    mount_point_markers.clear();
    server->clear();
    server->applyChanges();
    return true;
}

bool load_mount_point_marker(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{

   MountPointMarker marker;
   marker.link_name = req.link_name;
   marker.marker_name = req.link_name;
   ROS_INFO("Loading marker %s from YAML file", marker.marker_name.c_str());
   marker.number_of_markers++;

   geometry_msgs::Point p;
   p.x = req.parent_position.x;
   p.y = req.parent_position.y;
   p.z = req.parent_position.z;

   geometry_msgs::Pose pose; 
   pose.position = p;

   marker.pose = pose;
   marker.marker_id = 0;

   marker_names.push_back(marker.link_name);

    // Create 6dof marker with that link name. 
   tf::Vector3 position = tf::Vector3( marker.pose.position.x, marker.pose.position.y , marker.pose.position.z);
 
   ROS_INFO("Inserted marker %s", marker.marker_name.c_str());
   mount_point_markers.insert(std::pair<std::string, MountPointMarker>(marker.marker_name, marker));
   make6DofMarkerWithName( marker.marker_name, marker.link_name, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
   res.spawned = true;
   server->applyChanges();

   return true;
}

std::map<std::string, int> marker_counts;

// The server will have to spawn markers at the locations told, and be passed messages. 
bool spawn_mount_point_marker(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{
   // Create a mount point marker object.
   MountPointMarker marker;
   marker.marker_name = req.link_name;
   marker.link_name = req.link_name;
   marker.number_of_markers++;

   geometry_msgs::Point p;
   p.x = req.parent_position.x;
   p.y = req.parent_position.y;
   p.z = req.parent_position.z;

   geometry_msgs::Pose pose; 
   pose.position = p;

   marker.pose = pose;
   marker.marker_id = 0;

   int total = 0;
   for (int i = 0; i < marker_names.size(); i++)
   {
        if (marker_names.at(i) == marker.link_name) 
        {
             total++;
        } 
   }
   marker_names.push_back(marker.link_name);
   
   // Only need to append things if they have the same name.
   if (total)
   {
       // Append not the number of markers, but the number of count that is in there.
       marker.marker_id = total;
       marker.marker_name.append("_").append(SSTR(total));
   mount_point_markers.insert( std::pair<std::string, MountPointMarker>(marker.marker_name, marker) );
       ROS_INFO("Inserted marker %s to marker server", marker.marker_name.c_str());

   } else
   {
         mount_point_markers.insert( std::pair<std::string, MountPointMarker>(marker.marker_name, marker) );
       ROS_INFO("Inserted marker %s to marker server", marker.marker_name.c_str());
   }

   // Create 6dof marker with that link name. 
   tf::Vector3 position = tf::Vector3( marker.pose.position.x, marker.pose.position.y , marker.pose.position.z);
  
   make6DofMarkerWithName( marker.marker_name, marker.link_name, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
   res.spawned = true;
   server->applyChanges();

   return true;
}


// Start interactive marker server 
int main(int argc, char** argv)
{

   ros::init(argc, argv, "interactive_mount_points_server");
   ros::NodeHandle n;
   ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

   server.reset( new interactive_markers::InteractiveMarkerServer("interactive_mount_points_server","",false) );

   menu_handler.insert( "Attach", &processFeedback );
   menu_handler.insert( "Unattach", &processFeedback );
   menu_handler.insert( "Set Position", &processFeedback );
   menu_handler.insert( "Set Orientation", &processFeedback);
   //interactive_markers::MenuHandler::EntryHandle sub_menu_handle =    menu_handler.insert( "Submenu" );
   //menu_handler.insert( sub_menu_handle, "Attach", &processFeedback );
   //menu_handler.insert( sub_menu_handle, "Unattach", &processFeedback );

   ros::Duration(0.1).sleep();

   ros::ServiceServer service = n.advertiseService("spawn_mount_point_marker", spawn_mount_point_marker);
   ROS_INFO("Ready to spawn mount points");

   ros::ServiceServer update_service = n.advertiseService("update_mount_point_marker", update_mount_point_marker);
   ROS_INFO("Ready to update mount points");

    ros::ServiceServer clear_markers_service = n.advertiseService("clear_all_markers", clear_all_markers);
   ROS_INFO("Ready to remove mount points");

  ros::ServiceServer load_markers_service = n.advertiseService("load_mount_point_marker", load_mount_point_marker);
   ROS_INFO("Ready to load mount points");

  ros::ServiceServer get_all_markers_service = n.advertiseService("get_all_markers", get_all_markers);
   ROS_INFO("Ready to send all markers points");


   server->applyChanges();

   ros::spin();

   server.reset();

  return 0;
}
