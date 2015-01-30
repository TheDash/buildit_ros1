
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

bool update_mount_point_marker(buildit_ros::UpdateInteractiveMountPoint::Request &req, buildit_ros::UpdateInteractiveMountPoint::Response &res);

bool clear_all_markers(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res);

std::map<std::string, geometry_msgs::Vector3> parent_positions;
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

// Removes the marker from the server
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

   server->setPose(req.marker_name, req.new_pose);
   server->applyChanges();
   ROS_INFO("Successfully updated marker %s", req.marker_name.c_str());

   return true;
}

bool clear_all_markers(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{
    server->clear();
    server->applyChanges();
    return true;
}

int count_total_markers(std::string name)
{
         // actual name of the marker being checked
         std::string actual_name;
         std::string delimiter("_");
         //actual_name = name.substr(0, name.find(delimiter));
         

    int total = 0;
    for (int i = 0; i < marker_list.size(); i++)
    {
         // get the actual name of existing marker
         std::string existing_actual_name;
         std::string current_marker = marker_list.at(i);
         existing_actual_name = current_marker.substr(0, current_marker.find_last_of(delimiter));
         // get all the substring delimited and count them
              ROS_INFO(" %s is equal to %s", actual_name.c_str(), existing_actual_name.c_str());
         if (actual_name == existing_actual_name)
         {
              total++;
         }
    }
   ROS_INFO("Mount link %s now has %i markers", name.c_str(), total);
   return total;
}


std::map<std::string, int> marker_counts;
// The server will have to spawn markers at the locations told, and be passed messages. 
bool spawn_mount_point_marker(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{
   std::string name = req.link_name;
   std::string parent_name = req.parent_name;

   ROS_INFO("Marker count %d", marker_list.size());
   for (int i = 0; i < marker_list.size(); i++)
   {
       if (marker_list.at(i) == name.c_str())
       {
          //int nummarkers = count_total_markers(name);
          marker_counts.insert(std::pair<std::string, int> (name, 1));
          std::stringstream convert;
          convert << marker_counts[name];
          name.append("_").append(convert.str());
          marker_counts[name]++;
       }
   }
   marker_list.push_back(name);



   //visualization_msgs::InteractiveMarker m;
   /*if (server->get(name, m))
   {
     std::string markercount;
     std::string delimiter("_");
     markercount = name.substr(name.find(delimiter), name.size());
     int count = std::stoi(markercount);
     if (markercount == 1)
     {
         
     }
      // This means there's a marker already with the same name. But when adding the markers, shouldn't just keep a count of how many time that marker is in the server? 
// Marker names:
// sensor_link_1
// sensor_link_2
// sensor_link_3
// 
// From fresh:
// should add a marker named {link_name}_1
// and if {link_name}_X exists, create {link_name}_x+1
   }*/


  /* int total = 0;
   bool contains = false;
   std::string actual_name;
   for (int i = 0; i < marker_list.size(); i++) 
   {
       std::string marker_name = marker_list.at(i);
       std::string delimiter = "_";
       actual_name = marker_name.substr(0, marker_name.find(delimiter));
       if (actual_name.c_str() == name.c_str())
       {
          contains = true;
          total++;
       }
   }

   if (!contains)
   {
     marker_list.push_back(actual_name);
   } else
   {
     marker_list.push_back(actual_name.append("_").append(total + ""));
   }*/
   
   /*if (std::find(marker_list.begin(), marker_list.end(), name.c_str()) != marker_list.end())
   {
      int total = 0;
      // count how many times its in there.
      for (int i = 0; i < marker_list.size(); i++)
      {
         if (marker_list.at(i) == name)
         {
                total++;
         }
      }
      if (total > 0)
      {
       std::string num = static_cast<std::ostringstream*>( &(std::ostringstream() << total) )->str();
       name.append("_").append(num);
      }
   }*/
   // The position of the parent link should be passed in as a parameter.
   float x;
   float y;
   float z;
   x = req.parent_position.x;
   y = req.parent_position.y;
   z = req.parent_position.z;

   // Save parent_position object in map with the link_name, so that it can be queried. 
   parent_positions[name] = req.parent_position;

   // Create 6dof marker with that link name. 
   tf::Vector3 position = tf::Vector3( x, y , z);
   make6DofMarkerWithName( name, parent_name, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
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

   server->applyChanges();

   ros::spin();

   server.reset();

  return 0;
}
