#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <buildit_ros/InteractiveMountPoint.h>

using namespace visualization_msgs;

// GLOBAL VARS
static void alignMarker(const InteractiveMarkerFeedbackConstPtr&);
static void processFeedback( const InteractiveMarkerFeedbackConstPtr&);
void makeChessPieceMarker(tf::Vector3& );

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
visualization_msgs::Marker makeBox(InteractiveMarker&);  
// END GLOBAL VARS


Marker makeBox(InteractiveMarker &msg )
{
	Marker marker;
	marker.type = Marker::CUBE;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;
	return marker;
}


//Taken from RViz tutorials 
void makeChessPieceMarker(tf::Vector3& position)
{
  // Get the position from the link name. 
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "chess piece marker";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker,   visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

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

// The server will have to spawn markers at the locations told, and be passed messages. 
bool spawn_mount_point_marker(buildit_ros::InteractiveMountPoint::Request &req, buildit_ros::InteractiveMountPoint::Response &res)
{

   return true;
}


// Start interactive marker server 
int main(int argc, char** argv)
{

   ros::init(argc, argv, "interactive_mount_points_server");
   ros::NodeHandle n;
   ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

   server.reset( new interactive_markers::InteractiveMarkerServer("interactive_mount_points_server","",false) );

   menu_handler.insert( "First Entry", &processFeedback );
   menu_handler.insert( "Second Entry", &processFeedback );
   interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
   menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
   menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

   ros::Duration(0.1).sleep();

   ros::ServiceServer service = n.advertiseService("spawn_mount_point_marker", spawn_mount_point_marker);
   ROS_INFO("Ready to spawn mount points");

   tf::Vector3 position = tf::Vector3( 3,-3, 0);
   makeChessPieceMarker( position );

   server->applyChanges();

   ros::spin();

   server.reset();

  return 0;
}
