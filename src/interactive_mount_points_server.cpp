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
void makeChessPieceMarker(const tf::Vector3& );
InteractiveMarkerControl& makeBoxControl(InteractiveMarker&);
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
visualization_msgs::Marker makeBox(InteractiveMarker&);  
// END GLOBAL VARS

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
   make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
   //makeChessPieceMarker( position );

   server->applyChanges();

   ros::spin();

   server.reset();

  return 0;
}
