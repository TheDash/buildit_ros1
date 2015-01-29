#include <buildit_ros/buildit_config.h>

void operator >> (const YAML::Node& node, BuilditConfig::MountPoints mount_points);	
void operator >> (const YAML::Node& node, BuilditConfig::MountPoint& mount_point);
void operator >> (const YAML::Node& node, geometry_msgs::Quaternion& orientation);
void operator >> (const YAML::Node& node, geometry_msgs::Point& position);
void operator >> (const YAML::Node& node, BuilditConfig::MountPointMarker& marker);

BuilditConfig::~BuilditConfig()
{

}

BuilditConfig::BuilditConfig() : 
    edit_positions("true"), 
    modify_model("true"),
    edit_orientation("true")
{

}

// This parses a mount points node. It looks for all of the mount points inside of the mount_points tag and throws it down.
// E.g
// mount_points:
//     - mount_location_link_1:
//     - mount_location_link_2:
// It takes in a MountPoints type and breaks it down into an individual MountPoint. 
void operator >> (const YAML::Node& node, BuilditConfig::MountPoints mount_points)
{
   YAML::Iterator it;
   for (it = node.begin(); it != node.end(); ++it)
   {
      std::string key = it.first().to<std::string>();
      BuilditConfig::MountPoint point;
      node[key] >> point;
      mount_points.mount_points[key] = point;
      ROS_INFO("Link %s has mount point markers.", key.c_str());
   }
}

// This parses the mount point node. It looks for all of the markers inside that mount point. E.g
// mount_points:
//     - link_1:
//           marker_1:
//           marker_2:
//           etc..
// This takes in a MountPoint type and breaks it down into individual markers.
void operator >> (const YAML::Node& node, BuilditConfig::MountPoint& mount_point)
{
   YAML::Iterator it;
   for (it = node.begin(); it != node.end(); ++it)
   {
      std::string key = it.first().to<std::string>();
      BuilditConfig::MountPointMarker marker;
      marker.marker_name = key;
      ROS_INFO("Adding mount point marker named %s", key.c_str());
      node[key] >> marker;
      mount_point.mount_point_markers.push_back(marker);
   }

}

// Extract quaternion
void operator >> (const YAML::Node& node, geometry_msgs::Quaternion& orientation)
{
   orientation.w = 1;
   node["r"] >> orientation.x;
   node["p"] >> orientation.y;
   node["y"] >> orientation.z;
}

// Extract position
void operator >> (const YAML::Node& node, geometry_msgs::Point& position)
{
   node["x"] >> position.x;
   node["y"] >> position.y;
   node["z"] >> position.z;
}

// Extract a marker
void operator >> (const YAML::Node& node, BuilditConfig::MountPointMarker& marker)
{
    node["position"] >> marker.pose.position;
    node["orientation"] >> marker.pose.orientation;
}

// Loads the current config loaded into the buildit_config namespace on ROSPARAM. 
void BuilditConfig::load(std::string name)
{
   std::ifstream fin(name.c_str());
   YAML::Parser parser(fin);
   YAML::Node doc;
   ROS_INFO("Loaded YAML Config file %s: ", name.c_str());
   parser.GetNextDocument(doc);	
   //YAML::Node buildit_config;
   //dox["buildit_config"] >> doc;
   //this->name = doc["name"].as<std::string>();
   //this->edit_positions = doc["edit_positions"].as<std::string>();
   //this->edit_orientation = doc["edit_orientation"].as<std::string>();
   //this->modify_model = doc["modify_model"].as<std::string>();
   //this->model_path = doc["model"].as<std::string>();
 
   //const YAML::Node& doc = dox["buildit_config"];
   //ROS_INFO("Node type of buildit_config %s", dox.Type());


   doc["name"] >> this->name;
   doc["edit_positions"] >> this->edit_positions;
   doc["edit_orientation"] >> this->edit_orientation;
   doc["modify_model"] >> this->modify_model;
   doc["model"] >> this->model_path;

   ROS_INFO("YAML name: %s", this->name.c_str());
   ROS_INFO("YAML edit_positions: %s", this->edit_positions.c_str());
   ROS_INFO("YAML edit_orientation: %s", this->edit_orientation.c_str());
   ROS_INFO("YAML modify_model: %s", this->modify_model.c_str());
   ROS_INFO("YAML model_path: %s", this->model_path.c_str());

   doc["mount_points"] >> this->mount_points;
}

void BuilditConfig::load_robot_description(std::string& filepath)
{
    // This will load the robot description so it can be viewed.
}

void BuilditConfig::save(std::string name)
{

}
