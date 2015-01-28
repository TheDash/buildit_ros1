#include <buildit_ros/buildit_config.h>

BuilditConfig::~BuilditConfig()
{

}

BuilditConfig::BuilditConfig() : 
    edit_positions(true), 
    edit_model(true),
    edit_orientation(true)
{

}

// Loads the current config loaded into the buildit_config namespace on ROSPARAM. 
void BuilditConfig::load()
{
   /* ros::param::get("buildit_config/edit_orientation", this->edit_orientation);
    ros::param::get("buildit_config/edit_position", this->edit_position);
    ros::param::get("buildit_config/modify_model", this->modify_model);
    ros::param::get("buildit_config/name", this->name);

    std::string package;
    std::string model;
    ros::param::get("buildit_config/package", package);
    ros::param::get("buildit_config/model", model);
    this->load_robot_description(package, model);
*/ // Instead, use a yaml parser.. .ffs
    // map;
    //ros::param::get("buildit_config/mount_points", map);
    //std::vector<std::map<

    //this->
}

void BuilditConfig::load_robot_description(std::string package, std::string model)
{
    // This will load the robot description so it can be viewed.
}

void BuilditConfig::save(std::string name)
{

}
