#ifndef BUILDIT_CONFIG_H
#define BUILDIT_CONFIG_H

#include <string>
#include <map>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <ros/ros.h>

class BuilditConfig 
{


	struct MountPointMarker
	{
	    geometry_msgs::Pose pose;
	};
	struct MountPoint 
	{
	    std::vector<MountPointMarker> mount_point_markers;    
	};
	struct MountPoints
	{
	    std::vector<MountPoint> mount_points;
	};


    public:
      BuilditConfig();
     ~BuilditConfig();

      // Link names mapped to mount points. 
      //typedef std::map<std::string, geometry_msgs::Pose> MountPointsMap;
      inline std::string getName() { return name; }
      inline std::string getModelPath() { return model_path; }

      inline MountPoints getMountPoints() { return mount_points; }
      //inline std::map<std::string, std::vector<geometry_msgs::Pose> > getMountPoints() { return mount_points; }
      inline bool canEditPositions() { return edit_positions == "true" ? true : false; }
      inline bool canEditOrientation() { return edit_orientation == "true" ? true : false; }
      inline bool canEditModel() { return edit_model == "true" ? true : false; }
    
      void load(std::string name);
      void save(std::string config_name);
    private:

      std::string name;
      std::string model_path;
      // A map of links that have mount points, and where those mount points are positioned.
      //MountPointsMap mount_points;
      MountPoints mount_points;
      std::vector<MountPointMarker> mount_point_markers;
      std::string edit_positions;
      std::string edit_orientation;
      std::string edit_model;

      // Loads the current buildit_config/ namespace on the ROS Param server and sets it to be this object.

      // Saves the current buildit_config/ and exports it to a .yaml file. 

};

#endif
