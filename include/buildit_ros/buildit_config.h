#ifndef BUILDIT_CONFIG_H
#define BUILDIT_CONFIG_H

#include <string>
#include <map>
#include <geometry_msgs/Pose.h>
#include <vector>

class BuilditConfig 
{

    public:
      BuilditConfig();
     ~BuilditConfig();

      inline std::string getName() { return name; }
      inline std::string getModelPath() { return model_path; }
      inline std::map<std::string, std::vector<geometry_msgs::Pose> > getMountPoints() { return mount_points; }
      inline bool canEditPositions() { return edit_positions; }
      inline bool canEditOrientation() { return edit_orientation; }
      inline bool canEditModel() { return edit_model; }
    
    private:

      std::string name;
      std::string model_path;
      // A map of links that have mount points, and where those mount points are positioned.
      std::map<std::string, std::vector<geometry_msgs::Pose> > mount_points;
      
      bool edit_positions = true;
      bool edit_orientation = true;
      bool edit_model = true;

      // Loads the current buildit_config/ namespace on the ROS Param server and sets it to be this object.
      void load();

      // Saves the current buildit_config/ and exports it to a .yaml file. 
      void save(std::string config_name);

};

#endif
