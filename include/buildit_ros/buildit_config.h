#ifndef BUILDIT_CONFIG_H
#define BUILDIT_CONFIG_H

#include <string>
#include <map>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <QString>
#include <QProcess>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/iterator.h>

class MountPointMarker
{
   public:
       MountPointMarker();
       ~MountPointMarker();
       std::string marker_name;
       geometry_msgs::Pose pose;
};

class MountPoint
{
   public:
       MountPoint();
       ~MountPoint();
       std::vector<MountPointMarker> mount_point_markers;
};

class MountPoints
{
    public:
        MountPoints();
        ~MountPoints();
        std::map<std::string, MountPoint> mount_points;
};


class BuilditConfig 
{
    public:
      BuilditConfig();
     ~BuilditConfig();

      // Link names mapped to mount points. 
      //typedef std::map<std::string, geometry_msgs::Pose> MountPointsMap;
      inline std::string getName() { return name; }
      inline std::string getModelPath() { return model_path; }
      inline bool canEditPositions() { return edit_positions == "true" ? true : false; }
      inline bool canEditOrientation() { return edit_orientation == "true" ? true : false; }
      inline bool canEditModel() { return modify_model == "true" ? true : false; }
      MountPoints mount_points;

      inline MountPoints getMountPoints() { return mount_points; }
      //inline std::map<std::string, std::vector<geometry_msgs::Pose> > getMountPoints() { return mount_points; }
    
      void load(std::string name);
      void save(std::string config_name);
      void load_robot_description();
    private:

      std::string name;
      std::string model_path;
      // A map of links that have mount points, and where those mount points are positioned.
      //MountPointsMap mount_points;
      std::vector<MountPointMarker> mount_point_markers;
      std::string edit_positions;
      std::string edit_orientation;
      std::string modify_model;
};

#endif
