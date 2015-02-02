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
#include <ros/ros.h>

#include <buildit_ros/GetInteractiveMarkers.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/iterator.h>
#include <yaml-cpp/node.h>

class MountPointMarker
{
   public:
       MountPointMarker();
       ~MountPointMarker();

       // the link name is the link that the marker is relative to. 
       std::string link_name;
   
       // The marker name is the string that is shown on screen and a way to acces that specific marker.
       std::string marker_name;
       // The ID is the number after the marker name in the screen. 
       int marker_id;
       geometry_msgs::Pose pose;
       static int number_of_markers;
};

class MountPoint
{
   public:
       MountPoint();
       ~MountPoint();
       std::string link_location;
       std::vector<MountPointMarker> mount_point_markers;
};

class MountPoints
{
    public:
        MountPoints();
        ~MountPoints();
        std::map<std::string, MountPoint> mount_points;
        inline void add_mount_point(std::string link_location, MountPoint& p) { mount_points.insert(std::pair<std::string, MountPoint>(link_location, p)); }
        //inline void has_mount_point(MountPoint & p)

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
      void save(std::string& contents);
      void load_robot_description();
    private:
      void get_all_mount_point_markers_from_server();
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
