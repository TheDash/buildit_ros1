//Copyright (c) Clearpath Robotics Inc, 2015.
// BSD License.

#include <string>
#include <stdio.h>
#include <iostream>
#include <ros/package.h>

#include <QApplication>
#include <QPushButton>
#include <QBoxLayout>
#include <QAction>
#include <QUiLoader>
#include <QFile>
#include <QDir>
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <QComboBox>

#include <geometry_msgs/Twist.h>

#include <robot_mod_rviz_plugin/robot_mod_panel.h>
#include <robot_mod_rviz_plugin/myviz.h>
#include <robot_mod_rviz_plugin/mainwindow.h>
#include <robot_mod_rviz_plugin/secdialog.h>
#include <robot_mod_rviz_plugin/ui_mainwindow.h>
#include <fstream>


namespace robot_mod_plugin
{

	RobotModPanel::RobotModPanel( QWidget* parent )
	  : rviz::Panel( parent )
	{

	  // TODO [DESIGN GUI]
	  // Modifications to the RobotMod Panel in RViz can be made here
	  // Look up the specifications for the Qt UI design. 

          std::string path = ros::package::getPath("robot_mod_rviz_plugin");
          std::string ui_path = "ui/mainwindow.ui";
          std::string ui_file_path = path + "/" + ui_path;

          QUiLoader loader;
          QFile mainwindow_ui_file(ui_file_path.c_str());
	  mainwindow_ui_file.open(QFile::ReadOnly);
          QWidget *myWidget = loader.load(&mainwindow_ui_file, this);

          MainWindow * mainwindow = new MainWindow(myWidget);
          mainwindow->show();

          ROS_INFO("Loaded UI %s", ui_file_path.c_str());
          mainwindow_ui_file.close();

	}

	//TODO [WHAT SHOULD BE SAVED?]
	//Effect: Saves the configuration of RViz
	//See the RViz plugin tutorials for example 
	void RobotModPanel::save( rviz::Config config ) const
	{

	}

	//TODO [WHAT SHOULD BE LOADED?]
	//Effect: loads the configuration of Rviz
	//See the RViz plugin tutorials for example 
	void RobotModPanel::load( const rviz::Config& config )
	{

	}

	//TODO [UPDATE THE URDF CONFIGURATION]
	//Input: a sensor configuration decided by the user
	//Output: void
	//Effect: Modifies the URDF to reflect the changes
	void RobotModPanel::update_urdf(/* SensorConfig &sc */)
	{

	}

	//TODO [Matt Tanguay] [GENERATE THE URDF CONFIGURATION]
	//Input: None
	//Output: None
	//Effect: exports the current urdf configuration to a user defined location or default location. This new urdf
	// has all of the sensor placements that the user decided
	void RobotModPanel::export_urdf()
	{
		// Read the robot description and save to a file.
            std::string robot_description;
            ros::param::get("robot_description", robot_description);
            std::ofstream file;
           // file.open("urdf/hackweek.
	}


        //TODO [Devon]
        //Input: path to xacro file. This calls the rosparam set function on the output of the xacro.py node from the ROS xacro pkg.
        //Output: None
        //Effect: sets the rosparam robot_description to whatever the output of the xacro file is.
        void RobotModPanel::set_robot_description(std::string path_to_xacro)
        {
            // Use the xacro.py file
            std::string cmd = "rosrun xacro xacro.py ";
            cmd += path_to_xacro;
            std::string urdf_string = this->exec(cmd);
            // WOOT THIS WORKS
            // Set the robot_description on param serv. 
	    ros::param::set("robot_description", urdf_string);
            std::string got_param;
            ros::param::get("robot_description", got_param);
        }

        //Helper function
	std::string RobotModPanel::exec(std::string cmd) {
	    FILE* pipe = popen(cmd.c_str(), "r");
	    if (!pipe) return "ERROR";
	    char buffer[128];
	    std::string result = "";
	    while(!feof(pipe)) {
	    	if(fgets(buffer, 128, pipe) != NULL)
	    		result += buffer;
	    }
	    pclose(pipe);
	    return result;
	}

} 

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_mod_plugin::RobotModPanel,rviz::Panel )
// END_TUTORIAL
