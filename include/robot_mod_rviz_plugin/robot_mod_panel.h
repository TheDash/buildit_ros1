/*
* Copyright (c) 2015, Clearpath Robotics, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef ROBOT_MOD_PANEL_H
#define ROBOT_MOD_PANEL_H
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/default_plugin/robot_model_display.h>
#endif

class QLineEdit;

namespace robot_mod_plugin
{

	class RobotModPanel: public rviz::Panel
	{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
		Q_OBJECT
		public:
 		   
		    // QWidget subclass constructors usually take a parent widget
  		    // parameter (which usually defaults to 0). At the same time,
  		    // pluginlib::ClassLoader creates instances by calling the default
  		    // constructor (with no arguments). Taking the parameter and giving
   		    // a default of 0 lets the default constructor work and also lets
   		    // someone using the class for something else to pass in a parent
   		    // widget as they normally would with Qt.
   		    RobotModPanel( QWidget* parent = 0 );

		    // Now we declare overrides of rviz::Panel functions for saving and
		    // loading data from the config file. Here the data is the
		    // topic name.
		    virtual void load( const rviz::Config& config );
		    virtual void save( rviz::Config config ) const;

		    // The ROS node handle.
		    ros::NodeHandle nh_;

                    void export_urdf();
                    void update_urdf();
                    std::string exec(std::string cmd);
                    void set_robot_description(std::string path_to_xacro);
		public Q_SLOTS:
                 

	};

} // end namespace rviz_plugin_tutorials
#endif 
