/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <buildit_ros/myviz.h>

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  //main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  manager_->setFixedFrame (QString::fromStdString("base_link"));
  
  robot_state_display_ = new moveit_rviz_plugin::RobotStateDisplay();
  robot_state_display_->setName("Robot State");

  manager_->addDisplay(robot_state_display_, true);

  // Create an interaction tool..
  rviz::ToolManager * tm = manager_->getToolManager();
  tm->addTool( "rviz/Interact");
  rviz::Tool * default_tool = tm->getTool(1);
  tm->setDefaultTool(default_tool);
  tm->setCurrentTool(default_tool);  

  QStringList tools = tm->getToolClasses();
  ROS_INFO("The current tools are");
  for (int i = 0; i < tools.size(); i++)
  {
      ROS_INFO("Tool: %s", tools.at(i).toStdString().c_str());
  }
  //this->interaction_tool = new rviz::InteractionTool();

  
  //this->interaction_tool->initialize(render_panel_->getManager());
  ROS_INFO("initialized interaction tool");
  //this->interaction_tool->activate();
  ROS_INFO("Activated interaction tool");
  //this->interaction_tool->context_ = render_panel_->getContext();
  
  // Need to set things up so this works..
  //this->interaction_tool->onInitialize(); 

  // Add the marker display here 
  interactive_marker_display_ = new rviz::InteractiveMarkerDisplay();
  interactive_marker_display_->setTopic(QString("/interactive_mount_points_server/update"), QString());




  manager_->addDisplay(interactive_marker_display_, true);

  // Set robot description
  robot_state_display_->subProp("Robot Description")->setValue(QString::fromStdString( "robot_description" ));

  // Zoom into robot
  rviz::ViewController* view = manager_->getViewManager()->getCurrent();
  view->subProp( "Distance" )->setValue( 4.0f );

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}

