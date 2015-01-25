#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <moveit/robot_state_rviz_plugin/robot_state_display.h>
#include <rviz/view_manager.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0 );
  moveit_rviz_plugin::RobotStateDisplay* robot_state_display_;
  rviz::InteractiveMarkerDisplay * interactive_marker_display_;
  virtual ~MyViz();

private Q_SLOTS:
	void setThickness( int thickness_percent );
	void setCellSize( int cell_size_percent );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
  rviz::Display* robot_model_;
};
// END_TUTORIAL
#endif // MYVIZ_H
