#include <buildit_ros/start_screen_tab_widget.h>

StartScreenTabWidget::StartScreenTabWidget(QWidget * parent) : QTabWidget(parent)
{
    	this->setObjectName(QString::fromUtf8("startScreenTabWidget"));
        this->setEnabled(true);
        this->setLayoutDirection(Qt::LeftToRight);
        this->setTabPosition(QTabWidget::North);
        this->setTabShape(QTabWidget::Rounded);

        // Add the widgets to the tab widget
}
 
void StartScreenTabWidget::addMountPointsTabWidget()
{
      mount_points_widget = new MountPointsTabWidget();
      this->addTab(mount_points_widget, QString(QString::fromStdString("Step 1: Mount Points")));
}

void StartScreenTabWidget::addSensorsTabWidget()
{
      add_sensors_widget = new AddSensorsTabWidget();
      this->addTab(add_sensors_widget, QString(QString::fromStdString("Step 2: Add Sensors")));
}

void StartScreenTabWidget::addExportRobotTabWidget()
{
      export_robot_widget = new ExportRobotTabWidget();
      this->addTab(export_robot_widget, QString(QString::fromStdString("Step 3: Export Robot")));
}

StartScreenTabWidget::~StartScreenTabWidget()
{
    delete this;
}
