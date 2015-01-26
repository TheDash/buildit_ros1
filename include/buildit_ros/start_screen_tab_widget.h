#ifndef START_SCREEN_TAB_WIDGET_H
#define START_SCREEN_TAB_WIDGET_H

#include <QTabWidget>
#include <QString>
#include <buildit_ros/mount_points_tab_widget.h>
#include <buildit_ros/add_sensors_tab_widget.h>
#include <buildit_ros/export_robot_tab_widget.h>

class StartScreenTabWidget : public QTabWidget
{
   Q_OBJECT
   public:
   	StartScreenTabWidget(QWidget * parent = 0);
   	~StartScreenTabWidget();
   private:

        MountPointsTabWidget * mount_points_widget;
        AddSensorsTabWidget * add_sensors_widget;
        ExportRobotTabWidget * export_robot_widget;

        void addMountPointsTabWidget();
        void addSensorsTabWidget();
        void addExportRobotTabWidget();

};

#endif
