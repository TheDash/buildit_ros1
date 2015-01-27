#ifndef MOUNT_POINTS_TAB_WIDGET_H
#define MOUNT_POINTS_TAB_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLabel>
#include <QTableWidget>
#include <QList>
#include <QFileDialog>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <fstream>
#include <QProcess>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <buildit_ros/InteractiveMountPoint.h>
#include <geometry_msgs/Vector3.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/visualization_manager.h>
#include <buildit_ros/SetOrientation.h>

#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

using namespace visualization_msgs;

class MountPointsTabWidget : public QWidget
{
Q_OBJECT
    public: 
        MountPointsTabWidget(QWidget * parent = 0);
        ~MountPointsTabWidget();
        bool set_marker_orientation_editor(buildit_ros::SetOrientation::Request&, buildit_ros::SetOrientation::Response&);

    private:
        QPushButton * load_urdf_base_button;
        QPushButton * mount_button;
        QPushButton * unmount_button;
        QPushButton * create_mount_points_button;
        QPushButton * hide_mount_points_button;  

        QTableWidget * links_table;
        QTableWidget * selected_links_table;

        void load_robot_links();

        ros::NodeHandle nh;

        

        void populate_links_table_after_button();
        void populate_links_table();
        void create_create_mount_points_button();
        void create_load_base_urdf_button();
        void create_hide_mount_points_button();
        void create_mount_points_table_widget(); 
        void create_selected_mount_points_table_widget();
        void create_mount_button();
        void create_unmount_button();
        std::vector<const robot_model::LinkModel*> links;
        std::vector<const robot_model::LinkModel*> mount_point_links;
   private Q_SLOTS:
        void hide_mount_points_button_clicked();
        void create_mount_points_button_clicked();
        void load_urdf_base_button_clicked();
        void mount_button_clicked();
        void unmount_button_clicked();


};
#endif
