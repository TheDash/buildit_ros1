#ifndef MOUNT_POINTS_TAB_WIDGET_H
#define MOUNT_POINTS_TAB_WIDGET_H

#include <QDoubleValidator>
#include <QLineEdit>
#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLabel>
#include <QTableWidget>
#include <QList>
#include <QFileDialog>
#include <QProcess>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <fstream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/visualization_manager.h>

#include <buildit_ros/InteractiveMountPoint.h>
#include <buildit_ros/SetOrientation.h>
#include <buildit_ros/SetPosition.h>
#include <buildit_ros/UpdateInteractiveMountPoint.h>
#include <buildit_ros/buildit_config.h>

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
        bool set_marker_position_editor(buildit_ros::SetPosition::Request&, buildit_ros::SetPosition::Response&);
  

    private:
        QLineEdit * pos_x_tb;
        QLineEdit * pos_y_tb;
        QLineEdit * pos_z_tb;
        QLineEdit * or_r_tb;
        QLineEdit * or_p_tb;
        QLineEdit * or_y_tb;

        visualization_msgs::InteractiveMarkerFeedback edited_marker;

        double ypos;
        double xpos;
        double zpos;
        double orr;
        double orp;
        double ory;

        QWidget * orientation_editor;
        QWidget * position_editor;      

        QPushButton * set_position_button;
        QPushButton * set_orientation_button;
        QPushButton * load_urdf_base_button;
        QPushButton * mount_button;
        QPushButton * unmount_button;
        QPushButton * create_mount_points_button;
        QPushButton * hide_mount_points_button;  

        BuilditConfig * buildit_config;

        QTableWidget * links_table;
        QTableWidget * selected_links_table;

        void load_robot_links();

        ros::NodeHandle nh;
        ros::ServiceServer or_srv;
        ros::ServiceServer pos_srv;
        void clear_marker_server();
        void create_mount_point_marker(std::string, geometry_msgs::Pose);
        void create_mount_point_markers();
        void create_marker_position_editor();
        void create_marker_orientation_editor();
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
        void set_position_button_clicked();
        void set_orientation_button_clicked();
        void hide_mount_points_button_clicked();
        void create_mount_points_button_clicked();
        void load_urdf_base_button_clicked();
        void mount_button_clicked();
        void unmount_button_clicked();


};
#endif
