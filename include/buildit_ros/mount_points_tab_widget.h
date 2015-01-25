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

using namespace visualization_msgs;

class MountPointsTabWidget : public QWidget
{
Q_OBJECT
    public: 
        MountPointsTabWidget(QWidget * parent = 0);
        ~MountPointsTabWidget();

    private:
        QPushButton * load_urdf_base_button;
        QPushButton * mount_button;
        QPushButton * unmount_button;
        QPushButton * create_mount_points_button;
        interactive_markers::MenuHandler menu_handler;
  
        QTableWidget * links_table;
        QTableWidget * selected_links_table;

        void load_robot_links();
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

        visualization_msgs::Marker makeBox (InteractiveMarker&);
        void makeChessPieceMarker(std::string&);
        void create_create_mount_points_button();
        void create_load_base_urdf_button();
        void create_mount_points_table_widget(); 
        void create_selected_mount_points_table_widget();
        void create_mount_button();
        void create_unmount_button();
        std::vector<const robot_model::LinkModel*> links;
        std::vector<const robot_model::LinkModel*> mount_point_links;
   private Q_SLOTS:
        void create_mount_points_button_clicked();
        void load_urdf_base_button_clicked();
        void mount_button_clicked();
        void unmount_button_clicked();


};
#endif
