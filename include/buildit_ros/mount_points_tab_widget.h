#ifndef MOUNT_POINTS_TAB_WIDGET_H
#define MOUNT_POINTS_TAB_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLabel>
#include <QTableWidget>
#include <QList>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>

class MountPointsTabWidget : public QWidget
{
Q_OBJECT
    public: 
        MountPointsTabWidget(QWidget * parent = 0);
        ~MountPointsTabWidget();

    private:
        QPushButton * load_urdf_button;
        QPushButton * mount_button;
        QPushButton * unmount_button;

        QTableWidget * links_table;
        QTableWidget * selected_links_table;

        void load_robot_links();
        void create_mount_points_table_widget(); 
        void create_selected_mount_points_table_widget();
        void create_mount_button();
        void create_unmount_button();
        std::vector<const robot_model::LinkModel*> links;
        std::vector<const robot_model::LinkModel*> mount_point_links;
   private Q_SLOTS:
        void mount_button_clicked();
        void unmount_button_clicked();


};
#endif
