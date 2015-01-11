#ifndef MOUNT_POINTS_TAB_WIDGET_H
#define MOUNT_POINTS_TAB_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLabel>
#include <QTableWidget>
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
        QTableWidget * links_table;
        QTableWidget * selected_links_table;
        void load_robot_links();
 
        std::vector<const robot_model::LinkModel*> links;
        std::vector<const robot_model::LinkModel*> mount_point_links;

};
#endif
