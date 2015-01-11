#include <buildit_ros/mount_points_tab_widget.h>

MountPointsTabWidget::MountPointsTabWidget(QWidget * parent)
{
    // Functionality to add.
    // TODO File loader for URDF. Give URDF file to load. 
    // TODO Loaded URDF is set to robot description. 
    load_urdf_button = new QPushButton(QString(QString::fromStdString("Load URDF")), this);
    load_urdf_button->setGeometry(QRect(750, 35, 100, 50));
    load_urdf_button->setVisible(true);

    QString text_string(QString::fromStdString("Load your robot's URDF and select the mount points by moving the links you wish to set as mount points to the right side box"));
    QLabel * text_block = new QLabel(text_string, this);
    text_block->setGeometry(QRect(-25, 0, 900, 30));
    text_block->setAlignment(Qt::AlignCenter);
}


MountPointsTabWidget::~MountPointsTabWidget()
{

}
