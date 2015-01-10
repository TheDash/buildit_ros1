#include <QtGui/QApplication>
#include <buildit_ros/mainwindow.h>
#include <ros/ros.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "buildit_ros");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    return a.exec();
}
