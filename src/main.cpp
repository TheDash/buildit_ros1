#include <QtGui/QApplication>
#include <buildit_ros/start_screen.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "buildit_ros");

    QApplication a(argc, argv);
    StartScreen start_screen;
    start_screen.show();
    
    return a.exec();
}
