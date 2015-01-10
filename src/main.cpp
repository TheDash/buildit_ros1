#include <QtGui/QApplication>
#include <buildit_ros/mainwindow.h>
#include <ros/ros.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "buildit_ros");
    ros::NodeHandle nh;

    // TODO Embed Rviz frame. 

    QApplication a(argc, argv);
    MyViz* myviz = new MyViz();
    MainWindow w;
    w.show();
    
    return a.exec();
}
