#include <robot_mod_rviz_plugin/secdialog.h>
#include <robot_mod_rviz_plugin/ui_secdialog.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <robot_mod_rviz_plugin/myviz.h>

SecDialog::SecDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SecDialog)
{
    ui->setupUi(this);

  ui->location1combo->addItem("Select Component");  //Location1 Selections
  ui->location1combo->addItem("SICK LMS111 LIDAR");
  ui->location1combo->addItem("SICK LMS151 LIDAR");
  ui->location1combo->addItem("SICK LMS511 LIDAR");
  ui->location1combo->addItem("Velodyne LIDAR");
  ui->location1combo->addItem("Axis PTZ Camera");
  ui->location1combo->addItem("Axis Fixed Camera");
  ui->location1combo->addItem("Bumblebee Stereo Camera");
  ui->location1combo->addItem("Novatel GPS");
  ui->location1combo->addItem("Microstrain IMU");

  ui->location2combo->addItem("Select Component");  //Location2 Selections
  ui->location2combo->addItem("SICK LMS111 LIDAR");
  ui->location2combo->addItem("SICK LMS151 LIDAR");
  ui->location2combo->addItem("SICK LMS511 LIDAR");
  ui->location2combo->addItem("Velodyne LIDAR");
  ui->location2combo->addItem("Axis PTZ Camera");
  ui->location2combo->addItem("Axis Fixed Camera");
  ui->location2combo->addItem("Bumblebee Stereo Camera");
  ui->location2combo->addItem("Novatel GPS");
  ui->location2combo->addItem("Microstrain IMU");

  ui->location3combo->addItem("Select Component");  //Location3 Selections
  ui->location3combo->addItem("SICK LMS111 LIDAR");
  ui->location3combo->addItem("SICK LMS151 LIDAR");
  ui->location3combo->addItem("SICK LMS511 LIDAR");
  ui->location3combo->addItem("Velodyne LIDAR");
  ui->location3combo->addItem("Axis PTZ Camera");
  ui->location3combo->addItem("Axis Fixed Camera");
  ui->location3combo->addItem("Bumblebee Stereo Camera");
  ui->location3combo->addItem("Novatel GPS");
  ui->location3combo->addItem("Microstrain IMU");

  ui->location4combo->addItem("Select Component");  //Location4 Selections
  ui->location4combo->addItem("SICK LMS111 LIDAR");
  ui->location4combo->addItem("SICK LMS151 LIDAR");
  ui->location4combo->addItem("SICK LMS511 LIDAR");
  ui->location4combo->addItem("Velodyne LIDAR");
  ui->location4combo->addItem("Axis PTZ Camera");
  ui->location4combo->addItem("Axis Fixed Camera");
  ui->location4combo->addItem("Bumblebee Stereo Camera");
  ui->location4combo->addItem("Novatel GPS");
  ui->location4combo->addItem("Microstrain IMU");

  ui->location5combo->addItem("Select Component");  //Location5 Selections
  ui->location5combo->addItem("SICK LMS111 LIDAR");
  ui->location5combo->addItem("SICK LMS151 LIDAR");
  ui->location5combo->addItem("SICK LMS511 LIDAR");
  ui->location5combo->addItem("Velodyne LIDAR");
  ui->location5combo->addItem("Axis PTZ Camera");
  ui->location5combo->addItem("Axis Fixed Camera");
  ui->location5combo->addItem("Bumblebee Stereo Camera");
  ui->location5combo->addItem("Novatel GPS");
  ui->location5combo->addItem("Microstrain IMU");

  ui->location6combo->addItem("Select Component");  //Location6 Selections
  ui->location6combo->addItem("SICK LMS111 LIDAR");
  ui->location6combo->addItem("SICK LMS151 LIDAR");
  ui->location6combo->addItem("SICK LMS511 LIDAR");
  ui->location6combo->addItem("Velodyne LIDAR");
  ui->location6combo->addItem("Axis PTZ Camera");
  ui->location6combo->addItem("Axis Fixed Camera");
  ui->location6combo->addItem("Bumblebee Stereo Camera");
  ui->location6combo->addItem("Novatel GPS");
  ui->location6combo->addItem("Microstrain IMU");

  ui->location7combo->addItem("Select Component");  //Location7 Selections
  ui->location7combo->addItem("SICK LMS111 LIDAR");
  ui->location7combo->addItem("SICK LMS151 LIDAR");
  ui->location7combo->addItem("SICK LMS511 LIDAR");
  ui->location7combo->addItem("Velodyne LIDAR");
  ui->location7combo->addItem("Axis PTZ Camera");
  ui->location7combo->addItem("Axis Fixed Camera");
  ui->location7combo->addItem("Bumblebee Stereo Camera");
  ui->location7combo->addItem("Novatel GPS");
  ui->location7combo->addItem("Microstrain IMU");
}


SecDialog::~SecDialog()
{
    delete ui;
}

void SecDialog::on_viewrobotbutton_clicked()
{
   ROS_INFO("VIEW ROBOT CLICKED");
   MyViz* myviz = new MyViz();
   myviz->show();
   ROS_INFO("Showing robot..");
}

void SecDialog::on_addbutton2_clicked()
{
    int location1[1];                                   //creates int for each location
    std::fill(location1, location1+1, 0);

    if (ui->location1combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location1[1]=1;
    }
    if (ui->location1combo->currentIndex()==3) {
        location1[1]=2;
    }
    if (ui->location1combo->currentIndex()==4) {
        location1[1]=3;
    }
    if (ui->location1combo->currentIndex()==5) {
        location1[1]=7;
    }
    if (ui->location1combo->currentIndex()==6) {
        location1[1]=10;
    }
    if (ui->location1combo->currentIndex()==7) {
        location1[1]=11;
    }
    if (ui->location1combo->currentIndex()==8) {
        location1[1]=6;
    }
    if (ui->location1combo->currentIndex()==9) {
        location1[1]=9;
    }
    if (ui->location1combo->currentIndex()==10) {
        location1[1]=8;
    }
    if (ui->location1ptu->isChecked()==true) {              //add 10 for PTU to location int
        location1[1]=location1[1]+10;
    }


    int location2[1];                                   //creates int for each location
    std::fill(location2, location2+1, 0);

    if (ui->location2combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location2[1]=1;
    }
    if (ui->location2combo->currentIndex()==3) {
        location2[1]=2;
    }
    if (ui->location2combo->currentIndex()==4) {
        location2[1]=3;
    }
    if (ui->location2combo->currentIndex()==5) {
        location2[1]=7;
    }
    if (ui->location2combo->currentIndex()==6) {
        location2[1]=10;
    }
    if (ui->location2combo->currentIndex()==7) {
        location2[1]=11;
    }
    if (ui->location2combo->currentIndex()==8) {
        location2[1]=6;
    }
    if (ui->location2combo->currentIndex()==9) {
        location2[1]=9;
    }
    if (ui->location2combo->currentIndex()==10) {
        location2[1]=8;
    }
    if (ui->location2ptu->isChecked()==true) {              //add 10 for PTU to location int
        location2[1]=location2[1]+10;
    }



    int location3[1];                                   //creates int for each location
    std::fill(location3, location3+1, 0);

    if (ui->location3combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location3[1]=1;
    }
    if (ui->location3combo->currentIndex()==3) {
        location3[1]=2;
    }
    if (ui->location3combo->currentIndex()==4) {
        location3[1]=3;
    }
    if (ui->location3combo->currentIndex()==5) {
        location3[1]=7;
    }
    if (ui->location3combo->currentIndex()==6) {
        location3[1]=10;
    }
    if (ui->location3combo->currentIndex()==7) {
        location3[1]=11;
    }
    if (ui->location3combo->currentIndex()==8) {
        location3[1]=6;
    }
    if (ui->location3combo->currentIndex()==9) {
        location3[1]=9;
    }
    if (ui->location3combo->currentIndex()==10) {
        location3[1]=8;
    }
    if (ui->location3ptu->isChecked()==true) {              //add 10 for PTU to location int
        location3[1]=location3[1]+10;
    }



    int location4[1];                                   //creates int for each location
    std::fill(location4, location4+1, 0);

    if (ui->location1combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location4[1]=1;
    }
    if (ui->location4combo->currentIndex()==3) {
        location4[1]=2;
    }
    if (ui->location4combo->currentIndex()==4) {
        location4[1]=3;
    }
    if (ui->location4combo->currentIndex()==5) {
        location4[1]=7;
    }
    if (ui->location4combo->currentIndex()==6) {
        location4[1]=10;
    }
    if (ui->location4combo->currentIndex()==7) {
        location4[1]=11;
    }
    if (ui->location4combo->currentIndex()==8) {
        location4[1]=6;
    }
    if (ui->location4combo->currentIndex()==9) {
        location4[1]=9;
    }
    if (ui->location4combo->currentIndex()==10) {
        location4[1]=8;
    }
    if (ui->location4ptu->isChecked()==true) {              //add 10 for PTU to location int
        location4[1]=location4[1]+10;
    }



    int location5[1];                                   //creates int for each location
    std::fill(location5, location5+1, 0);

    if (ui->location5combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location5[1]=1;
    }
    if (ui->location5combo->currentIndex()==3) {
        location5[1]=2;
    }
    if (ui->location5combo->currentIndex()==4) {
        location5[1]=3;
    }
    if (ui->location5combo->currentIndex()==5) {
        location5[1]=7;
    }
    if (ui->location5combo->currentIndex()==6) {
        location5[1]=10;
    }
    if (ui->location5combo->currentIndex()==7) {
        location5[1]=11;
    }
    if (ui->location5combo->currentIndex()==8) {
        location5[1]=6;
    }
    if (ui->location5combo->currentIndex()==9) {
        location5[1]=9;
    }
    if (ui->location5combo->currentIndex()==10) {
        location5[1]=8;
    }
    if (ui->location5ptu->isChecked()==true) {              //add 10 for PTU to location int
        location5[1]=location5[1]+10;
    }


    int location6[1];                                   //creates int for each location
    std::fill(location6, location6+1, 0);

    if (ui->location6combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location6[1]=1;
    }
    if (ui->location6combo->currentIndex()==3) {
        location6[1]=2;
    }
    if (ui->location6combo->currentIndex()==4) {
        location6[1]=3;
    }
    if (ui->location6combo->currentIndex()==5) {
        location6[1]=7;
    }
    if (ui->location6combo->currentIndex()==6) {
        location6[1]=10;
    }
    if (ui->location6combo->currentIndex()==7) {
        location6[1]=11;
    }
    if (ui->location6combo->currentIndex()==8) {
        location6[1]=6;
    }
    if (ui->location6combo->currentIndex()==9) {
        location6[1]=9;
    }
    if (ui->location6combo->currentIndex()==10) {
        location6[1]=8;
    }
    if (ui->location6ptu->isChecked()==true) {              //add 10 for PTU to location int
        location6[1]=location6[1]+10;
    }



    int location7[1];                                   //creates int for each location
    std::fill(location7, location7+1, 0);

    if (ui->location7combo->currentIndex()==2) {        //each selection adds the corresponding # to the locaiton int
        location7[1]=1;
    }
    if (ui->location7combo->currentIndex()==3) {
        location7[1]=2;
    }
    if (ui->location7combo->currentIndex()==4) {
        location7[1]=3;
    }
    if (ui->location7combo->currentIndex()==5) {
        location7[1]=7;
    }
    if (ui->location7combo->currentIndex()==6) {
        location7[1]=10;
    }
    if (ui->location7combo->currentIndex()==7) {
        location7[1]=11;
    }
    if (ui->location7combo->currentIndex()==8) {
        location7[1]=6;
    }
    if (ui->location7combo->currentIndex()==9) {
        location7[1]=9;
    }
    if (ui->location7combo->currentIndex()==10) {
        location7[1]=8;
    }
    if (ui->location7ptu->isChecked()==true) {              //add 10 for PTU to location int
        location7[1]=location7[1]+10;
    }

}
