#include <robot_mod_rviz_plugin/mainwindow.h>
#include <robot_mod_rviz_plugin/ui_mainwindow.h>
#include <QMessageBox>
#include <iostream>
#include <robot_mod_rviz_plugin/secdialog.h>
#include <QDialog>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->robotbreed->addItem("Select Breed");  //gives robot breed its selections
    ui->robotbreed->addItem("Husky");
    ui->robotbreed->addItem("Jackal");
    ui->robotbreed->addItem("Grizzly");

    ui->robotbreedtype->addItem("Select Type");  //gives robot breed type its selections
    ui->robotbreedtype->addItem("Top Plate");
    ui->robotbreedtype->addItem("Top Plate with sensor arch");

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_addbutton_clicked()
{
int Array[9];
std::fill(Array, Array+9, 0);                   //Warn if nothing selected
    //if (ui->robotbreed->currentIndex()==0) {
      //      if (ui->robotbreedtype->currentIndex()==0){
        //        QMessageBox msgBox;
          //      msgBox.setText("Please select a Breed and Type");
            //    msgBox.exec();
            //}
    //}                                                           //Set for Husky
    //if (ui->robotbreed->currentIndex()==1) {
         //Array[0]=1;
   // }
        if (ui->robotbreed->currentIndex()==2) {                    //Set for jackal
        QMessageBox msgBox;
        msgBox.setText("Robot Breed not yet supported, select another");
        msgBox.exec();
         }

        else if (ui->robotbreed->currentIndex()==3) {                    //Set for Grizzly
        QMessageBox msgBox;
        msgBox.setText("Robot Breed not yet supported, select another");
        msgBox.exec();
        }
            else if (ui->robotbreed->currentIndex()==0) {                    //Set for no breed
            QMessageBox msgBox;
            msgBox.setText("Please select Breed");
            msgBox.exec();
            }


    if (ui->robotbreedtype->currentIndex()==1) {                //Set array for Top plate
        Array[1]=4;
    }
        else if (ui->robotbreedtype->currentIndex()==2) {                //Set array for Top plate and sensor arch
        Array[5]=5;
        }
            else if (ui->robotbreedtype->currentIndex()==0) {                    //Set for no type
            QMessageBox msgBox;
            msgBox.setText("Please select Type");
            msgBox.exec();
            }
    if (ui->robotbreed->currentIndex()==1) {                                    //spawning new screen
            if (ui->robotbreedtype->currentIndex()!=0){
                secdiaglog = new SecDialog(this);
                secdiaglog->show();
            }
    }
}
