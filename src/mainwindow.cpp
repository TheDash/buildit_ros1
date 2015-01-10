#include <buildit_ros/mainwindow.h>
#include <buildit_ros/ui_mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
 
    // Modify UI from here to add RViz window.

}

MainWindow::~MainWindow()
{
    delete ui;
}
