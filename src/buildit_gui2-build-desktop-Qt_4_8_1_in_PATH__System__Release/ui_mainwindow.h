/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Jan 9 21:47:19 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QToolBar>
#include <QtGui/QTreeView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *mount_points_tab;
    QTextBrowser *textBrowser;
    QLabel *label_2;
    QListView *listView_3;
    QLabel *label_3;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QTreeView *treeView;
    QWidget *tab_2;
    QLabel *label_4;
    QPushButton *pushButton_3;
    QLineEdit *lineEdit;
    QTextBrowser *textBrowser_2;
    QListView *listView_2;
    QLabel *label_5;
    QTableWidget *tableWidget;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QWidget *tab;
    QTextBrowser *textBrowser_3;
    QPushButton *pushButton_7;
    QPushButton *pushButton_8;
    QWidget *tab_3;
    QFrame *visualizationFrame;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(927, 544);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 591, 471));
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setMovable(false);
        mount_points_tab = new QWidget();
        mount_points_tab->setObjectName(QString::fromUtf8("mount_points_tab"));
        textBrowser = new QTextBrowser(mount_points_tab);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(80, 350, 441, 81));
        label_2 = new QLabel(mount_points_tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(100, 30, 141, 21));
        listView_3 = new QListView(mount_points_tab);
        listView_3->setObjectName(QString::fromUtf8("listView_3"));
        listView_3->setGeometry(QRect(340, 60, 231, 281));
        label_3 = new QLabel(mount_points_tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(400, 30, 161, 17));
        pushButton = new QPushButton(mount_points_tab);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(260, 130, 71, 27));
        pushButton_2 = new QPushButton(mount_points_tab);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(260, 240, 71, 27));
        treeView = new QTreeView(mount_points_tab);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setGeometry(QRect(10, 60, 241, 281));
        tabWidget->addTab(mount_points_tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 10, 161, 17));
        pushButton_3 = new QPushButton(tab_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(170, 30, 98, 27));
        lineEdit = new QLineEdit(tab_2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(10, 30, 151, 27));
        textBrowser_2 = new QTextBrowser(tab_2);
        textBrowser_2->setObjectName(QString::fromUtf8("textBrowser_2"));
        textBrowser_2->setGeometry(QRect(90, 360, 421, 61));
        listView_2 = new QListView(tab_2);
        listView_2->setObjectName(QString::fromUtf8("listView_2"));
        listView_2->setGeometry(QRect(10, 100, 231, 251));
        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 80, 191, 17));
        tableWidget = new QTableWidget(tab_2);
        if (tableWidget->columnCount() < 2)
            tableWidget->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(370, 100, 201, 251));
        tableWidget->setMaximumSize(QSize(261, 16777215));
        tableWidget->verticalHeader()->setDefaultSectionSize(30);
        pushButton_4 = new QPushButton(tab_2);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(260, 180, 98, 27));
        pushButton_5 = new QPushButton(tab_2);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(260, 250, 98, 27));
        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        textBrowser_3 = new QTextBrowser(tab);
        textBrowser_3->setObjectName(QString::fromUtf8("textBrowser_3"));
        textBrowser_3->setGeometry(QRect(50, 330, 411, 101));
        pushButton_7 = new QPushButton(tab);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        pushButton_7->setGeometry(QRect(180, 180, 241, 27));
        pushButton_8 = new QPushButton(tab);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        pushButton_8->setGeometry(QRect(510, 400, 71, 27));
        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        tabWidget->addTab(tab_3, QString());
        visualizationFrame = new QFrame(centralWidget);
        visualizationFrame->setObjectName(QString::fromUtf8("visualizationFrame"));
        visualizationFrame->setGeometry(QRect(610, 40, 311, 441));
        visualizationFrame->setFrameShape(QFrame::StyledPanel);
        visualizationFrame->setFrameShadow(QFrame::Raised);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(720, 0, 251, 20));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 927, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        textBrowser->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Turn a link into a mount point. The links below are all of the links in the robot. Select the link and add it to the mount point area by pressing the arrow. Added mount points will be highlighted in the robot model and labelled by their mount number. </p></body></html>", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Robot links", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Mount points", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Mount", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("MainWindow", "Unmount", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(mount_points_tab), QApplication::translate("MainWindow", "Step 1: Mount Points", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Add Sensors via URDF", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("MainWindow", "Browse", 0, QApplication::UnicodeUTF8));
        lineEdit->setText(QString());
        textBrowser_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Adding Sensors: first find all the URDF/Xacro files for all of the sensors you wish to add to the robot. Then, attach the sensors to the mount points.</p></body></html>", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Loaded sensors", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("MainWindow", "Sensor", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("MainWindow", "Mount", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("MainWindow", "Set Mount ", 0, QApplication::UnicodeUTF8));
        pushButton_5->setText(QApplication::translate("MainWindow", "Unmount", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Step 2: Add Sensors", 0, QApplication::UnicodeUTF8));
        textBrowser_3->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Confirm that your robot has the sensors mounted correctly. The red highlights show the mount points, and their respective URDF sensors are mounted to them. When you are sure that this is the robot you want, export the robot description!</p></body></html>", 0, QApplication::UnicodeUTF8));
        pushButton_7->setText(QApplication::translate("MainWindow", "Export Robot Description", 0, QApplication::UnicodeUTF8));
        pushButton_8->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Step 3: Confirm Robot", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Step 4: Export Robot", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Robot Model", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
