#ifndef START_SCREEN_H
#define START_SCREEN_H

#include <QMainWindow>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
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
#include <buildit_ros/myviz.h>
#include <buildit_ros/start_screen_tab_widget.h>
QT_BEGIN_NAMESPACE


class StartScreen : public QMainWindow
{
    Q_OBJECT
    
public:

    ros::NodeHandle nh;
    // Functions
    explicit StartScreen(QWidget *parent = 0);
    ~StartScreen();
    static MyViz * visualizationDisplay;

private:


    // New fields
    QHBoxLayout * centralLayout;
    StartScreenTabWidget * tab_widget;

    // Fields
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QWidget *mount_points_tab;
    QTextBrowser *textBrowser;
    QListView *listView_3;
    QLabel *label_3;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QTreeView *treeView;
    QLabel *label_2;
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
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QFrame *visualizationFrame;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
};
#endif // MAINWINDOW_H
