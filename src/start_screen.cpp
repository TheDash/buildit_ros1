#include <buildit_ros/start_screen.h>
//#include <buildit_ros/ui_mainwindow.h>

// Start screen initialization for buildit_ros
StartScreen::StartScreen(QWidget *parent)
{

   if (this->objectName().isEmpty())
   {
            this->setObjectName(QString::fromUtf8("StartScreen"));
   }

   // Added central widget that the layouts will connect to
   centralWidget = new QWidget;
   centralWidget->resize(1000, 600);
   
   centralLayout = new QHBoxLayout;

   //centralLayout = new QHBoxLayout;
  
   // Add Tab widget for left horizontal layout
   tab_widget = new StartScreenTabWidget;

   // Add Display widget for right horizontal layout.
   visualizationDisplay = new MyViz;
   visualizationDisplay->setGeometry(QRect(600, 0, 331, 501));

   centralLayout->addWidget(tab_widget);
   centralLayout->addWidget(visualizationDisplay);
   centralWidget->setLayout(centralLayout);
   this->setCentralWidget(centralWidget);
   //this->setLayout(centralLayout);
   //this->setVisible(true);
/*
   // This needs to be refactored.
   tabWidget = new QTabWidget(horizontalLayoutWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setEnabled(true);
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setMovable(false);
        mount_points_tab = new QWidget();
        mount_points_tab->setObjectName(QString::fromUtf8("mount_points_tab"));
        textBrowser = new QTextBrowser(mount_points_tab);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(80, 350, 441, 81));
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
        label_2 = new QLabel(mount_points_tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(90, 30, 141, 21));
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

        horizontalLayout->addWidget(tabWidget);

        horizontalLayoutWidget_2 = new QWidget(centralWidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(600, 0, 331, 501));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);

        MyViz *myviz = new MyViz(centralWidget);
        myviz->setGeometry(QRect(600, 0, 331, 501));
	//myviz->setFrameShape(QFrame::StyledPanel);
 
        /*visualizationFrame = new QFrame(horizontalLayoutWidget_2);
        visualizationFrame->setObjectName(QString::fromUtf8("visualizationFrame"));
        visualizationFrame->setFrameShape(QFrame::StyledPanel);
        visualizationFrame->setFrameShadow(QFrame::Raised);

        label = new QLabel(horizontalLayoutWidget_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(130, 20, 251, 20));

       // horizontalLayout_2->addWidget(visualizationFrame);

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

        tabWidget->setCurrentIndex(3);
*/

       // QMetaObject::connectSlotsByName(this); 
}

StartScreen::~StartScreen()
{
}
