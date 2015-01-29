#include <buildit_ros/start_screen.h>
//#include <buildit_ros/ui_mainwindow.h>

//public static
MyViz * StartScreen::visualizationDisplay;

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

   // Add Display widget for right horizontal layout.
   StartScreen::visualizationDisplay = new MyViz;
   StartScreen::visualizationDisplay->setGeometry(QRect(600, 0, 331, 501));

   // Tabs must be initialized after the viz display or else segfault occurs. The tabs depend on display's vars.
   tab_widget = new StartScreenTabWidget;

   centralLayout->addWidget(tab_widget);
   centralLayout->addWidget(StartScreen::visualizationDisplay);
   centralWidget->setLayout(centralLayout);
   this->setCentralWidget(centralWidget);
   this->showMaximized();
}

StartScreen::~StartScreen()
{
}
