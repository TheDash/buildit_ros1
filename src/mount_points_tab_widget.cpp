#include <buildit_ros/mount_points_tab_widget.h>
#include <buildit_ros/start_screen.h>

MountPointsTabWidget::MountPointsTabWidget(QWidget * parent)
{
    // Functionality to add.
    // TODO File loader for URDF. Give URDF file to load. 
    // TODO Loaded URDF is set to robot description. 
    load_urdf_button = new QPushButton(QString(QString::fromStdString("Load URDF")), this);
    load_urdf_button->setGeometry(QRect(750, 35, 100, 50));
    load_urdf_button->setVisible(true);

    QString text_string(QString::fromStdString("Load your robot's URDF and select the mount points by moving the links you wish to set as mount points to the right side box"));
    QLabel * text_block = new QLabel(text_string, this);
    text_block->setGeometry(QRect(-25, 0, 900, 30));
    text_block->setAlignment(Qt::AlignCenter);

    this->load_robot_links();
    /*ROS_INFO("Loaded links");

    // Create the list of available links widget;
    int row_count;
    row_count = this->links.size();
    
    if (!this->links.empty())
    {
	    this->links_table = new QTableWidget(row_count, 1, this);
	    ROS_INFO("Creating data table..");
	    
	    // Load the QTableWidget
	    for (int i = 0; i < row_count; i++)
	    {
		const robot_model::LinkModel* link_model = this->links.at(i);
		std::string link_name = link_model->getName();
		ROS_INFO("Found link: %s ... Loading ... ", link_name.c_str());
		
		// Create the table item. 
		QTableWidgetItem * entry = new QTableWidgetItem(QString(QString::fromStdString(link_name)));
		this->links_table->setItem(i, 0, entry);
    }
    } else
    {
       ROS_WARN("No robot model loaded.. Please load a robot model.");
    }*/
}


MountPointsTabWidget::~MountPointsTabWidget()
{

}


void MountPointsTabWidget::load_robot_links()
{
   // Get the kinematic model from the display
   if (StartScreen::visualizationDisplay != NULL)
   {
	   robot_model::RobotModelConstPtr display_model = 	StartScreen::visualizationDisplay->robot_state_display_->getRobotModel();

           if (display_model)
           {
	           this->links = display_model->getLinkModels();
		   ROS_INFO("Got link models from display..");
           } 
           else
           {
                   ROS_WARN("No robot has been loaded. Please load a robot.");
           }
   } 
   else
   {
       ROS_WARN("The display has not been initialized yet...");
   }
}

