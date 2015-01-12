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
    ROS_INFO("Loaded links");

    this->create_mount_points_table_widget();
    this->create_selected_mount_points_table_widget();
    this->create_mount_button();
    this->create_unmount_button();

    //TODO allow mounting buttons to move things over. [Done]
    //TODO make the mount points highlight links in the display
    // this means that everything in the selected links list will highlight something in the display
}


MountPointsTabWidget::~MountPointsTabWidget()
{

}

void MountPointsTabWidget::create_mount_points_table_widget()
{
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
    }
    this->links_table->setGeometry(QRect(20, 300, 400, 400));
    this->links_table->horizontalHeader()->setStretchLastSection(true);
    this->links_table->setHorizontalHeaderLabels(QString("Robot links;").split(";"));

}

void MountPointsTabWidget::create_selected_mount_points_table_widget()
{
  //  int row_count;
  //  row_count = this->links.size();

    this->selected_links_table = new QTableWidget(0, 1, this);
    ROS_INFO("Creating selection table..");
    this->selected_links_table->setGeometry(QRect(600, 300, 400, 400));
    this->selected_links_table->horizontalHeader()->setStretchLastSection(true);

    this->selected_links_table->setHorizontalHeaderLabels(QString("Mount points;").split(";"));

}

void MountPointsTabWidget::create_mount_button()
{
  this->mount_button = new QPushButton(">", this);
  this->mount_button->setGeometry(QRect(470, 450, 70, 40));

  // Associate button with function call
  connect(this->mount_button, SIGNAL(clicked()), this, SLOT(mount_button_clicked()));

}

// Takes selected links from list and adds it to selected table.
void MountPointsTabWidget::mount_button_clicked()
{
    QList<QTableWidgetItem*> selected_links = this->links_table->selectedItems();

    for (int i = 0; i < selected_links.size(); i++)
    {
        // Check if the string already exists in table. If it does, don't add
        if (this->selected_links_table->findItems(selected_links[i]->text(), Qt::MatchContains).empty())
        {
		// Add to mount points table if not already in it.
		this->selected_links_table->insertRow(0);
		QTableWidgetItem * newItem = selected_links[i]->clone();
		this->selected_links_table->setItem(0, 0, newItem);
		//links_to_be_added[i] = found_link;
		//this->links_table->removeRow(i);
		//this->selected_links_table->setItem(0, 0, found_link);

                // highlight the link in the display 
                StartScreen::visualizationDisplay->robot_state_display_->setLinkColor(newItem->text().toStdString(), QColor(0, 0, 240));
        } 
        else
        {
                ROS_WARN("%s is already in the mounted points table.", selected_links[i]->text().toStdString().c_str());
        }
    }


}

void MountPointsTabWidget::unmount_button_clicked()
{

}

void MountPointsTabWidget::create_unmount_button()
{
  this->unmount_button = new QPushButton("<", this);
  this->unmount_button->setGeometry(QRect(470, 500, 70, 40));

   connect(this->unmount_button, SIGNAL(clicked()), this, SLOT(unmount_button_clicked()));

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

