#include <buildit_ros/mount_points_tab_widget.h>
#include <buildit_ros/start_screen.h>	
using namespace visualization_msgs;


MountPointsTabWidget::MountPointsTabWidget(QWidget * parent)
{

    QString text_string(QString::fromStdString("Load your robot's URDF and select the mount points by moving the links you wish to set as mount points to the right side box"));
    QLabel * text_block = new QLabel(text_string, this);
    text_block->setGeometry(QRect(-25, 0, 900, 30));
    text_block->setAlignment(Qt::AlignCenter);
 
    // Advertise orientation service
    this->or_srv = this->nh.advertiseService("set_marker_orientation_editor", &MountPointsTabWidget::set_marker_orientation_editor, this);
    ROS_INFO("Ready to orientate markers.");

    this->load_robot_links();
    this->create_load_base_urdf_button();
    this->create_create_mount_points_button();
    this->create_mount_points_table_widget();

     if (!this->links.empty())
    {
	ROS_INFO("Populating links table!");
        this->populate_links_table();
    }
    this->create_selected_mount_points_table_widget();
    this->create_mount_button();
    this->create_unmount_button();
    this->create_hide_mount_points_button();
}


MountPointsTabWidget::~MountPointsTabWidget()
{

}

bool MountPointsTabWidget::set_marker_orientation_editor(buildit_ros::SetOrientation::Request &req, buildit_ros::SetOrientation::Response &res)
{

   return true;
}

void MountPointsTabWidget::hide_mount_points_button_clicked()
{
       rviz::VisualizationManager * vm = StartScreen::visualizationDisplay->manager_;
       rviz::ToolManager * tm = vm->getToolManager();

   if (this->hide_mount_points_button->text().toStdString() == "Hide Markers")
   {
       // It is in hide marker state. Hide the markers, and set the button to be the show markers button
       this->hide_mount_points_button->setText(QString("Show Markers"));

       // As well, change the current tool to be just MoveCamera.
       rviz::Tool * t = tm->getTool(0);
       tm->setCurrentTool(t);
   } else
   {
       rviz::Tool * t = tm->getTool(1);
       tm->setCurrentTool(t);
       this->hide_mount_points_button->setText(QString("Hide Markers"));
   }

}

void MountPointsTabWidget::create_hide_mount_points_button()
{
    hide_mount_points_button = new QPushButton(QString(QString::fromStdString("Hide Markers")), this);
    hide_mount_points_button->setGeometry(QRect(150, 80, 180, 50));
    hide_mount_points_button->setVisible(true);

     connect(this->hide_mount_points_button, SIGNAL(clicked()), this, SLOT(hide_mount_points_button_clicked()));

}


// This will take all of the links selected in the selected table
// and create interactive markers at those link points
void MountPointsTabWidget::create_mount_points_button_clicked()
{
     // get selected mount points and create markers for them.
     for (int i = 0; i < this->selected_links_table->rowCount(); i++)
     {
         QTableWidgetItem* item = this->selected_links_table->item(i, 0);

         // get link name
         QString link = item->text();
         std::string link_name = link.toStdString();

         // get link position
        Ogre::Vector3 position = StartScreen::visualizationDisplay->robot_state_display_->robot_->getRobot().getLink(link_name)->getPosition();
        
         // Make service call
         ros::ServiceClient client = this->nh.serviceClient<buildit_ros::InteractiveMountPoint>("spawn_mount_point_marker");
         buildit_ros::InteractiveMountPoint mp_msg;
         mp_msg.request.link_name = link_name;
         mp_msg.request.parent_name = link_name;
         mp_msg.request.parent_position.x = position.x;
         mp_msg.request.parent_position.y = position.y;
         mp_msg.request.parent_position.z = position.z;

         // Send the call 
         if (client.call(mp_msg))
         {

         }
         else
         {
            ROS_INFO("Failed to send markers to server");
         }
     }
}

void MountPointsTabWidget::create_create_mount_points_button()
{
    create_mount_points_button = new QPushButton(QString(QString::fromStdString("Create Mount Points")), this);
    create_mount_points_button->setGeometry(QRect(150, 30, 180, 50));
    create_mount_points_button->setVisible(true);

     connect(this->create_mount_points_button, SIGNAL(clicked()), this, SLOT(create_mount_points_button_clicked()));

}

void MountPointsTabWidget::populate_links_table_after_button()
{
  // Get the current tables
  int rowcount;
  rowcount = this->links_table->rowCount();
  for (int i =0; i < rowcount; i++)
  {
      this->links_table->removeRow(0);
  }

  // Now repopulate it.

  int newrowcount;
  newrowcount = this->links.size();

  for (int i = 0; i < newrowcount; i++)
  {
      this->links_table->insertRow(0);
      const robot_model::LinkModel* link_model = this->links.at(i);

      std::string link_name = link_model->getName(); 
      ROS_INFO("Found link: %s ... Loading ... ", link_name.c_str());
		
      // Create the table item. 
      QTableWidgetItem * entry = new QTableWidgetItem(QString(QString::fromStdString(link_name)));
      this->links_table->setItem(0, 0, entry);
  }

}

// Load the URDF and set the robot description to be whatever is inside the urdf 
void MountPointsTabWidget::load_urdf_base_button_clicked()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Open File"), "/home/");
    
    if (fileNames.size() == 1)
    {
      ROS_INFO("Selected file.");
      QString qFileName = fileNames.value(0);
      std::string fileName = qFileName.toStdString();
      if (!fileName.empty())
      {
        // Check if .urdf or .xacro
        if (qFileName.endsWith(".urdf"))
        {
           std::ifstream ifs(fileName.c_str());
           std::string content( (std::istreambuf_iterator<char>(ifs) ),
                                (std::istreambuf_iterator<char>()    )  );

           ros::param::set("robot_description", content);
           ROS_INFO("Set the robot description to %s ", fileName.c_str());
        }
        else if (qFileName.endsWith(".xacro"))
        {
           // This chunk of code just gets the output of rosrun xacro xacro.py and sets it to robot_description
           std::string cmd("rosrun xacro xacro.py ");
           cmd += fileName;
           QProcess process;
           process.start(cmd.c_str());
           process.waitForReadyRead();
           process.waitForFinished();
           QString output(process.readAllStandardOutput());
           std::string robot_desc = output.toStdString();
           ros::param::set("robot_description", robot_desc);
           ROS_INFO("Set the robot description to %s", fileName.c_str());
        }
        else 
        {
          ROS_WARN("URDF or XACRO file not selected. Try again.");
        }
 
      } else 
      {
         ROS_WARN("Something is wrong with the file name. Please file a bug report with what file you used.");
      } 
      
    } else if (fileNames.size() > 1)
    {
      ROS_WARN("Please select only one URDF/XACRO file. You may add parts later on.");

    } else if (fileNames.size() < 1)
    {
      ROS_WARN("Please select a URDF/XACRO file to be the robot base model.");
    }

    //QFileDialog dialog(this);
    //dialog.setFileMode(QFileDialog::AnyFile);

    // Spawn the RViz Model in visualization. (E.g create visualization display)
    StartScreen::visualizationDisplay->robot_state_display_->subProp("Robot Description")->setValue(QString::fromStdString( "robot_description" ));
    
    std::string robot_description = StartScreen::visualizationDisplay->robot_state_display_->subProp("Robot Description")->getValue().toString().toStdString();

    // Check param server for wherever robot_description is set. 
    std::string urdf_description;
    ros::param::get(robot_description, urdf_description);

    // Check if robot description exists
    if (!urdf_description.empty())
    {
       ROS_INFO("Robot description found. Loading robot model.");
       ROS_INFO("Desc: %s", robot_description.c_str());
       StartScreen::visualizationDisplay->robot_state_display_->reset();
       
    } else
    {
       ROS_INFO("Robot description not found. Please set robot model.");
    }

    this->load_robot_links();
    this->populate_links_table_after_button();
}

void MountPointsTabWidget::create_load_base_urdf_button()
{
    load_urdf_base_button = new QPushButton(QString(QString::fromStdString("Load URDF Base")), this);
    load_urdf_base_button->setGeometry(QRect(700, 35, 120, 50));
    load_urdf_base_button->setVisible(true);

     connect(this->load_urdf_base_button, SIGNAL(clicked()), this, SLOT(load_urdf_base_button_clicked()));
}

void MountPointsTabWidget::create_mount_points_table_widget()
{
    this->links_table = new QTableWidget(0, 1, this);
    this->links_table->setGeometry(QRect(20, 150, 400, 400));
    this->links_table->horizontalHeader()->setStretchLastSection(true);
    this->links_table->setHorizontalHeaderLabels(QString("Robot links;").split(";"));

}

void MountPointsTabWidget::create_selected_mount_points_table_widget()
{
    this->selected_links_table = new QTableWidget(0, 1, this);
    ROS_INFO("Creating selection table..");
    this->selected_links_table->setGeometry(QRect(520, 150, 400, 400));
    this->selected_links_table->horizontalHeader()->setStretchLastSection(true);

    this->selected_links_table->setHorizontalHeaderLabels(QString("Mount points;").split(";"));
}

void MountPointsTabWidget::create_mount_button()
{
  this->mount_button = new QPushButton(">", this);
  this->mount_button->setGeometry(QRect(450, 300, 70, 40));

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
     // Remove item from selected_links_table. Essentially inverse of above..
     QList<QTableWidgetItem*> selected_links = this->selected_links_table->selectedItems();

    for (int i = 0; i < selected_links.size(); i++)
    {
		// Add to mount points table if not already in it.
                StartScreen::visualizationDisplay->robot_state_display_->unsetLinkColor(selected_links[i]->text().toStdString());
                ROS_INFO("%s is no longer selected to be a mount point", selected_links[i]->text().toStdString().c_str());
		this->selected_links_table->removeRow(selected_links[i]->row());
    }

}

void MountPointsTabWidget::create_unmount_button()
{
  this->unmount_button = new QPushButton("<", this);
  this->unmount_button->setGeometry(QRect(450, 350, 70, 40));

   connect(this->unmount_button, SIGNAL(clicked()), this, SLOT(unmount_button_clicked()));

}

void MountPointsTabWidget::populate_links_table()
{

    int row_count;
    row_count = this->links.size();
    ROS_INFO("Row count %d", row_count);
    this->links_table = new QTableWidget(row_count, 1, this);
    this->links_table->setSortingEnabled(false);    
    this->links_table->setGeometry(QRect(20, 150, 400, 400));
    this->links_table->horizontalHeader()->setStretchLastSection(true);
    this->links_table->setHorizontalHeaderLabels(QString("Robot links;").split(";"));
    ROS_INFO("Current row count %d", this->links_table->rowCount());

    if (!this->links.empty())
    {

	    // Insert new data
	    for (int i = 0; i < row_count; i++)
	    {
		const robot_model::LinkModel* link_model = this->links.at(i);

		std::string link_name = link_model->getName();
		ROS_INFO("Found link: %s ... Loading ... ", link_name.c_str());
		
		// Create the table item. 
		QTableWidgetItem * entry = new QTableWidgetItem(QString(QString::fromStdString(link_name)));
		this->links_table->setItem(i, 0, entry);
           }
    }
    else
    {
       ROS_WARN("No robot model loaded.. Please load a robot model.");
    }

    ROS_INFO("Current row count %d", this->links_table->rowCount());
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
   } 
   else
   {
       ROS_WARN("The display has not been initialized yet...");
   }

}

