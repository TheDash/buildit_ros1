cmake_minimum_required(VERSION 2.8.3)
project(buildit_ros)

find_package(catkin REQUIRED COMPONENTS std_msgs moveit_ros_visualization rviz roscpp pluginlib visualization_marker_tutorials buildit_msgs)

catkin_package(
   LIBRARIES buildit_config
)


include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit QtOpenGL QtUiTools REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/start_screen.h
    include/${PROJECT_NAME}/myviz.h
    include/${PROJECT_NAME}/start_screen_tab_widget.h
    include/${PROJECT_NAME}/add_sensors_tab_widget.h
    include/${PROJECT_NAME}/mount_points_tab_widget.h
    include/${PROJECT_NAME}/export_robot_tab_widget.h
    include/${PROJECT_NAME}/buildit_config.h
)

set(SOURCE_FILES
    src/main.cpp
    src/start_screen.cpp
    src/myviz.cpp
    src/start_screen_tab_widget.cpp
    src/add_sensors_tab_widget.cpp
    src/mount_points_tab_widget.cpp
    src/export_robot_tab_widget.cpp
    src/buildit_config.cpp
    ${MOC_FILES}
)

add_library(buildit_config src/buildit_config.cpp)
target_link_libraries(buildit_config ${catkin_LIBRARIES} yaml-cpp ${QT_LIBRARIES})
add_dependencies(buildit_config buildit_msgs_generate_messages_cpp buildit_msgs_gencpp ${catkin_EXPORTED_TARGETS})

#add_executable(interactive_mount_points_server src/interactive_mount_points_server.cpp )
#target_link_libraries(interactive_mount_points_server ${catkin_LIBRARIES} ${QT_LIBRARIES} buildit_config yaml-cpp)
#add_dependencies(interactive_mount_points_server interactive_mount_points_server_gencpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
#add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} default_plugin ${QT_LIBRARIES} yaml-cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS
	${PROJECT_NAME} buildit_config
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ui/      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

