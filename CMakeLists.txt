cmake_minimum_required(VERSION 2.8.3)
project(buildit_ros)

find_package(catkin REQUIRED rviz roscpp pluginlib)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit QtOpenGL QtUiTools REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/mainwindow.h
    include/${PROJECT_NAME}/ui_mainwindow.h
    include/${PROJECT_NAME}/myviz.h
)

set(SOURCE_FILES
    src/main.cpp
    src/mainwindow.cpp
    src/myviz.cpp
    ${MOC_FILES}
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

#add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} default_plugin ${QT_LIBRARIES})

install(TARGETS
	${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ui/      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

