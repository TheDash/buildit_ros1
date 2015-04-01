cmake_minimum_required(VERSION 2.8.3)
project(robot_mod_rviz_plugin)

find_package(catkin REQUIRED rviz roscpp pluginlib)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit QtOpenGL QtUiTools REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} forms/*.ui)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
#qt4_automoc(${QT_SOURCES})


#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})


qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/robot_mod_panel.h
    include/${PROJECT_NAME}/myviz.h
    include/${PROJECT_NAME}/secdialog.h
    include/${PROJECT_NAME}/mainwindow.h
    include/${PROJECT_NAME}/ui_mainwindow.h
    include/${PROJECT_NAME}/ui_secdialog.h
)

set(SOURCE_FILES
    src/robot_mod_panel.cpp
    src/myviz.cpp
    src/mainwindow.cpp
    src/secdialog.cpp
    ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} default_plugin ${QT_LIBRARIES})

install(TARGETS
	${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
