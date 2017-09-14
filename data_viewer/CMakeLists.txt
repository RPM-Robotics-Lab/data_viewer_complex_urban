cmake_minimum_required(VERSION 2.8.3)
project(data_viewer)

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rosbag
  image_transport
  cv_bridge
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  pcl_msgs
  irp_sen_msgs
)


find_package(Eigen REQUIRED)

#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)


add_message_files(
  DIRECTORY msg
)

generate_messages(
   DEPENDENCIES
    std_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/data_viewer.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    data_viewer
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    message_runtime 
    image_transport 
    cv_bridge 
    dynamic_reconfigure 
    pcl_ros
    pcl_conversions  
    pcl_msgs
    irp_sen_msgs

  DEPENDS 
    Eigen
)


set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (data_viewer_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/glwidget.cpp ${SRC_DIR}/ROSThread.cpp)
set (data_viewer_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/glwidget.h ${SRC_DIR}/ROSThread.h)
set (data_viewer_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (data_viewer_QTBin_src ${SRC_DIR}/main.cpp)



#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${data_viewer_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
)


qt5_wrap_cpp(data_viewer_QTLib_hdr_moc ${data_viewer_QTLib_hdr})
qt5_wrap_ui (data_viewer_QTLib_ui_moc  ${data_viewer_QTLib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)



###########
## Build ##
###########

add_definitions(-std=c++11)

#add_definitions(${Qt5Widgets_DEFINITIONS})


add_library(data_viewer_QTLib SHARED
 ${data_viewer_QTLib_src}
 ${data_viewer_QTLib_hdr_moc}
 ${data_viewer_QTLib_ui_moc}
)

target_link_libraries (data_viewer_QTLib Qt5::Widgets Qt5::Gui Qt5::OpenGL)

add_executable(data_viewer ${data_viewer_QTLib_src} ${data_viewer_QTLib_hdr} ${data_viewer_QTBin_src} ${SHADER_RSC_ADDED})     

add_dependencies(data_viewer data_viewer_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(data_viewer ${catkin_EXPORTED_TARGETS})
add_dependencies(data_viewer_QTLib ${catkin_EXPORTED_TARGETS})

target_link_libraries(data_viewer

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  data_viewer_QTLib
  GL
  ${BOOST_CUSTOM_LIBS}
  ${Eigen_LIBRARIES}
)



