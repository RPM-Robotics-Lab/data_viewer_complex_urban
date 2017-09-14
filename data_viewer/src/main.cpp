#include "mainwindow.h"
#include <QApplication>
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include "glwidget.h"
#include <ros/ros.h>
#include <QProcess>

int main(int argc, char *argv[])
{
//  glutInit(&argc, argv);
//  glewInit();


//  QProcess roscore_Process;
//  QProcessEnvironment env;
//  QString program;
//  roscore_Process.setProcessEnvironment(env);

//  if(roscore_Process.state() == QProcess::NotRunning){
//    program.clear();
//    program = "roscore";
//    roscore_Process.setProcessEnvironment(env);
//    roscore_Process.start(program);
//    roscore_Process.waitForStarted(-1);
//  }

//  sleep(1);

  ros::init(argc, argv, "sensor_driver_viewer");
  ros::NodeHandle nh;

  QApplication a(argc, argv);
  MainWindow w;
  w.RosInit(nh);
  w.show();


//  roscore_Process.terminate();
//  roscore_Process.waitForFinished();

//    GLWidget glwidget;
//    glwidget.ros_init(nh);
//    glwidget.resize(1280,720);
//    glwidget.show();

   return a.exec();
}
