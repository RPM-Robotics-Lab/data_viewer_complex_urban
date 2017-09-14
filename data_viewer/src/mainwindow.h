#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <iostream>
#include <QMainWindow>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QProcess>
#include <QThread>
#include "ROSThread.h"
#include <std_srvs/SetBool.h>
#include <QErrorMessage>
#include <QCloseEvent>
#include <QInputDialog>
#include <signal.h>
#include <algorithm>
#include <rosbag/bag.h>
#include <dirent.h>
#include <ctime>
#include <chrono>
#include <string.h>
#define R2D 180/PI
#define D2R PI/180
#define POWER_CTR_DELAY 200000
#define INTENSITY_MIN 0.0
#define INTENSITY_MAX 80.0
#define INTENSITY_COLOR_MIN 0.0
#define INTENSITY_COLOR_MAX 1.0

using namespace std;

extern QMutex mutex;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
   Q_OBJECT

public:

  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  QMutex mutex;
  ROSThread *my_ros;

  QErrorMessage *error_;

  int point_cloud_color_mode_;

  void RosInit(ros::NodeHandle &n);
  void CpcCopy (QVector <QVector3D> &des, QVector <QVector3D> src);
  int GetDir(string dir, std::vector<string> &files);
private slots:

  //data assign function
  void SetPc();
  void SetEncoder();
  void SetFog();
  void SetAltimeter();
  void SetGps();
  void SetVrsGps();
  void SetXsensImu();
  void SetImgBox(QString msg);
  void SetImage();

  //point cloud color
  void SetCpcColorIntensity(bool checked);
  void SetCpcColorLeftRight(bool checked);
  void PoinCloudClear();

  QString GetTime();
  QString GetDay();
  //close
  void TryClose();



signals:

   void setThreadFinished(bool);

private:

  Ui::MainWindow *ui_;

};

#endif // MAINWINDOW_H
