#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::MainWindow)
{
  my_ros = new ROSThread(this, &mutex);
  ui_->setupUi(this);

  error_ = new QErrorMessage(this);
  point_cloud_color_mode_ = 2;

  connect(my_ros, SIGNAL(pc_signal()), this, SLOT(SetPc()));
  connect(my_ros, SIGNAL(encoder_signal()), this, SLOT(SetEncoder()));
  connect(my_ros, SIGNAL(fog_signal()), this, SLOT(SetFog()));
  connect(my_ros, SIGNAL(altimeter_signal()), this, SLOT(SetAltimeter()));
  connect(my_ros, SIGNAL(gps_signal()), this, SLOT(SetGps()));
  connect(my_ros, SIGNAL(vrs_gps_signal()), this, SLOT(SetVrsGps()));
  connect(my_ros, SIGNAL(xsens_imu_signal()), this, SLOT(SetXsensImu()));
  connect(my_ros, SIGNAL(add_img_box_signal(QString)), this, SLOT(SetImgBox(QString)));
  connect(my_ros, SIGNAL(image_update()), this, SLOT(SetImage()));

  //Point color selection
  connect(ui_->radioButton, SIGNAL(toggled(bool)), this, SLOT(SetCpcColorIntensity(bool)));
  connect(ui_->radioButton_2, SIGNAL(toggled(bool)), this, SLOT(SetCpcColorLeftRight(bool)));
  connect(ui_->pushButton_29, SIGNAL(pressed()), this, SLOT(PoinCloudClear()));

  ui_->comboBox_1->addItem("Image");
  ui_->comboBox_2->addItem("Image");
  ui_->comboBox_3->addItem("Image");
  ui_->comboBox_4->addItem("Image");

  connect(ui_->quitButton, SIGNAL(pressed()), this, SLOT(TryClose()));

  my_ros->start();
}

MainWindow::~MainWindow()
{
  emit setThreadFinished(true); //Tell the thread to finish
  delete ui_;
  my_ros->quit();
  if(!my_ros->wait(500)) //Wait until it actually has terminated (max. 3 sec)
  {
      my_ros->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
      my_ros->wait(); //We have to wait again here!
  }

}

void MainWindow::CpcCopy(QVector<QVector3D> &des, QVector<QVector3D> src)
{
  for (int i=0; i<src.size(); i++)
    des.push_back(src[i]);
}

void MainWindow::RosInit(ros::NodeHandle &n)
{
  my_ros->ros_initialize(n);
}

void MainWindow::SetPc()
{
  this->ui_->GLwidget->mutex.lock();
  this->ui_->GLwidget->cpc.clear();
  this->ui_->GLwidget->cpc_color.clear();
  this->ui_->GLwidget->mutex.unlock();

  QVector3D tmp;

  if(point_cloud_color_mode_ == 1){
    my_ros->mutex_VLP_left_.lock();
    for(int i = 0 ; i < (int)my_ros->vlp_left_cloud_.size(); i ++){
      if(my_ros->vlp_left_cloud_[i].intensity > INTENSITY_MAX) continue;
      tmp.setX(my_ros->vlp_left_cloud_[i].x);
      tmp.setY(my_ros->vlp_left_cloud_[i].y);
      tmp.setZ(my_ros->vlp_left_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      float normal_intensity =static_cast<float>(min(static_cast<int>(my_ros->vlp_left_cloud_[i].intensity),static_cast<int>(INTENSITY_MAX)) + (100 - INTENSITY_MAX))/100.0;
      float intensity_value = (INTENSITY_COLOR_MAX - INTENSITY_COLOR_MIN)*min(normal_intensity,(float)1.0);

//      float normal_intensity = (my_ros->vlp_left_cloud_[i].intensity - INTENSITY_MIN)/(INTENSITY_MAX - INTENSITY_MIN);
//      float intensity_value = (INTENSITY_COLOR_MAX - INTENSITY_COLOR_MIN)*normal_intensity;

      tmp.setX(intensity_value);
      tmp.setY(intensity_value);
      tmp.setZ(intensity_value);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_VLP_left_.unlock();

    my_ros->mutex_VLP_right_.lock();
    for(int i = 0 ; i < (int)my_ros->vlp_right_cloud_.size(); i ++){
      if(my_ros->vlp_right_cloud_[i].intensity > INTENSITY_MAX) continue;
      tmp.setX(my_ros->vlp_right_cloud_[i].x);
      tmp.setY(my_ros->vlp_right_cloud_[i].y);
      tmp.setZ(my_ros->vlp_right_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();

      float normal_intensity = static_cast<float>(min(static_cast<int>(my_ros->vlp_right_cloud_[i].intensity),static_cast<int>(INTENSITY_MAX)) + (100 - INTENSITY_MAX))/100.0;
      float intensity_value = (INTENSITY_COLOR_MAX - INTENSITY_COLOR_MIN)*min(normal_intensity,(float)1.0);

//      float normal_intensity = (my_ros->vlp_right_cloud_[i].intensity - INTENSITY_MIN)/(INTENSITY_MAX - INTENSITY_MIN);
//      float intensity_value = (INTENSITY_COLOR_MAX - INTENSITY_COLOR_MIN)*normal_intensity;
      tmp.setX(intensity_value);
      tmp.setY(intensity_value);
      tmp.setZ(intensity_value);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_VLP_right_.unlock();

    my_ros->mutex_SICK_back_.lock();
    for(int i = 0 ; i < (int)my_ros->sick_back_cloud_.size(); i ++){
      tmp.setX(my_ros->sick_back_cloud_[i].x);
      tmp.setY(my_ros->sick_back_cloud_[i].y);
      tmp.setZ(my_ros->sick_back_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(static_cast<double>(my_ros->sick_back_cloud_[i].intensity)/255.0);
      tmp.setY(static_cast<double>(my_ros->sick_back_cloud_[i].intensity)/255.0);
      tmp.setZ(static_cast<double>(my_ros->sick_back_cloud_[i].intensity)/255.0);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_SICK_back_.unlock();

    my_ros->mutex_SICK_middle_.lock();
    for(int i = 0 ; i < (int)my_ros->sick_middle_cloud_.size(); i ++){
      tmp.setX(my_ros->sick_middle_cloud_[i].x);
      tmp.setY(my_ros->sick_middle_cloud_[i].y);
      tmp.setZ(my_ros->sick_middle_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(static_cast<double>(my_ros->sick_middle_cloud_[i].intensity)/255.0);
      tmp.setY(static_cast<double>(my_ros->sick_middle_cloud_[i].intensity)/255.0);
      tmp.setZ(static_cast<double>(my_ros->sick_middle_cloud_[i].intensity)/255.0);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_SICK_middle_.unlock();

    my_ros->mutex->lock();
    for(int i = 0 ; i < (int)my_ros->calibration_points_.size() ; i ++){
        tmp.setX(my_ros->calibration_points_[i].x);
        tmp.setY(my_ros->calibration_points_[i].y);
        tmp.setZ(my_ros->calibration_points_[i].z);
        this->ui_->GLwidget->mutex.lock();
        this->ui_->GLwidget->cpc.push_back(tmp);
        this->ui_->GLwidget->mutex.unlock();
        tmp.setX(1);
        tmp.setY(1);
        tmp.setZ(0);
        this->ui_->GLwidget->mutex.lock();
        this->ui_->GLwidget->cpc_color.push_back(tmp);
        this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex->unlock();
  }

  if(point_cloud_color_mode_ == 2){
    my_ros->mutex_VLP_left_.lock();
    for(int i = 0 ; i < (int)my_ros->vlp_left_cloud_.size(); i ++){
      tmp.setX(my_ros->vlp_left_cloud_[i].x);
      tmp.setY(my_ros->vlp_left_cloud_[i].y);
      tmp.setZ(my_ros->vlp_left_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(1);
      tmp.setY(0);
      tmp.setZ(0);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_VLP_left_.unlock();

    my_ros->mutex_VLP_right_.lock();
    for(int i = 0 ; i < (int)my_ros->vlp_right_cloud_.size(); i ++){
      tmp.setX(my_ros->vlp_right_cloud_[i].x);
      tmp.setY(my_ros->vlp_right_cloud_[i].y);
      tmp.setZ(my_ros->vlp_right_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(0);
      tmp.setY(1);
      tmp.setZ(0);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_VLP_right_.unlock();

    my_ros->mutex_SICK_back_.lock();
    for(int i = 0 ; i < (int)my_ros->sick_back_cloud_.size(); i ++){
      tmp.setX(my_ros->sick_back_cloud_[i].x);
      tmp.setY(my_ros->sick_back_cloud_[i].y);
      tmp.setZ(my_ros->sick_back_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(1);
      tmp.setY(1);
      tmp.setZ(1);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_SICK_back_.unlock();

    my_ros->mutex_SICK_middle_.lock();
    for(int i = 0 ; i < (int)my_ros->sick_middle_cloud_.size(); i ++){
      tmp.setX(my_ros->sick_middle_cloud_[i].x);
      tmp.setY(my_ros->sick_middle_cloud_[i].y);
      tmp.setZ(my_ros->sick_middle_cloud_[i].z);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
      tmp.setX(0.5);
      tmp.setY(0.7);
      tmp.setZ(1);
      this->ui_->GLwidget->mutex.lock();
      this->ui_->GLwidget->cpc_color.push_back(tmp);
      this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex_SICK_middle_.unlock();

    my_ros->mutex->lock();
    for(int i = 0 ; i < (int)my_ros->calibration_points_.size() ; i ++){
        tmp.setX(my_ros->calibration_points_[i].x);
        tmp.setY(my_ros->calibration_points_[i].y);
        tmp.setZ(my_ros->calibration_points_[i].z);
        this->ui_->GLwidget->mutex.lock();
        this->ui_->GLwidget->cpc.push_back(tmp);
        this->ui_->GLwidget->mutex.unlock();
        tmp.setX(1);
        tmp.setY(1);
        tmp.setZ(0);
        this->ui_->GLwidget->mutex.lock();
        this->ui_->GLwidget->cpc_color.push_back(tmp);
        this->ui_->GLwidget->mutex.unlock();
    }
    my_ros->mutex->unlock();

  }


  this->ui_->GLwidget->update();
}

void MainWindow::SetEncoder()
{
  this->ui_->label_66->setText(QString::number(my_ros->encoder_data_.header.stamp.toSec(),'f',2));

  this->ui_->label_19->setText(QString::number(my_ros->encoder_data_.left_count));
  this->ui_->label_20->setText(QString::number(my_ros->encoder_data_.right_count));

}

void MainWindow::SetFog()
{
  this->ui_->label_77->setText(QString::number(my_ros->fog_3axis_data_.header.stamp.toSec(),'f',2));
  this->ui_->label_78->setText(QString::number(my_ros->fog_3axis_data_.d_roll));
  this->ui_->label_75->setText(QString::number(my_ros->fog_3axis_data_.d_pitch));
  this->ui_->label_34->setText(QString::number(my_ros->fog_3axis_data_.d_yaw));
}
void MainWindow::SetAltimeter()
{
  this->ui_->label_81->setText(QString::number(my_ros->altimeter_data_.header.stamp.toSec(),'f',2));
  this->ui_->label_64->setText(QString::number(my_ros->altimeter_data_.data));
}
void MainWindow::SetGps()
{

  this->ui_->label_89->setText(QString::number(my_ros->gps_data_.header.stamp.toSec(),'f',2));
  this->ui_->label_38->setText(QString::number(my_ros->gps_data_.longitude, 'f', 7));
  this->ui_->label_40->setText(QString::number(my_ros->gps_data_.latitude, 'f', 7));
}
void MainWindow::SetVrsGps()
{

  this->ui_->label_93->setText(QString::number(my_ros->vrs_gps_data_.header.stamp.toSec(),'f',2));

  this->ui_->label_44->setText(QString::number(my_ros->vrs_gps_data_.longitude,'f',7));
  this->ui_->label_48->setText(QString::number(my_ros->vrs_gps_data_.latitude,'f',7));
  this->ui_->label_94->setText(QString::number(my_ros->vrs_gps_data_.altitude,'f',7));
  if(my_ros->vrs_gps_data_.fix_state == 1){
    this->ui_->label_47->setText("<font color=#ff0000>normal</font>"); 
  }else if(my_ros->vrs_gps_data_.fix_state == 4){
    this->ui_->label_47->setText("<font color=#00ff00>fix</font>");
  }else if(my_ros->vrs_gps_data_.fix_state == 5){
    this->ui_->label_47->setText("<font color=#ffff00>float</font>");
  }else{
    this->ui_->label_47->setText("<font color=#ff0000>invalied</font>");
  }


}
void MainWindow::SetXsensImu()
{
  this->ui_->label_70->setText(QString::number(my_ros->xsens_imu_data_.header.stamp.toSec(),'f',2));

  this->ui_->label_26->setText(QString::number(my_ros->xsens_imu_data_.eular_data.x));
  this->ui_->label_28->setText(QString::number(my_ros->xsens_imu_data_.eular_data.y));
  this->ui_->label_30->setText(QString::number(my_ros->xsens_imu_data_.eular_data.z));

}
void MainWindow::SetImgBox(QString msg)
{
  ui_->comboBox_1->addItem(msg);
  ui_->comboBox_2->addItem(msg);
  ui_->comboBox_3->addItem(msg);
  ui_->comboBox_4->addItem(msg);

}
void MainWindow::SetImage()
{
  QString box[4];
  box[0] = ui_->comboBox_1->currentText();
  box[1] = ui_->comboBox_2->currentText();
  box[2] = ui_->comboBox_3->currentText();
  box[3] = ui_->comboBox_4->currentText();


  for(int i = 0 ; i < 4 ; i ++){
    QPixmap image_src;
    bool image_src_flag = false;

    my_ros->mutex->lock();

    if(!box[i].compare("/left/image_raw_color")){
      image_src = my_ros->left_raw_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/left/image_rect_color")){
      image_src = my_ros->left_rect_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/left/image_undistorted_color")){
      image_src = my_ros->left_undistorted_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/right/image_raw_color")){
      image_src = my_ros->right_raw_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/right/image_rect_color")){
      image_src = my_ros->right_rect_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/right/image_undistorted_color")){
      image_src = my_ros->right_undistorted_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/disparity/disparity_registered")){
      image_src = my_ros->disparity_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/depth/depth_registered")){
      image_src = my_ros->depth_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/confidence/confidence_registered")){
      image_src = my_ros->confidence_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/stereo/left/image_raw")){
      image_src = my_ros->stereo_left_raw_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/stereo/right/image_raw")){
      image_src = my_ros->stereo_right_raw_img_;
      image_src_flag = true;
    }
    if(!box[i].compare("/pg_14449005/image_raw")){
      image_src = my_ros->mono_back_raw_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/occam_node/image0")){
      image_src = my_ros->omni_0_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/occam_node/image1")){
      image_src = my_ros->omni_1_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/occam_node/image2")){
      image_src = my_ros->omni_2_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/occam_node/image3")){
      image_src = my_ros->omni_3_img_;
      image_src_flag = true;
    }

    if(!box[i].compare("/occam_node/image4")){
      image_src = my_ros->omni_4_img_;
      image_src_flag = true;
    }


    my_ros->mutex->unlock();

    if(image_src_flag == true){
      if(i == 0) this->ui_->label_13->setPixmap(image_src.scaled(this->ui_->label_13->size(),Qt::KeepAspectRatio));
      if(i == 1) this->ui_->label_14->setPixmap(image_src.scaled(this->ui_->label_14->size(),Qt::KeepAspectRatio));
      if(i == 2) this->ui_->label_9->setPixmap(image_src.scaled(this->ui_->label_9->size(),Qt::KeepAspectRatio));
      if(i == 3) this->ui_->label_10->setPixmap(image_src.scaled(this->ui_->label_10->size(),Qt::KeepAspectRatio));
    }
  }

}

void MainWindow::SetCpcColorIntensity(bool checked)
{
  if (checked)
      this->point_cloud_color_mode_ = 1;
}

void MainWindow::SetCpcColorLeftRight(bool checked)
{
  if (checked)
      this->point_cloud_color_mode_ = 2;
}


QString MainWindow::GetTime()
{
  string time;

  std::chrono::time_point<std::chrono::system_clock> time_c;
  time_c = std::chrono::system_clock::now();
  std::time_t timet = std::chrono::system_clock::to_time_t(time_c);
  struct tm *aTime = localtime(&timet);
  int hour = aTime->tm_hour;
  int min = aTime->tm_min;
  int sec = aTime->tm_sec;

  string str_hour;
  string str_min;
  string str_sec;
  if(to_string(hour).size() == 1){
    str_hour = "0" + to_string(hour);
  }else{
    str_hour = to_string(hour);
  }

  if(to_string(min).size() == 1){
    str_min = "0" + to_string(min);
  }else{
    str_min = to_string(min);
  }

  if(to_string(sec).size() == 1){
    str_sec = "0" + to_string(sec);
  }else{
    str_sec = to_string(sec);
  }

  time =  str_hour + str_min + str_sec;
  return QString::fromStdString(time);

}
QString MainWindow::GetDay()
{
  string time;
  std::chrono::time_point<std::chrono::system_clock> time_c;
  time_c = std::chrono::system_clock::now();
  std::time_t timet = std::chrono::system_clock::to_time_t(time_c);

  struct tm *aTime = localtime(&timet);
  int day = aTime->tm_mday;
  int month = aTime->tm_mon + 1; // Month is 0 - 11, add 1 to get a jan-dec 1-12 concept
  int year = aTime->tm_year + 1900; // Year is # years since 1900
  string str_day;
  string str_month;
  string str_year;

  str_year = to_string(year);

  if(to_string(month).size() == 1){
    str_month = "0" + to_string(month);
  }else{
    str_month = to_string(month);
  }

  if(to_string(day).size() == 1){
    str_day = "0" + to_string(day);
  }else{
    str_day = to_string(day);
  }

  time =  str_year + str_month + str_day;
  return QString::fromStdString(time);

}


int MainWindow::GetDir(string dir, std::vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

void MainWindow::TryClose()
{
  close();
}



void MainWindow::PoinCloudClear()
{
  my_ros->calibration_points_.clear();
  my_ros->vlp_left_cloud_.clear();
  my_ros->vlp_right_cloud_.clear();

}
