#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

dynamic_data_viewer::dynamic_data_viewerConfig config_data_;
bool parameter_update = false;

void dynamic_parameter_callback(dynamic_data_viewer::dynamic_data_viewerConfig &config, uint32_t level)
{

//  cout << "===============================================================================" << endl;
//  cout << "Left Velodyne" << endl;
//  cout << "roll: " << config.L_V_roll<< " pitch: " << config.L_V_pitch<< " yaw: " << config.L_V_yaw << endl;
//  cout << "x: " << config.L_V_x<< " y: " << config.L_V_y<< " z: " << config.L_V_z << endl;
//  cout << "===============================================================================" << endl;
//  cout << "Right Velodyne" << endl;
//  cout << "roll: " << config.R_V_roll<< " pitch: " << config.R_V_pitch<< " yaw: " << config.R_V_yaw << endl;
//  cout << "x: " << config.R_V_x<< " y: " << config.R_V_y<< " z: " << config.R_V_z << endl;
//  cout << "===============================================================================" << endl;
//  cout << "Back Sick" << endl;
//  cout << "roll: " << config.B_S_roll<< " pitch: " << config.B_S_pitch<< " yaw: " << config.B_S_yaw << endl;
//  cout << "x: " << config.B_S_x<< " y: " << config.B_S_y<< " z: " << config.B_S_z << endl;
//  cout << "===============================================================================" << endl;
//  cout << "Middle Sick" << endl;
//  cout << "roll: " << config.M_S_roll<< " pitch: " << config.M_S_pitch<< " yaw: " << config.M_S_yaw << endl;
//  cout << "x: " << config.M_S_x<< " y: " << config.M_S_y<< " z: " << config.M_S_z << endl;

  config_data_ = config;
  parameter_update = true;

}

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex(th_mutex)
{

  stereo_left_count_ = 0;
  stereo_right_count_ = 0;
  mono_back_count_ = 0;
  omni_count_ = 0;

  total_file_mutex_ = new QMutex;

  stereo_left_raw_img_init_ = false;
  stereo_right_raw_img_init_ = false;
  mono_back_raw_img_init_ = false;

  omni_0_img_init_ = false;
  omni_1_img_init_ = false;
  omni_2_img_init_ = false;
  omni_3_img_init_ = false;
  omni_4_img_init_ = false;

  vlp_left_cloud_.clear();
  vlp_right_cloud_.clear();

  thread_run_ = true;

  velodyne_left_save_thread_ = std::thread(&ROSThread::VelodyneLeftSave,this);
  velodyne_right_save_thread_ = std::thread(&ROSThread::VelodyneRightSave,this);
  sick_back_save_thread_ = std::thread(&ROSThread::SickBackSave,this);
  sick_middle_save_thread_ = std::thread(&ROSThread::SickMiddleSave,this);

}
ROSThread::~ROSThread()
{
    thread_run_ = false;
    usleep(100000);

    if(velodyne_left_save_thread_.joinable()){
      velodyne_left_cv_.notify_all();
      velodyne_left_save_thread_.join();
    }
    if(velodyne_right_save_thread_.joinable()){
      velodyne_right_cv_.notify_all();
      velodyne_right_save_thread_.join();
    }
    if(sick_back_save_thread_.joinable()){
      sick_back_cv_.notify_all();
      sick_back_save_thread_.join();
    }
    if(sick_middle_save_thread_.joinable()){
      sick_middle_cv_.notify_all();
      sick_middle_save_thread_.join();
    }
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
    //subscriber
    this->n_ = n;
    timer_ = n.createTimer(ros::Duration(0.03), boost::bind(&ROSThread::TimerCallback, this, _1));
    timer2_ = n.createTimer(ros::Duration(1.0), boost::bind(&ROSThread::ImageFrameCheckCallback, this, _1));
    timer3_ = n.createTimer(ros::Duration(0.05), boost::bind(&ROSThread::PointcloudUpdateCallback, this, _1));

    VLP_right_sub_ = n.subscribe<sensor_msgs::PointCloud2>("/ns1/velodyne_points", 1000, boost::bind(&ROSThread::VelodyneRightCallback, this, _1));
    VLP_left_sub_ = n.subscribe<sensor_msgs::PointCloud2>("/ns2/velodyne_points", 1000, boost::bind(&ROSThread::VelodyneLeftCallback, this, _1));
    SICK_back_sub_ = n.subscribe<irp_sen_msgs::LaserScanArray>("/lms511_back/scan", 1000, boost::bind(&ROSThread::SickBackCallback, this, _1));
    SICK_middle_sub_ = n.subscribe<irp_sen_msgs::LaserScanArray>("/lms511_middle/scan", 1000, boost::bind(&ROSThread::SickMiddleCallback, this, _1));

    encoder_count_sub_ = n.subscribe<irp_sen_msgs::encoder>("/encoder_count", 1000, boost::bind(&ROSThread::EncoderCountCallback, this, _1));
    altimeter_sub_ = n.subscribe<irp_sen_msgs::altimeter>("/altimeter_data", 1000, boost::bind(&ROSThread::AltimeterCallback, this, _1));
    gps_sub_ = n.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 1000, boost::bind(&ROSThread::GpsCallback, this, _1));
    vrs_gps_sub_ = n.subscribe<irp_sen_msgs::vrs>("/vrs_gps_data", 1000, boost::bind(&ROSThread::VrsGpsCallback, this, _1));
    xsens_imu_sub_ = n.subscribe<irp_sen_msgs::imu>("/xsens_imu_data", 1000, boost::bind(&ROSThread::XsensImuCallback, this, _1));
    dsp1760_fog_sub_ = n.subscribe<irp_sen_msgs::fog_3axis>("/dsp1760_data", 1000, boost::bind(&ROSThread::DSP1760Callback, this, _1));

    //Flee3 camera
    stereo_left_img_sub_ = n.subscribe<sensor_msgs::Image>("/stereo/left/image_raw", 100, boost::bind(&ROSThread::StereoLeftImgCallback, this, _1));
    stereo_right_img_sub_ = n.subscribe<sensor_msgs::Image>("/stereo/right/image_raw", 100, boost::bind(&ROSThread::StereoRightImgCallback, this, _1));
    mono_back_img_sub_ = n.subscribe<sensor_msgs::Image>("/pg_14449005/image_raw", 100, boost::bind(&ROSThread::MonoBackImgCallback, this, _1));

    //Omni cam
    omni_img_0_sub_ = n.subscribe<sensor_msgs::Image>("/occam_node/image0", 10, boost::bind(&ROSThread::Omni0Callback, this, _1));
    omni_img_1_sub_ = n.subscribe<sensor_msgs::Image>("/occam_node/image1", 10, boost::bind(&ROSThread::Omni1Callback, this, _1));
    omni_img_2_sub_ = n.subscribe<sensor_msgs::Image>("/occam_node/image2", 10, boost::bind(&ROSThread::Omni2Callback, this, _1));
    omni_img_3_sub_ = n.subscribe<sensor_msgs::Image>("/occam_node/image3", 10, boost::bind(&ROSThread::Omni3Callback, this, _1));
    omni_img_4_sub_ = n.subscribe<sensor_msgs::Image>("/occam_node/image4", 10, boost::bind(&ROSThread::Omni4Callback, this, _1));

    calibration_points_sub_ = n.subscribe<sensor_msgs::PointCloud2>("/calibration/ICP_points", 5, boost::bind(&ROSThread::CalibrationPointCallback, this, _1));
    //publisher
    power_control_pub_ = n.advertise<std_msgs::String>("/power_control_write", 1000);

    f_ = boost::bind(&dynamic_parameter_callback,  _1, _2);
    server_.setCallback(f_);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::VelodyneLeftCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    int64_t msg_stamp = msg->header.stamp.toNSec();
    pcl::PointCloud<pc_type> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    mutex_VLP_left_.lock();
    velodyne_left_queue_.push(make_pair(msg_stamp,tmp_cloud));
    mutex_VLP_left_.unlock();
    velodyne_left_cv_.notify_all();
}


void ROSThread::VelodyneRightCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    int64_t msg_stamp = msg->header.stamp.toNSec();
    pcl::PointCloud<pc_type> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    mutex_VLP_right_.lock();
    velodyne_right_queue_.push(make_pair(msg_stamp,tmp_cloud));
    mutex_VLP_right_.unlock();
    velodyne_right_cv_.notify_all();
}

void ROSThread::SickBackCallback(const irp_sen_msgs::LaserScanArray::ConstPtr& msg)
{
    int64_t msg_stamp = msg->header.stamp.toNSec();
    mutex_SICK_back_.lock();
    sick_back_queue_.push(make_pair(msg_stamp,*msg));
    mutex_SICK_back_.unlock();
    sick_back_cv_.notify_all();
}
void ROSThread::SickMiddleCallback(const irp_sen_msgs::LaserScanArray::ConstPtr& msg)
{
    int64_t msg_stamp = msg->header.stamp.toNSec();
    mutex_SICK_middle_.lock();
    sick_middle_queue_.push(make_pair(msg_stamp,*msg));
    mutex_SICK_middle_.unlock();
    sick_middle_cv_.notify_all();
}

void ROSThread::PowerControlPublish(string msg)
{
  std_msgs::String pub_msg;
  pub_msg.data = msg;
  power_control_pub_.publish(pub_msg);
}


void ROSThread::EncoderCountCallback(const irp_sen_msgs::encoder::ConstPtr& msg)
{
  encoder_data_ = *msg;
  emit encoder_signal();
}


void ROSThread::AltimeterCallback(const irp_sen_msgs::altimeter::ConstPtr& msg)
{
  altimeter_data_ = *msg;
  emit altimeter_signal();
}

void ROSThread::GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_data_ = *msg;
  emit gps_signal();
}

void ROSThread::VrsGpsCallback(const irp_sen_msgs::vrs::ConstPtr& msg)
{
  vrs_gps_data_ = *msg;
  emit vrs_gps_signal();
}
void ROSThread::XsensImuCallback(const irp_sen_msgs::imu::ConstPtr& msg)
{
  xsens_imu_data_ = *msg;
  emit xsens_imu_signal();
}
void ROSThread::DSP1760Callback(const irp_sen_msgs::fog_3axis::ConstPtr& msg)
{
  fog_3axis_data_ = *msg;
  emit fog_signal();
}


void ROSThread::StereoLeftImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  mutex->lock();
  stereo_left_count_++;
  mutex->unlock();
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
  mutex->lock();
  stereo_left_raw_img_ = QPixmap::fromImage(QImage(rgb.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(stereo_left_raw_img_init_ == false){
    stereo_left_raw_img_init_ = true;
    emit add_img_box_signal("/stereo/left/image_raw");
  }
}
void ROSThread::StereoRightImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  mutex->lock();
  stereo_right_count_++;
  mutex->unlock();
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
  mutex->lock();
  stereo_right_raw_img_ = QPixmap::fromImage(QImage(rgb.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(stereo_right_raw_img_init_ == false){
    stereo_right_raw_img_init_ = true;
    emit add_img_box_signal("/stereo/right/image_raw");
  }
}

void ROSThread::MonoBackImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  mutex->lock();
  mono_back_count_++;
  mutex->unlock();
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
  mutex->lock();
  mono_back_raw_img_ = QPixmap::fromImage(QImage(rgb.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(mono_back_raw_img_init_ == false){
    mono_back_raw_img_init_ = true;
    emit add_img_box_signal("/pg_14449005/image_raw");
  }
}

void ROSThread::ImageFrameCheckCallback(const ros::TimerEvent&)
{

}
void ROSThread::PointcloudUpdateCallback(const ros::TimerEvent&)
{
  emit pc_signal();
}


void ROSThread::Omni0Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
  mutex->lock();
  omni_0_img_ = QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(omni_0_img_init_ == false){
    omni_0_img_init_ = true;
    emit add_img_box_signal("/occam_node/image0");
  }
}

void ROSThread::Omni1Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);

  mutex->lock();
  omni_1_img_ = QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(omni_1_img_init_ == false){
    omni_1_img_init_ = true;
    emit add_img_box_signal("/occam_node/image1");
  }
}


void ROSThread::Omni2Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
//  rgb = cv_ptr->image;
  mutex->lock();
  omni_2_img_ = QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(omni_2_img_init_ == false){
    omni_2_img_init_ = true;
    emit add_img_box_signal("/occam_node/image2");
  }
}

void ROSThread::Omni3Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
//  rgb = cv_ptr->image;
  mutex->lock();
  omni_3_img_ = QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();

  if(omni_3_img_init_ == false){
    omni_3_img_init_ = true;
    emit add_img_box_signal("/occam_node/image3");
  }
}

void ROSThread::Omni4Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
//  rgb = cv_ptr->image;
  mutex->lock();
  omni_4_img_ = QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888));
  mutex->unlock();
  if(omni_4_img_init_ == false){
    omni_4_img_init_ = true;
    emit add_img_box_signal("/occam_node/image4");
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
  if(parameter_update == true){

    //Velodyne
    Eigen::AngleAxisd rollAngle_L(config_data_.L_V_roll*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_L(config_data_.L_V_pitch*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_L(config_data_.L_V_yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    left_vlp_rotation_matrix_ = yawAngle_L * pitchAngle_L * rollAngle_L;

    left_vlp_translation_vector_(0) = config_data_.L_V_x;
    left_vlp_translation_vector_(1) = config_data_.L_V_y;
    left_vlp_translation_vector_(2) = config_data_.L_V_z;

    Eigen::AngleAxisd rollAngle_R(config_data_.R_V_roll*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_R(config_data_.R_V_pitch*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_R(config_data_.R_V_yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    right_vlp_rotation_matrix_ = yawAngle_R * pitchAngle_R * rollAngle_R;

    right_vlp_translation_vector_(0) = config_data_.R_V_x;
    right_vlp_translation_vector_(1) = config_data_.R_V_y;
    right_vlp_translation_vector_(2) = config_data_.R_V_z;

    //Sick

    Eigen::AngleAxisd rollAngle_B(config_data_.B_S_roll*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_B(config_data_.B_S_pitch*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_B(config_data_.B_S_yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    back_sick_rotation_matrix_ = yawAngle_B * pitchAngle_B * rollAngle_B;

    back_sick_translation_vector_(0) = config_data_.B_S_x;
    back_sick_translation_vector_(1) = config_data_.B_S_y;
    back_sick_translation_vector_(2) = config_data_.B_S_z;

    Eigen::AngleAxisd rollAngle_M(config_data_.M_S_roll*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_M(config_data_.M_S_pitch*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_M(config_data_.M_S_yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    middle_sick_rotation_matrix_ = yawAngle_M * pitchAngle_M * rollAngle_M;

    middle_sick_translation_vector_(0) = config_data_.M_S_x;
    middle_sick_translation_vector_(1) = config_data_.M_S_y;
    middle_sick_translation_vector_(2) = config_data_.M_S_z;

    //global adjust parameter added
    Eigen::Matrix3d global_velodyne_added_rotationMatrix;
    Eigen::Vector3d global_velodyne_added_translationVector;
    Eigen::AngleAxisd rollAngle_global(0.0*M_PI/180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_global(0.0*M_PI/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle_global(config_data_.global_V_yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    global_velodyne_added_rotationMatrix = yawAngle_global * pitchAngle_global * rollAngle_global;
    global_velodyne_added_translationVector(0) = config_data_.global_V_x;
    global_velodyne_added_translationVector(1) = config_data_.global_V_y;
    global_velodyne_added_translationVector(2) = 0.0;

    Eigen::Matrix4d global_velodyne_added_transformMatrix = Eigen::Matrix4d::Identity();
    global_velodyne_added_transformMatrix.block(0,0,3,3) = global_velodyne_added_rotationMatrix;
    global_velodyne_added_transformMatrix.block(0,3,3,1) = global_velodyne_added_translationVector;

    //velodyne
    Eigen::Matrix4d Left_velodyne_transformMatrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Right_velodyne_transformMatrix = Eigen::Matrix4d::Identity();

    Left_velodyne_transformMatrix.block(0,0,3,3) = left_vlp_rotation_matrix_;
    Left_velodyne_transformMatrix.block(0,3,3,1) = left_vlp_translation_vector_;

    Right_velodyne_transformMatrix.block(0,0,3,3) = right_vlp_rotation_matrix_;
    Right_velodyne_transformMatrix.block(0,3,3,1) = right_vlp_translation_vector_;

    Left_velodyne_transformMatrix = global_velodyne_added_transformMatrix * Left_velodyne_transformMatrix;
    Right_velodyne_transformMatrix = global_velodyne_added_transformMatrix * Right_velodyne_transformMatrix;

    left_vlp_rotation_matrix_ = Left_velodyne_transformMatrix.block(0,0,3,3);
    left_vlp_translation_vector_ = Left_velodyne_transformMatrix.block(0,3,3,1);
    right_vlp_rotation_matrix_ = Right_velodyne_transformMatrix.block(0,0,3,3);
    right_vlp_translation_vector_ = Right_velodyne_transformMatrix.block(0,3,3,1);

    //sick
    Eigen::Matrix4d Back_sick_transformMatrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Middle_sick_transformMatrix = Eigen::Matrix4d::Identity();

    Back_sick_transformMatrix.block(0,0,3,3) = middle_sick_rotation_matrix_;
    Back_sick_transformMatrix.block(0,3,3,1) = back_sick_translation_vector_;

    Middle_sick_transformMatrix.block(0,0,3,3) = middle_sick_rotation_matrix_;
    Middle_sick_transformMatrix.block(0,3,3,1) = middle_sick_translation_vector_;

    Back_sick_transformMatrix = global_velodyne_added_transformMatrix * Back_sick_transformMatrix;
    Middle_sick_transformMatrix = global_velodyne_added_transformMatrix * Middle_sick_transformMatrix;

    middle_sick_rotation_matrix_ = Back_sick_transformMatrix.block(0,0,3,3);
    back_sick_translation_vector_ = Back_sick_transformMatrix.block(0,3,3,1);
    middle_sick_rotation_matrix_ = Middle_sick_transformMatrix.block(0,0,3,3);
    middle_sick_translation_vector_ = Middle_sick_transformMatrix.block(0,3,3,1);


    Eigen::Vector3d left_eular = left_vlp_rotation_matrix_.eulerAngles(2,1,0);
    cout << "=========================================================" << endl;
    cout << FYEL(" roll: ") << left_eular(2)*180/M_PI <<  FYEL(" pitch: ") << left_eular(1)*180/M_PI << FYEL(" Yaw: ") << left_eular(0)*180/M_PI << endl;
    cout << FYEL(" x: ") << left_vlp_translation_vector_(0) << FYEL(" y: ") << left_vlp_translation_vector_(1) << FYEL(" z: ") << left_vlp_translation_vector_(2) << endl;
    cout << FYEL("Please use this result for left lidar extrinsic parameter") << endl;

    Eigen::Vector3d right_eular = right_vlp_rotation_matrix_.eulerAngles(2,1,0);
    cout << "============================" << endl;
    cout << FYEL(" roll: ") << right_eular(2)*180/M_PI <<  FYEL(" pitch: ") << right_eular(1)*180/M_PI << FYEL(" Yaw: ") << right_eular(0)*180/M_PI << endl;
    cout << FYEL(" x: ") << right_vlp_translation_vector_(0) << FYEL(" y: ") << right_vlp_translation_vector_(1) << FYEL(" z: ") << right_vlp_translation_vector_(2) << endl;
    cout << FYEL("Please use this result for right lidar extrinsic parameter") << endl;

    Eigen::Vector3d back_sick_eular = middle_sick_rotation_matrix_.eulerAngles(2,1,0);
    cout << "============================" << endl;
    cout << FYEL(" roll: ") << back_sick_eular(2)*180/M_PI <<  FYEL(" pitch: ") << back_sick_eular(1)*180/M_PI << FYEL(" Yaw: ") << back_sick_eular(0)*180/M_PI << endl;
    cout << FYEL(" x: ") << back_sick_translation_vector_(0) << FYEL(" y: ") << back_sick_translation_vector_(1) << FYEL(" z: ") << back_sick_translation_vector_(2) << endl;
    cout << FYEL("Please use this result for back sick extrinsic parameter") << endl;

    Eigen::Vector3d middle_sick_eular = middle_sick_rotation_matrix_.eulerAngles(2,1,0);
    cout << "============================" << endl;
    cout << FYEL(" roll: ") << middle_sick_eular(2)*180/M_PI <<  FYEL(" pitch: ") << middle_sick_eular(1)*180/M_PI << FYEL(" Yaw: ") << middle_sick_eular(0)*180/M_PI << endl;
    cout << FYEL(" x: ") << middle_sick_translation_vector_(0) << FYEL(" y: ") << middle_sick_translation_vector_(1) << FYEL(" z: ") << middle_sick_translation_vector_(2) << endl;
    cout << FYEL("Please use this result for middle sick extrinsic parameter") << endl;

    parameter_update = false;
  }
  emit image_update();
}


void ROSThread::CalibrationPointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    mutex->lock();
    pcl::fromROSMsg(*msg, calibration_points_);
    mutex->unlock();
}

void ROSThread::VelodyneLeftSave(){

    while(1){
        std::unique_lock<std::mutex> ul(mutex_VLP_left_);
        velodyne_left_cv_.wait(ul);
        if(thread_run_ == false){
            mutex_common_.lock();
            cout << "Velodyne left save thread finish" << endl;
            mutex_common_.unlock();
            return;
        }
        ul.unlock();
        while (!velodyne_left_queue_.empty())
        {
            ul.lock();
            auto tmp_cloud = velodyne_left_queue_.front().second;
            auto msg_stamp = velodyne_left_queue_.front().first;
            velodyne_left_queue_.pop();
            ul.unlock();

            //Point cloud
            pcl::PointCloud<pc_type> transformed_cloud;

            int data_size = (int)tmp_cloud.size();

            for(int i = 0 ; i < (int)tmp_cloud.size() ; i ++){

              //save points to binary file in velodyne coordinate
              pc_type tmp_point;
              tmp_point = tmp_cloud.points[i];

              //coordinate transform to vehicle
              tmp_point.x = left_vlp_rotation_matrix_(0,0)*tmp_cloud[i].x+left_vlp_rotation_matrix_(0,1)*tmp_cloud[i].y+left_vlp_rotation_matrix_(0,2)*tmp_cloud[i].z+left_vlp_translation_vector_(0);
              tmp_point.y = left_vlp_rotation_matrix_(1,0)*tmp_cloud[i].x+left_vlp_rotation_matrix_(1,1)*tmp_cloud[i].y+left_vlp_rotation_matrix_(1,2)*tmp_cloud[i].z+left_vlp_translation_vector_(1);
              tmp_point.z = left_vlp_rotation_matrix_(2,0)*tmp_cloud[i].x+left_vlp_rotation_matrix_(2,1)*tmp_cloud[i].y+left_vlp_rotation_matrix_(2,2)*tmp_cloud[i].z+left_vlp_translation_vector_(2);
              transformed_cloud.push_back(tmp_point);
            }
            ul.lock();
            vlp_left_cloud_ = transformed_cloud;
            ul.unlock();

        }
    }
}

void ROSThread::VelodyneRightSave(){

    while(1){
        std::unique_lock<std::mutex> ul(mutex_VLP_right_);
        velodyne_right_cv_.wait(ul);
        if(thread_run_ == false){
            mutex_common_.lock();
            cout << "Velodyne right save thread finish" << endl;
            mutex_common_.unlock();
            return;
        }
        ul.unlock();
        while (!velodyne_right_queue_.empty())
        {
            ul.lock();
            auto tmp_cloud = velodyne_right_queue_.front().second;
            auto msg_stamp = velodyne_right_queue_.front().first;
            velodyne_right_queue_.pop();
            ul.unlock();


            //Point cloud
            pcl::PointCloud<pc_type> transformed_cloud;

            int data_size = (int)tmp_cloud.size();

            for(int i = 0 ; i < (int)tmp_cloud.size() ; i ++){

              //save points to binary file in velodyne coordinate
              pc_type tmp_point;
              tmp_point = tmp_cloud.points[i];

              //coordinate transform to vehicle
              tmp_point.x = right_vlp_rotation_matrix_(0,0)*tmp_cloud[i].x+right_vlp_rotation_matrix_(0,1)*tmp_cloud[i].y+right_vlp_rotation_matrix_(0,2)*tmp_cloud[i].z+right_vlp_translation_vector_(0);
              tmp_point.y = right_vlp_rotation_matrix_(1,0)*tmp_cloud[i].x+right_vlp_rotation_matrix_(1,1)*tmp_cloud[i].y+right_vlp_rotation_matrix_(1,2)*tmp_cloud[i].z+right_vlp_translation_vector_(1);
              tmp_point.z = right_vlp_rotation_matrix_(2,0)*tmp_cloud[i].x+right_vlp_rotation_matrix_(2,1)*tmp_cloud[i].y+right_vlp_rotation_matrix_(2,2)*tmp_cloud[i].z+right_vlp_translation_vector_(2);
              transformed_cloud.push_back(tmp_point);
            }
            ul.lock();
            vlp_right_cloud_ = transformed_cloud;
            ul.unlock();
        }
    }
}
void ROSThread::SickBackSave(){

    while(1){
        std::unique_lock<std::mutex> ul(mutex_SICK_back_);
        sick_back_cv_.wait(ul);
        if(thread_run_ == false){
            mutex_common_.lock();
            cout << "Sick back save thread finish" << endl;
            mutex_common_.unlock();
            return;
        }
        ul.unlock();
        while (!sick_back_queue_.empty())
        {
            ul.lock();
            auto scan_data = sick_back_queue_.front().second;
            sick_back_queue_.pop();
            ul.unlock();

            for(auto iter = scan_data.LaserScans.begin() ; iter != scan_data.LaserScans.end() ; iter++){

              auto msg_stamp = iter->header.stamp.toNSec();
              auto msg = *iter;
              double start_ang = -5.0;
              double angle_diff = msg.angle_increment*RTOD;
              pc_type point;
              pc_type transformed_point;
              pcl::PointCloud<pc_type> sick_back_transformed_cloud;

              int data_size = (int)msg.ranges.size();

              for (int i=0; i < msg.ranges.size(); i++) {
                  double alpha_deg = start_ang + static_cast<double>(i)*angle_diff;

                  point.x = msg.ranges[i]*cos(alpha_deg*DTOR);
                  point.y = msg.ranges[i]*sin(alpha_deg*DTOR);
                  point.z = 0.0;
                  point.intensity = msg.intensities[i];

                  transformed_point.x = back_sick_rotation_matrix_(0,0)*point.x+back_sick_rotation_matrix_(0,1)*point.y+back_sick_rotation_matrix_(0,2)*point.z+back_sick_translation_vector_(0);
                  transformed_point.y = back_sick_rotation_matrix_(1,0)*point.x+back_sick_rotation_matrix_(1,1)*point.y+back_sick_rotation_matrix_(1,2)*point.z+back_sick_translation_vector_(1);
                  transformed_point.z = back_sick_rotation_matrix_(2,0)*point.x+back_sick_rotation_matrix_(2,1)*point.y+back_sick_rotation_matrix_(2,2)*point.z+back_sick_translation_vector_(2);
                  transformed_point.intensity = msg.intensities[i];

                  sick_back_transformed_cloud.push_back(transformed_point);
              }
              ul.lock();
              sick_back_cloud_ = sick_back_transformed_cloud;
              ul.unlock();
            }

        }
    }

}
void ROSThread::SickMiddleSave(){

    while(1){
        std::unique_lock<std::mutex> ul(mutex_SICK_middle_);
        sick_middle_cv_.wait(ul);
        if(thread_run_ == false){
            mutex_common_.lock();
            cout << "Sick middle save thread finish" << endl;
            mutex_common_.unlock();
            return;
        }
        ul.unlock();
        while (!sick_middle_queue_.empty())
        {
          ul.lock();
          auto scan_data = sick_middle_queue_.front().second;
          sick_middle_queue_.pop();
          ul.unlock();

          for(auto iter = scan_data.LaserScans.begin() ; iter != scan_data.LaserScans.end() ; iter++){

            auto msg_stamp = iter->header.stamp.toNSec();
            auto msg = *iter;

            double start_ang = -5.0;
            double angle_diff = msg.angle_increment*RTOD;
            pc_type point;
            pc_type transformed_point;
            pcl::PointCloud<pc_type> sick_middle_transformed_cloud;

            int data_size = (int)msg.ranges.size();
            for (int i=0; i < msg.ranges.size(); i++) {
                double alpha_deg = start_ang + static_cast<double>(i)*angle_diff;

                point.x = msg.ranges[i]*cos(alpha_deg*DTOR);
                point.y = msg.ranges[i]*sin(alpha_deg*DTOR);
                point.z = 0.0;
                point.intensity = msg.intensities[i];

                transformed_point.x = middle_sick_rotation_matrix_(0,0)*point.x+middle_sick_rotation_matrix_(0,1)*point.y+middle_sick_rotation_matrix_(0,2)*point.z+middle_sick_translation_vector_(0);
                transformed_point.y = middle_sick_rotation_matrix_(1,0)*point.x+middle_sick_rotation_matrix_(1,1)*point.y+middle_sick_rotation_matrix_(1,2)*point.z+middle_sick_translation_vector_(1);
                transformed_point.z = middle_sick_rotation_matrix_(2,0)*point.x+middle_sick_rotation_matrix_(2,1)*point.y+middle_sick_rotation_matrix_(2,2)*point.z+middle_sick_translation_vector_(2);
                transformed_point.intensity = msg.intensities[i];

                sick_middle_transformed_cloud.push_back(transformed_point);
            }
            ul.lock();
            sick_middle_cloud_ = sick_middle_transformed_cloud;
            ul.unlock();
          }
        }
    }

}

