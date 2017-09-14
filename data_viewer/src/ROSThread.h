#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <irp_sen_msgs/vrs.h>
#include <irp_sen_msgs/altimeter.h>
#include <irp_sen_msgs/encoder.h>
#include <irp_sen_msgs/fog.h>
#include <irp_sen_msgs/imu.h>
#include <irp_sen_msgs/fog_3axis.h>
#include <irp_sen_msgs/LaserScanArray.h>

#include <dynamic_reconfigure/server.h>
#include <data_viewer/dynamic_data_viewerConfig.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "color.h"
#include "rosbag/bag.h"
#include <ros/transport_hints.h>

#define pc_type pcl::PointXYZI
#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI

using namespace std;

struct Point3D {

  float x;
  float y;
  float z;
  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {

  }
};


class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();


    dynamic_reconfigure::Server<dynamic_data_viewer::dynamic_data_viewerConfig> server_;
    dynamic_reconfigure::Server<dynamic_data_viewer::dynamic_data_viewerConfig>::CallbackType f_;
    QMutex *mutex;
    QMutex *total_file_mutex_;

    std::mutex mutex_VLP_left_;
    std::mutex mutex_VLP_right_;
    std::mutex mutex_SICK_back_;
    std::mutex mutex_SICK_middle_;

    std::mutex mutex_common_;

    queue<pair<int64_t,pcl::PointCloud<pc_type>>> velodyne_left_queue_;
    queue<pair<int64_t,pcl::PointCloud<pc_type>>> velodyne_right_queue_;
    queue<pair<int64_t,irp_sen_msgs::LaserScanArray>> sick_back_queue_;
    queue<pair<int64_t,irp_sen_msgs::LaserScanArray>> sick_middle_queue_;

    std::thread velodyne_left_save_thread_;
    std::thread velodyne_right_save_thread_;
    std::thread sick_back_save_thread_;
    std::thread sick_middle_save_thread_;

    std::condition_variable velodyne_left_cv_;
    std::condition_variable velodyne_right_cv_;
    std::condition_variable sick_back_cv_;
    std::condition_variable sick_middle_cv_;

    bool thread_run_;

    pcl::PointCloud<pc_type> vlp_left_cloud_;
    pcl::PointCloud<pc_type> vlp_right_cloud_;

    pcl::PointCloud<pc_type> calibration_points_;

    pcl::PointCloud<pc_type> sick_back_cloud_;
    pcl::PointCloud<pc_type> sick_middle_cloud_;

    //velodyne
    Eigen::Matrix3d left_vlp_rotation_matrix_;
    Eigen::Matrix3d right_vlp_rotation_matrix_;
    Eigen::Vector3d left_vlp_translation_vector_;
    Eigen::Vector3d right_vlp_translation_vector_;

    //sick
    Eigen::Matrix3d back_sick_rotation_matrix_;
    Eigen::Matrix3d middle_sick_rotation_matrix_;
    Eigen::Vector3d back_sick_translation_vector_;
    Eigen::Vector3d middle_sick_translation_vector_;

    //subscriber
    ros::Subscriber VLP_left_sub_;
    ros::Subscriber VLP_right_sub_;
    ros::Subscriber SICK_back_sub_;
    ros::Subscriber SICK_middle_sub_;
    ros::Subscriber encoder_count_sub_;

    ros::Subscriber altimeter_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber vrs_gps_sub_;
    ros::Subscriber xsens_imu_sub_;
    ros::Subscriber dsp1760_fog_sub_;

    //flee image sub
    ros::Subscriber stereo_left_img_sub_;
    ros::Subscriber stereo_right_img_sub_;
    ros::Subscriber mono_back_img_sub_;

    //Omni Cam
    ros::Subscriber omni_img_0_sub_;
    ros::Subscriber omni_img_1_sub_;
    ros::Subscriber omni_img_2_sub_;
    ros::Subscriber omni_img_3_sub_;
    ros::Subscriber omni_img_4_sub_;

    ros::Subscriber calibration_points_sub_;    //calibration points


    //publisher
    ros::Publisher power_control_pub_;

    //encoder data
    irp_sen_msgs::encoder encoder_data_;

    //altimeter data
    irp_sen_msgs::altimeter altimeter_data_;

    //gps data
    sensor_msgs::NavSatFix gps_data_;

    //vrs gps data
    irp_sen_msgs::vrs vrs_gps_data_;

    //xsens imu data

    irp_sen_msgs::imu xsens_imu_data_;

    //3 axis fog data

    irp_sen_msgs::fog_3axis fog_3axis_data_;

    QPixmap stereo_left_raw_img_;
    QPixmap stereo_right_raw_img_;
    QPixmap mono_back_raw_img_;

    bool stereo_left_raw_img_init_;
    bool stereo_right_raw_img_init_;
    bool mono_back_raw_img_init_;

    int stereo_left_count_;
    int stereo_right_count_;
    int mono_back_count_;
    int omni_count_;

    //ZED image data
    QPixmap left_raw_img_;
    QPixmap left_rect_img_;
    QPixmap left_undistorted_img_;
    QPixmap right_raw_img_;
    QPixmap right_rect_img_;
    QPixmap right_undistorted_img_;
    QPixmap disparity_img_;
    QPixmap depth_img_;
    QPixmap confidence_img_;

    //Omni image data
    QPixmap omni_0_img_;
    QPixmap omni_1_img_;
    QPixmap omni_2_img_;
    QPixmap omni_3_img_;
    QPixmap omni_4_img_;

    bool omni_0_img_init_;
    bool omni_1_img_init_;
    bool omni_2_img_init_;
    bool omni_3_img_init_;
    bool omni_4_img_init_;

    ros::Timer timer_;
    ros::Timer timer2_;
    ros::Timer timer3_;

    void ros_initialize(ros::NodeHandle &n_);
    void run();
    ros::NodeHandle n_;
    //callback function

    void VelodyneLeftCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void VelodyneRightCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void SickBackCallback(const irp_sen_msgs::LaserScanArray::ConstPtr& msg);
    void SickMiddleCallback(const irp_sen_msgs::LaserScanArray::ConstPtr& msg);

    void EncoderCountCallback(const irp_sen_msgs::encoder::ConstPtr& msg);
    void AltimeterCallback(const irp_sen_msgs::altimeter::ConstPtr& msg);
    void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void VrsGpsCallback(const irp_sen_msgs::vrs::ConstPtr& msg);
    void XsensImuCallback(const irp_sen_msgs::imu::ConstPtr& msg);
    void DSP1760Callback(const irp_sen_msgs::fog_3axis::ConstPtr& msg);


    void StereoLeftImgCallback(const sensor_msgs::ImageConstPtr& msg);
    void StereoRightImgCallback(const sensor_msgs::ImageConstPtr& msg);
    void MonoBackImgCallback(const sensor_msgs::ImageConstPtr& msg);

    //Omni

    void Omni0Callback(const sensor_msgs::ImageConstPtr& msg);
    void Omni1Callback(const sensor_msgs::ImageConstPtr& msg);
    void Omni2Callback(const sensor_msgs::ImageConstPtr& msg);
    void Omni3Callback(const sensor_msgs::ImageConstPtr& msg);
    void Omni4Callback(const sensor_msgs::ImageConstPtr& msg);

    void CalibrationPointCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    void TimerCallback(const ros::TimerEvent&);
    void ImageFrameCheckCallback(const ros::TimerEvent&);
    void PointcloudUpdateCallback(const ros::TimerEvent&);

    //publish
    void PowerControlPublish(string msg);

    void VelodyneLeftSave();
    void VelodyneRightSave();
    void SickBackSave();
    void SickMiddleSave();

signals:
    void pc_signal();
    void power_signal();
    void encoder_signal();
    void fog_signal();
    void altimeter_signal();
    void gps_signal();
    void vrs_gps_signal();
    void xsens_imu_signal();
    void image_update();
    void add_img_box_signal(QString msg);

private:
    //void run (void);

public slots:

};

#endif // VIEWER_LCM_H
