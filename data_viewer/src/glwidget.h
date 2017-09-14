#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QGLShaderProgram>
#include <QGLShader>
#include <QTimer>
#include <QDebug>
#include <vector>
#include <QVector3D>
#include <QGLBuffer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QEvent>
#include <QtMath>
#include <QGLBuffer>
#include <vector>
#include <QGLFunctions>
#include <GL/gl.h>

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QMouseEvent>
#include <QKeyEvent>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ROSThread.h"

using namespace std;


class GLWidget : public QGLWidget
{
  Q_OBJECT

public:
  explicit GLWidget(QWidget *parent = 0);
  ~GLWidget();
  QSize sizeHint() const;
  QMutex mutex;
//    ROSThread *my_ros;

protected:
  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();
  //! [2]
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent *event);
  void keyPressEvent(QKeyEvent *event);

  void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor color);
  void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <QVector3D> data, Qt::GlobalColor color);
  void drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QColor color, int stride);
  void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <QVector3D> data, QVector <QVector3D> data_color);
  void drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<GLfloat> data, QVector<GLfloat> data_color, int stride);
  void drawVehicle(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector3D cur_translation, QVector3D cur_rotation, QColor color);

  void loadVehiclefile();
public:
  QVector <QVector3D> axes;
  QVector <QVector3D> axes_color;
  QVector <QVector3D> cpc;
  QVector <QVector3D> cpc_color;
  QVector <QVector3D> grid;
  QVector <QVector3D> init_cube;
  QVector3D current_translation;
  QVector3D current_rotation;

  QGLShaderProgram shaderProgram;
  QGLShaderProgram shaderProgramColor;
  QOpenGLShaderProgram shaderProgram_obj;

  QOpenGLTexture *textures;

  QMatrix4x4 pMatrix;
  double alpha;
  double beta;
  double distance;
  QPoint lastMousePosition;
  double tr_x;
  double tr_y;
  float yaw;
  float pitch;

  GLuint texture;

  QVector3D cameraPosition;
  QVector3D cameraFront;
  QVector3D cameraUpDirection;

  QVector<QVector3D> vpose_trans;
  QVector<QVector3D> vpose_rot;

  QVector<QVector3D> vehicle_vertex;
  QVector<QVector2D> vehicle_texture;
  QVector<QVector3D> vehicle_normal;

  QVector<QVector3D> vehicle_face;
  QVector<QVector3D> vehicle_face_texture;
  QVector<QVector3D> vehicle_face_normal;

  QVector<QVector3D> vehicle_shape;
  QVector<QVector2D> vehicle_shape_texture;
  QVector<QVector3D> vehicle_shape_normal;

  QVector<QVector3D> vehicle_shape_global;

//  void ros_init(ros::NodeHandle &n);

private:
\
signals:
  void mouseEvent();

private:

  QTimer timer;
  QGLBuffer arrayBuf;


  QPoint lastPos;
private slots:

};

#endif // GLWIDGET_H
