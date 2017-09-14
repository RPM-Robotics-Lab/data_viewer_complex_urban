#include <iostream>
#include "glwidget.h"
#include <GL/glut.h>

using namespace std;

GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(parent)
{

  cpc.clear();
  cpc_color.clear();

  alpha = 0;
  beta = 0;
  distance = 2.5;
  tr_x = 0;
  tr_y = 0;
  yaw = -90.f;
  pitch = 0;
  cameraPosition = QVector3D(-13, 0, 5);
  cameraFront = QVector3D(13, 0, -5);
  cameraUpDirection = QVector3D(1, 0, 0);

  setFocusPolicy(Qt::StrongFocus);

  vpose_trans.push_back(QVector3D(0, 0, 0));
  vpose_rot.push_back(QVector3D(0, 0, 0));

  init_cube.clear();

  float a = 0.1;
  float b = 0.07;
  float c = 0.07;
  init_cube << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
            << QVector3D(a, b, -c) << QVector3D(a, b, c) << QVector3D(a, -b, c) // 1
            << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c)
            << QVector3D(a, -b, -c) << QVector3D(a, -b, c) << QVector3D(-a, -b, c) // 2
            << QVector3D(a, b, c) << QVector3D(a, b, -c) << QVector3D(-a, b, -c)
            << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(a, b, c) //3
            << QVector3D(-a, -b, c) << QVector3D(a, -b, c) << QVector3D(a, b, c)
            << QVector3D(a, b, c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) // 4
            << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
            << QVector3D(a, b, -c) << QVector3D(-a, b, -c) << QVector3D(-a, -b, -c) // 5
            << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(-a, b, -c)
            << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) //6;
            << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c);

  vehicle_vertex.clear();
  vehicle_texture.clear();
  vehicle_normal.clear();
  vehicle_face.clear();
  vehicle_face_texture.clear();
  vehicle_face_normal.clear();
  vehicle_shape.clear();
  vehicle_shape_texture.clear();
  vehicle_shape_global.clear();
// loadVehiclefile();
//  my_ros->start();
}

GLWidget::~GLWidget()
{

}

QSize GLWidget::sizeHint() const
{
  return QSize(640, 480);
}

void GLWidget::initializeGL()
{
  glEnable(GL_DEPTH_TEST);

  qglClearColor(QColor(Qt::black));

  shaderProgram.addShaderFromSourceFile(QGLShader::Vertex, ":resources/vertexShader.vsh");
  shaderProgram.addShaderFromSourceFile(QGLShader::Fragment, ":resources/fragmentShader.fsh");
  shaderProgram.link();

  shaderProgramColor.addShaderFromSourceFile(QGLShader::Vertex, ":resources/vertexShaderColor.vsh");
  shaderProgramColor.addShaderFromSourceFile(QGLShader::Fragment, ":resources/fragmentShaderColor.fsh");
  shaderProgramColor.link();

  shaderProgram_obj.addShaderFromSourceFile(QOpenGLShader::Vertex, ":resources/vertexShaderOBJ.vsh");
  shaderProgram_obj.addShaderFromSourceFile(QOpenGLShader::Fragment, ":resources/fragmentShaderOBJ.fsh");
  shaderProgram_obj.link();

  //textures =  new QOpenGLTexture(QImage(QString("./src/labs_viewer/data/Lexus.png")));

  axes << QVector3D(0, 0, 0) << QVector3D(1, 0, 0)
       << QVector3D(0, 0, 0) << QVector3D(0, 1, 0)
       << QVector3D(0, 0, 0) << QVector3D(0, 0, 1);
  axes_color << QVector3D(1, 0, 0) << QVector3D(1, 0, 0)
             << QVector3D(0, 1, 0) << QVector3D(0, 1, 0)
             << QVector3D(0, 0, 1) << QVector3D(0, 0, 1);

  float max_size = 100;
  for (float i=0; i<=max_size; i = i+ 1) {
      grid.push_back(QVector3D(-max_size, i, 0)); grid.push_back(QVector3D(max_size, i, 0));
      grid.push_back(QVector3D(-max_size, -i, 0)); grid.push_back(QVector3D(max_size, -i, 0));
      grid.push_back(QVector3D(i, -max_size, 0)); grid.push_back(QVector3D(i, max_size, 0));
      grid.push_back(QVector3D(-i, -max_size, 0)); grid.push_back(QVector3D(-i, max_size, 0));
  }

}

void GLWidget::resizeGL(int width, int height)
{
  if (height == 0) {
      height = 1;
  }

  pMatrix.setToIdentity();
  pMatrix.perspective(60.0, (float) width / (float) height, 0.001, 1000);

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);   //<--- add
  glLoadIdentity();              //<--- add
}

void GLWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  QMatrix4x4 mMatrix;
  QMatrix4x4 vMatrix;

  mMatrix.setToIdentity();

  vMatrix.lookAt(cameraPosition, cameraPosition+cameraFront, cameraUpDirection);
  QMatrix4x4 T = pMatrix * vMatrix * mMatrix;

  glLineWidth(1);
  drawShader(shaderProgram, GL_LINES, T, grid, QColor(30,30,30));
  glLineWidth(3);
  drawShader(shaderProgramColor, GL_LINES, T, axes, axes_color);

  glPointSize(2);
  mutex.lock();
  drawShader(shaderProgramColor, GL_POINTS, T, cpc, cpc_color);
  mutex.unlock();

  glFlush();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor color)
{
  shader.bind();
  shader.setUniformValue("color", color);
  shader.setAttributeArray("vertex", data.constData());
  shader.enableAttributeArray("vertex");
  glDrawArrays(type, 0, data.size());
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, Qt::GlobalColor color)
{
  shader.bind();
  shader.setUniformValue("color", QColor(color));
  shader.setAttributeArray("vertex", data.constData());
  shader.enableAttributeArray("vertex");
  glDrawArrays(type, 0, data.size());
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QVector <GLfloat> data_color, int stride)
{
  shader.bind();
  shader.setAttributeArray("vertex", data.constData(), 3, 3*stride*sizeof(GLfloat));
  shader.enableAttributeArray("vertex");
  shader.setAttributeArray("color", data_color.constData(), 3, 3*stride*sizeof(GLfloat));
  shader.enableAttributeArray("color");
  if (stride==0) stride = 1;
  glDrawArrays(type, 0, data.size()/(3*stride));
  shader.disableAttributeArray("color");
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}
void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QColor color, int stride)
{
  shader.bind();
  shader.setUniformValue("color", color);
  shader.setAttributeArray("vertex", data.constData(), 3, 3*stride*sizeof(GLfloat));
  shader.enableAttributeArray("vertex");
  // shader.setAttributeArray("color", data_color.constData(), 3, 3*stride*sizeof(GLfloat));
  // shader.enableAttributeArray("color");
  if (stride==0) stride = 1;
  glDrawArrays(type, 0, data.size()/(3*stride));
  // shader.disableAttributeArray("color");
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}
void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color)
{
  shader.bind();
  shader.setAttributeArray("vertex", data.constData(), sizeof(QVector3D));
  shader.enableAttributeArray("vertex");
  shader.setAttributeArray("color", data_color.constData(), sizeof(QVector3D));
  shader.enableAttributeArray("color");
  glDrawArrays(type, 0, data.size());
  shader.disableAttributeArray("color");
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}

void GLWidget::drawVehicle(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector3D cur_translation, QVector3D cur_rotation, QColor color)
{
  vehicle_shape_global.clear();
  QMatrix4x4 tf;
  tf.setToIdentity();
  tf.rotate(qRadiansToDegrees(cur_rotation.x()), 1,0,0);
  tf.rotate(qRadiansToDegrees(cur_rotation.y()), 0,1,0);
  tf.rotate(qRadiansToDegrees(cur_rotation.z())+180, 0,0,1);
  for (int i=0; i<vehicle_shape.size(); i++) {
      vehicle_shape_global.push_back(tf*vehicle_shape[i]+cur_translation);
  }

  shader.bind();
  // shader.setUniformValue("color", QColor(color));
  shader.setUniformValue("mvpMatrix", T);
  shader.setAttributeArray("vertex", vehicle_shape_global.constData());
  shader.enableAttributeArray("vertex");

  shader.setAttributeArray("textureCoordinate", vehicle_shape_texture.constData());
  shader.enableAttributeArray("textureCoordinate");

  shader.setUniformValue("texture", 0);
  textures->bind();
  glFrontFace(GL_CW);
  glDrawArrays(type, 0, vehicle_shape_global.size());
  shader.disableAttributeArray("vertex");
  shader.disableAttributeArray("textureCoordinate");
  shader.release();

}


void GLWidget::mousePressEvent(QMouseEvent *event)
{
  lastMousePosition = event->pos();
  // cout << "mouse click event" << endl;
  event->accept();

  glPushMatrix();
  glBegin(GL_POINT);
  glColor3d(1, 1, 0);
  glVertex3f(0, 0, 0);
  glEnd();
  glPopMatrix();
}



void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
 // cout << "mouse event" << endl;
  float deltaX = event->x() - lastMousePosition.x();
  float deltaY = lastMousePosition.y() - event->y();
  float sensitivity = 0.1;

  float alpha;
  QVector3D pt_xy;
  QVector3D tmp_cam_pos;

  if (abs(cameraFront.z()) > 0.000005) {
    alpha = -cameraPosition.z() / cameraFront.z();
    pt_xy = cameraPosition + alpha*cameraFront;
  }
  else {
    pt_xy = cameraPosition;
  }

  tmp_cam_pos = cameraPosition - pt_xy;

  if (event->buttons() & Qt::LeftButton) { // translation motion
    QVector3D left = QVector3D::normal(cameraUpDirection, cameraFront);
    QVector3D z_fix_front = QVector3D(cameraUpDirection[0], cameraUpDirection[1], 0).normalized();
    cameraPosition = cameraPosition - (cameraPosition.z()*0.02*sensitivity)*deltaY*z_fix_front;
    cameraPosition = cameraPosition + (cameraPosition.z()*0.02*sensitivity)*deltaX*left;
    update();
  }

  if (event->buttons() & Qt::RightButton) {

    QMatrix4x4 rotation;
    rotation.setToIdentity();

    QVector3D right = QVector3D::normal(cameraFront, cameraUpDirection);

    deltaX *= sensitivity;
    deltaY *= sensitivity;

    yaw += deltaX;
    pitch += deltaY;

    if(pitch > 89.9f)
        pitch = 89.9f;
    if(pitch < -89.9f)
        pitch = -89.9f;

    rotation.rotate(deltaY, right);
    rotation.rotate(-deltaX, 0, 0, 1);


    cameraPosition = rotation*tmp_cam_pos + pt_xy;
    cameraFront = rotation*cameraFront;
    cameraUpDirection = rotation*cameraUpDirection;

    update();
  }

  lastMousePosition = event->pos();
  emit mouseEvent();

  event->accept();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
  int delta = (float)event->delta();

  if (event->orientation() == Qt::Vertical) {
    if (delta < 0) {
       cameraPosition -= 0.3 * cameraFront;
    } else if (delta > 0) {
      cameraPosition += 0.3 * cameraFront;
    }
    update();
  }
  emit mouseEvent();

  event->accept();

}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
//   float cameraSpeed = 0.2f;
//   float adjust_height_rate = 0.1;
//   float adjust_search_bound_rate = 0.1;
  QVector3D z_fix_front = QVector3D(cameraUpDirection[0], cameraUpDirection[1], 0).normalized();
  if (event->key() == Qt::Key_1) {
      cout << "1 Key Pressed" << endl;

  }
  if (event->key() == Qt::Key_2) {
    cout << "2 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_3) {
    cout << "3 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_4) {
    cout << "4 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_5) {
    cout << "5 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_6) {
    cout << "6 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_7) {
    cout << "7 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_8) {
    cout << "8 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_9) {
    cout << "9 Key Pressed" << endl;
  }
  if (event->key() == Qt::Key_0) {
     cout << "0 Key Pressed" << endl;
  }


  if (event->key() == Qt::Key_R) {
     cout << "R Key Pressed" << endl;

  }


  if (event->key() == Qt::Key_W) {
    cout << "W Key Pressed" << endl;
        //cameraPosition += cameraSpeed * z_fix_front;
  }

  if (event->key() == Qt::Key_A) {
    cout << "A Key Pressed" << endl;

  }

  if (event->key() == Qt::Key_Q) {
    cout << "Q Key Pressed" << endl;
  }

  if (event->key() == Qt::Key_I) {
    cout << "I Key Pressed" << endl;

  }

  if (event->key() == Qt::Key_S) {
    cout << "S Key Pressed" << endl;


  }

  if (event->key() == Qt::Key_F) {
    cout << "F Key pressed " << endl;
    if(isFullScreen()) {
      this->setWindowState(Qt::WindowMaximized);
      cout << "Set normal screen size" << endl;
    } else {
      this->setWindowState(Qt::WindowFullScreen);
      cout << "Set FullScreen" << endl;
    }
    this->show();
  }

  if (event->key() == Qt::Key_U) {
    cout << "U Key pressed" << endl;

  }
  if (event->key() == Qt::Key_C) {
    cout << "C Key pressed" << endl;

  }


  if (event->key() == Qt::Key_D) {
    cout << "D Key pressed" << endl;

  }

  if (event->key() == Qt::Key_V) {
    cout << "V Key pressed" << endl;

  }
  update();
  event->accept();
}

void GLWidget::loadVehiclefile(){


  FILE *file = fopen("./src/labs_viewer/data/lexus_hs.obj","r");
  float scale = 0.08;
  if(file == NULL){
    printf("fail to read obj file\n");
    return;
  }
  bool flag = true;
  while(1){
    char lineHeader[128];
    int res = fscanf(file,"%s",lineHeader);
    if(res == EOF) break;

    if(strcmp(lineHeader,"v") == 0){
        QVector3D vertex;
        float data[3];
        fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
        vertex.setX(data[0]*scale);
        vertex.setY(data[1]*scale);
        vertex.setZ(data[2]*scale);

        vehicle_vertex.push_back(vertex);
    }else if(strcmp(lineHeader,"vn") == 0){
        QVector3D normal;
        float data[3];
        fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
        normal.setX(data[0]);
        normal.setY(data[1]);
        normal.setZ(data[2]);
        vehicle_normal.push_back(normal);

    }else if(strcmp(lineHeader,"vt") == 0){
        QVector2D texture;
        float data[3];
        fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
        texture.setX(data[0]);
        texture.setY(1-data[1]);
        //texture.setZ(data[2]);
        vehicle_texture.push_back(texture);

    }else if(strcmp(lineHeader,"f") == 0){
        QVector3D face;
        QVector3D texture;
        QVector3D normal;
        flag = true;
        int data[12];
        int return_v = fscanf(file,"%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n",&data[0],&data[1],&data[2]
            ,&data[3],&data[4],&data[5],&data[6],&data[7],&data[8],&data[9],&data[10],&data[11]);
        if(return_v == 9&&flag == true){
            face.setX(data[0]);
            face.setY(data[3]);
            face.setZ(data[6]);
            texture.setX(data[1]);
            texture.setY(data[4]);
            texture.setZ(data[7]);
            normal.setX(data[2]);
            normal.setY(data[5]);
            normal.setZ(data[8]);

            vehicle_face.push_back(face);
            vehicle_face_texture.push_back(texture);
            vehicle_face_normal.push_back(normal);
        }
        if(return_v == 12&&flag == true){
            face.setX(data[0]);
            face.setY(data[3]);
            face.setZ(data[6]);
            texture.setX(data[1]);
            texture.setY(data[4]);
            texture.setZ(data[7]);
            normal.setX(data[2]);
            normal.setY(data[5]);
            normal.setZ(data[8]);

            vehicle_face.push_back(face);
            vehicle_face_texture.push_back(texture);
            vehicle_face_normal.push_back(normal);
            face.setX(data[6]);
            face.setY(data[9]);
            face.setZ(data[0]);
            texture.setX(data[7]);
            texture.setY(data[10]);
            texture.setZ(data[1]);
            normal.setX(data[8]);
            normal.setY(data[11]);
            normal.setZ(data[2]);
            vehicle_face.push_back(face);
            vehicle_face_texture.push_back(texture);
            vehicle_face_normal.push_back(normal);
        }

    }else if(strcmp(lineHeader,"s") == 0){
      int value;
      int return_v = fscanf(file,"%d\n",&value);
      if(value == 1){
          flag = true;
      }else{
          flag = false;
      }

    }else{


    }
    //set vehicle node
    for(int i = 0 ; i < vehicle_face.size(); i ++){
      vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].x()-1]);
      vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].y()-1]);
      vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].z()-1]);
      vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].x()-1]);
      vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].y()-1]);
      vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].z()-1]);
      vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].x()-1]);
      vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].y()-1]);
      vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].z()-1]);

    }
  }

}



