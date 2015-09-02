#pragma once

#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QMatrix4x4>

class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
  ModelViewer(QWidget *parent = 0);
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

private:
  QMatrix4x4 m_projection;
};
