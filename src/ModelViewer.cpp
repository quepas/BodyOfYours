#include "ModelViewer.h"

ModelViewer::ModelViewer(QWidget *parent) : QOpenGLWidget(parent)
{
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  setFormat(format);
}

void ModelViewer::initializeGL()
{
  initializeOpenGLFunctions();
}

void ModelViewer::resizeGL(int w, int h)
{
  m_projection.setToIdentity();
  m_projection.perspective(60.0f, w / float(h), 0.01f, 1000.0f);
}

void ModelViewer::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  GLfloat vertices [] = {
    0.0f, 0.707f,
    -0.5f, -0.5f,
    0.5f, -0.5f
  };

  GLfloat colors [] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  };
  
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, vertices);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, colors);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}