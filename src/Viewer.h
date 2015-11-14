#pragma once

#include <QGLViewer/qglviewer.h>
#include "RecFusion.h"
#include <QVector>
#include <vector>

class CMesh;

class Viewer : public QGLViewer
{
  Q_OBJECT
public:
  Viewer();
  ~Viewer();

  CMesh* cmesh_;

protected :
  virtual void initializeGL();
  virtual void draw();
  virtual void init();
};
