#pragma once

#include <QGLViewer/qglviewer.h>
#include "RecFusion.h"
#include <QVector>
#include <vector>

class CMesh;
class CFace;

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

private:
  void drawFace(CFace& face);
  void drawVertexFromFace(CFace& face, int vertex_num);
};
