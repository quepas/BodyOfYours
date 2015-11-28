#pragma once

#include <QGLViewer/qglviewer.h>
#include "RecFusion.h"
#include <QVector>
#include <vector>

class CMesh;
class CFace;

class MeshViewer : public QGLViewer
{
  Q_OBJECT
public:
  MeshViewer();
  ~MeshViewer();

  void addMesh(QString name, CMesh* mesh);
  void removeMesh(QString name);
  void clearMesh();
  CMesh* getLastMesh();

protected :
  virtual void initializeGL();
  virtual void draw();
  virtual void init();

private:
  void drawFace(CFace& face);
  void drawVertexFromFace(CFace& face, int vertex_num);

  QMap<QString, CMesh*> mesh_map_;
};
