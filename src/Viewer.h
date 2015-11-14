#pragma once

#include <QGLViewer/qglviewer.h>
#include "RecFusion.h"
#include <QVector>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/mesh.h>

struct ViewerData
{
  int num_verts;
  int num_faces;
  std::vector<aiVector3D> verts;
  std::vector<aiFace> faces;
  std::vector<aiVector3D> normals;
};

class CMesh;

class Viewer : public QGLViewer
{
  Q_OBJECT
public:
  Viewer();
  ~Viewer();

  bool addMeshFromFile(QString filename);
  bool addMeshFromCMesh(QString filename);
  CMesh* cmesh_;
  bool use_cmesh_;
protected :
  virtual void initializeGL();
  virtual void draw();
  virtual void init();
private:
  ViewerData* data_;
};
