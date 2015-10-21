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

class Viewer : public QGLViewer
{
  Q_OBJECT
public:
  Viewer();
  ~Viewer();

  bool addMesh(RecFusion::Mesh* mesh);
  bool addMeshFromFile(QString filename);
  bool removeMesh(RecFusion::Mesh* mesh);
protected :
  virtual void draw();
  virtual void init();
private:
  QVector<RecFusion::Mesh*> meshes_;
  ViewerData* data_;
};
