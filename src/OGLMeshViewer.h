#pragma once

#include "MeshProcessing.h"
#include <QGLViewer/qglviewer.h>
#include <assimp\vector3.h>
#include "OGLDummy.h"

template<typename MeshT, typename FaceT>
class OGLMeshViewer;

typedef OGLMeshViewer<CMesh, CFace> CMeshViewer;

/*
 * Draw all inserted meshes
 */
template<typename MeshT, typename FaceT>
class OGLMeshViewer : public QGLViewer
{
public:
  OGLMeshViewer(QWidget* parent) : QGLViewer(parent) {};
  ~OGLMeshViewer() { clear(); }

  void insert(MeshT* mesh);
  virtual void clear();

  MeshT* last();

protected :
  virtual void initializeGL();
  virtual void draw();
  virtual void init() { restoreStateFromFile(); }

private:
  void drawFace(FaceT& face);
  void drawVertexFromFace(FaceT& face, int vertex_num);

  QList<MeshT*> meshes_;
};

//////////////////////////////////////////////////////////////////////////
// template implementation
//////////////////////////////////////////////////////////////////////////

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::initializeGL()
{
  QGLViewer::initializeGL();
  this->setSceneRadius(10000.0);
  this->toggleFPSIsDisplayed();
  //this->setAxisIsDrawn(true);
  this->setGridIsDrawn();
}

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::draw()
{
  if (!meshes_.isEmpty()) {
    for (auto mesh : meshes_) {
      glBegin(GL_TRIANGLES);
      for (int i = 0; i < mesh->FN(); ++i) {
        auto face = mesh->face[i];
        drawFace(face);
      }
      glEnd();
    }
  }
}

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::drawFace(FaceT& face)
{
  for (int i = 0; i < face.VN(); ++i) {
    drawVertexFromFace(face, i);
  }
}

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::drawVertexFromFace(FaceT& face, int vertex_num)
{
  auto point = face.V(vertex_num)->P();
  auto normal = face.V(vertex_num)->N();
  auto color = toOGLColor(face.V(vertex_num)->C());
  glColor3f(color.r, color.g, color.b);
  glNormal3f(normal.X(), normal.Y(), normal.Z());
  glVertex3f(point.X(), point.Y(), point.Z());
}

template<typename MeshT, typename FaceT>
MeshT* OGLMeshViewer<MeshT, FaceT>::last()
{
  if (meshes_.isEmpty()) return nullptr;
  return meshes_.last();
}

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::insert(MeshT* mesh)
{
  if (mesh) meshes_.append(mesh);
}

template<typename MeshT, typename FaceT>
void OGLMeshViewer<MeshT, FaceT>::clear()
{
  meshes_.clear();
  update();
}
