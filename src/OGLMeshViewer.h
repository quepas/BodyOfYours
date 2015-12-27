#pragma once

#include "MeshProcessing.h"
#include <QGLViewer/qglviewer.h>
#include <assimp\vector3.h>
#include "OGLDummy.h"

template<typename Mesh, typename Face>
class OGLMeshViewer;

typedef OGLMeshViewer<CMesh, CFace> CMeshViewer;

/*
 * Draw all inserted meshes
 */
template<typename Mesh, typename Face>
class OGLMeshViewer : public QGLViewer
{
public:
  OGLMeshViewer(QWidget* parent) : QGLViewer(parent) {};
  ~OGLMeshViewer() { clear(); }

  void insert(Mesh* mesh);
  virtual void clear();

  Mesh* last();

protected :
  virtual void initializeGL();
  virtual void draw();
  virtual void init() { restoreStateFromFile(); }

private:
  void drawFace(Face& face);
  void drawVertexFromFace(Face& face, int vertex_num);

  QList<Mesh*> meshes_;
};

//////////////////////////////////////////////////////////////////////////
// template implementation
//////////////////////////////////////////////////////////////////////////

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::initializeGL()
{
  QGLViewer::initializeGL();
  this->setSceneRadius(10000.0);
  this->toggleFPSIsDisplayed();
  //this->setAxisIsDrawn(true);
  this->setGridIsDrawn();
}

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::draw()
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

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::drawFace(Face& face)
{
  for (int i = 0; i < face.VN(); ++i) {
    drawVertexFromFace(face, i);
  }
}

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::drawVertexFromFace(Face& face, int vertex_num)
{
  auto point = face.V(vertex_num)->P();
  auto normal = face.V(vertex_num)->N();
  auto color = toOGLColor(face.V(vertex_num)->C());
  glColor3f(color.r, color.g, color.b);
  glNormal3f(normal.X(), normal.Y(), normal.Z());
  glVertex3f(point.X(), point.Y(), point.Z());
}

template<typename Mesh, typename Face>
Mesh* OGLMeshViewer<Mesh, Face>::last()
{
  if (meshes_.isEmpty()) return nullptr;
  return meshes_.last();
}

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::insert(Mesh* mesh)
{
  if (mesh) meshes_.append(mesh);
}

template<typename Mesh, typename Face>
void OGLMeshViewer<Mesh, Face>::clear()
{
  meshes_.clear();
  update();
}
