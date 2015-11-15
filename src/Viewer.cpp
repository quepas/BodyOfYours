#include "Viewer.h"
#include "MeshProcessing.h"

#include <assimp\vector3.h>
#include <QDebug>

using namespace std;
using RecFusion::Mesh;

void Viewer::draw()
{
  if (!mesh_map_.isEmpty()) {
    for (auto mesh : mesh_map_) {
      glBegin(GL_TRIANGLES);
      for (int i = 0; i < mesh->FN(); ++i) {
        auto face = mesh->face[i];
        drawFace(face);
      }
      glEnd();
    }
  }
}

void Viewer::init()
{
  // Restore previous viewer state.
  restoreStateFromFile();
}

Viewer::Viewer()
{

}

Viewer::~Viewer()
{

}

void Viewer::initializeGL()
{
  QGLViewer::initializeGL();
  this->setSceneRadius(10000.0);
  this->toggleFPSIsDisplayed();
}

void Viewer::drawFace(CFace& face)
{
  for (int i = 0; i < face.VN(); ++i) {
    drawVertexFromFace(face, i);
  }
}

void Viewer::drawVertexFromFace(CFace& face, int vertex_num)
{
  auto point = face.V(vertex_num)->P();
  auto normal = face.V(vertex_num)->N();
  auto color = toOGLColor(face.V(vertex_num)->C());
  glColor3f(color.r, color.g, color.b);
  glNormal3f(normal.X(), normal.Y(), normal.Z());
  glVertex3f(point.X(), point.Y(), point.Z());
}

void Viewer::addMesh(QString name, CMesh* mesh)
{
  if (mesh_map_.contains(name)) {
    qDebug() << "[WARNING] Render map already contains mesh: " << name
             << "\n\tInserting mesh anyway.";
  }
  mesh_map_.insert(name, mesh);
}

void Viewer::removeMesh(QString name)
{
  if (mesh_map_.remove(name) == 0) {
    qDebug() << "[WARNING] Render map doesn't contain mesh: " << name;
  }
}

void Viewer::clearMesh()
{
  mesh_map_.clear();
}
