#include "Viewer.h"
#include "MeshProcessing.h"

#include <assimp\vector3.h>
#include <QDebug>

using namespace std;
using RecFusion::Mesh;

void Viewer::draw()
{
  if (cmesh_ != nullptr) {
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < cmesh_->FN(); ++i) {
      auto face = cmesh_->face[i];
      drawFace(face);
    }
    glEnd();
  }
}

void Viewer::init()
{
  // Restore previous viewer state.
  restoreStateFromFile();
}

Viewer::Viewer()
  : cmesh_(nullptr)
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
