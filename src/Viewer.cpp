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
      auto v1 = aiVector3D(face.V(0)->P().X(), face.V(0)->P().Y(), face.V(0)->P().Z());
      auto n1 = aiVector3D(face.V(0)->N().X(), face.V(0)->N().Y(), face.V(0)->N().Z());
      auto c1 = toOGLColor(face.V(0)->C());
      glColor3f(c1.r, c1.g, c1.b);
      glNormal3f(n1.x, n1.y, n1.z);
      glVertex3f(v1.x, v1.y, v1.z);
      auto v2 = aiVector3D(face.V(1)->P().X(), face.V(1)->P().Y(), face.V(1)->P().Z());
      auto n2 = aiVector3D(face.V(1)->N().X(), face.V(1)->N().Y(), face.V(1)->N().Z());
      auto c2 = toOGLColor(face.V(1)->C());
      glColor3f(c2.r, c2.g, c2.b);
      glNormal3f(n2.x, n2.y, n2.z);
      glVertex3f(v2.x, v2.y, v2.z);
      auto v3 = aiVector3D(face.V(2)->P().X(), face.V(2)->P().Y(), face.V(2)->P().Z());
      auto n3 = aiVector3D(face.V(2)->N().X(), face.V(2)->N().Y(), face.V(2)->N().Z());
      auto c3 = toOGLColor(face.V(2)->C());
      glColor3f(c3.r, c3.g, c3.b);
      glNormal3f(n3.x, n3.y, n3.z);
      glVertex3f(v3.x, v3.y, v3.z);
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
