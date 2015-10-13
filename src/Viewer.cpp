#include "Viewer.h"

using namespace std;
using RecFusion::Mesh;

// Draws a spiral
void Viewer::draw()
{
  if (meshes_.count() > 0) {
    Mesh* mesh = meshes_[0];
    glBegin(GL_TRIANGLES);
      for (unsigned i = 0; i < mesh->triangleCount(); ++i) {
        Mesh::Triangle tri = mesh->triangle(i);
        Mesh::Coordinate vert = mesh->vertex(tri.i1);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(vert.x, vert.y, vert.z);
        vert = mesh->vertex(tri.i2);
        glVertex3f(vert.x, vert.y, vert.z);
        vert = mesh->vertex(tri.i3);
        glVertex3f(vert.x, vert.y, vert.z);
      }
    glEnd();
  }
  else {
    const float nbSteps = 200.0;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i < nbSteps; ++i)
    {
      const float ratio = i / nbSteps;
      const float angle = 21.0*ratio;
      const float c = cos(angle);
      const float s = sin(angle);
      const float r1 = 1.0 - 0.8f*ratio;
      const float r2 = 0.8f - 0.8f*ratio;
      const float alt = ratio - 0.5f;
      const float nor = 0.5f;
      const float up = sqrt(1.0 - nor*nor);
      glColor3f(1.0 - ratio, 0.2f, ratio);
      glNormal3f(nor*c, up, nor*s);
      glVertex3f(r1*c, alt, r1*s);
      glVertex3f(r2*c, alt + 0.05f, r2*s);
    }
    glEnd();
  }
}

void Viewer::init()
{
  // Restore previous viewer state.
  restoreStateFromFile();
}

bool Viewer::addMesh(RecFusion::Mesh* mesh)
{
  if (meshes_.contains(mesh)) {
    return false;
  }
  else {
    meshes_.push_back(mesh);
    return true;
  }
}

bool Viewer::removeMesh(RecFusion::Mesh* mesh)
{
  if (meshes_.contains(mesh)) {
    meshes_.removeOne(mesh);
    return true;
  }
  else {
    return false;
  }
}
