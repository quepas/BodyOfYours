#include "Viewer.h"
#include <assimp\postprocess.h> 
#include <assimp\mesh.h>
#include <assimp\scene.h>
#include <QDebug>

using namespace std;
using RecFusion::Mesh;

void Viewer::draw()
{
  if (data_ != nullptr) {
    camera()->setZClippingCoefficient(150.0f);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < data_->num_faces; ++i) {
      aiFace face = data_->faces[i];
      if (face.mNumIndices != 3) {
        std::cout << "Face " << i << " (" << face.mNumIndices << ")" << std::endl;
      }
      else {
        auto v1 = data_->verts[face.mIndices[0]];
        glColor3f(0.7f, 0.4f, 0.1f);
        auto n1 = data_->normals[face.mIndices[0]];
        glNormal3f(n1.x, n1.y, n1.z);
        glVertex3f(v1.x, v1.y, v1.z);
        auto v2 = data_->verts[face.mIndices[1]];
        auto n2 = data_->normals[face.mIndices[1]];
        glNormal3f(n2.x, n2.y, n2.z);
        glVertex3f(v2.x, v2.y, v2.z);
        auto v3 = data_->verts[face.mIndices[2]];
        auto n3 = data_->normals[face.mIndices[2]];
        glNormal3f(n3.x, n3.y, n3.z);
        glVertex3f(v3.x, v3.y, v3.z);
      }
    }
    glEnd();
  }
  else {
    // Draws a spiral
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

bool Viewer::addMeshFromFile(QString filename)
{
  Assimp::Importer model_importer;
  const aiScene* scene = (model_importer
    .ReadFile(
    filename.toStdString(),
    aiProcessPreset_TargetRealtime_Fast));

  if (!scene) {
    qDebug() << "Import failed. Unable to load " << filename;
    return false;
  }
  else {
    qDebug() << "Import success! Loaded " << filename;
    auto mesh = scene->mMeshes[0];
    data_ = new ViewerData;
    data_->num_verts = mesh->mNumVertices;
    data_->num_faces = mesh->mNumFaces;
    for (int i = 0; i < data_->num_verts; ++i) {
      data_->verts.push_back(mesh->mVertices[i]);
      data_->normals.push_back(mesh->mNormals[i]);
    }
    for (int i = 0; i < data_->num_faces; ++i) {
      data_->faces.push_back(mesh->mFaces[i]);
    }
    return true;
  }
}

Viewer::Viewer()
  : data_(nullptr)
{

}

Viewer::~Viewer()
{

}
