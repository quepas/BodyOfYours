#include "CMeshStorage.h"

// VCGLib
#include <wrap/io_trimesh/import.h>

// QT 
#include <QDebug>
#include <QFile>

using vcg::tri::Append;
using vcg::tri::io::Importer;

CMeshStorage::CMeshStorage()
  : meshes_(10), qualityMaps_(10)
{

}

bool CMeshStorage::loadMesh(QString filename, int key, bool cleanData /*= true*/)
{
  CMesh* mesh = new CMesh();
  int err = Importer<CMesh>::Open(*mesh, qPrintable(filename));
  if (err) {
    qDebug() << "[ERROR@CMeshStorage] Error in reading" << filename << ":" << Importer<CMesh>::ErrorMsg(err);
    if (Importer<CMesh>::ErrorCritical(err)) qDebug() << "[CRIT@CMeshStorage] It is a very serious error.";
    delete mesh;
    return false;
  }
  UpdateNormal<CMesh>::PerVertexNormalized(*mesh);
  qDebug() << "[INFO@CMeshStorage] Successfully read mesh" << filename;
  if (cleanData) {
    int numDuplicated = Clean<CMesh>::RemoveDuplicateVertex(*mesh);
    int numUnref = Clean<CMesh>::RemoveUnreferencedVertex(*mesh);
    qDebug() << "Removed" << numDuplicated << "duplicate and" << numUnref << "unreferenced vertices from mesh" << filename;
  }
  meshes_.insert(key, mesh);
  return true;
}

CMeshStorage::~CMeshStorage()
{
  deleteAll();
}

bool CMeshStorage::loadQualityMap(QString filename, int key)
{
  QVector<float> quality;
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) return false;
  QDataStream in(&file);
  in >> quality;
  qualityMaps_.insert(key, quality);
  return true;
}

CMesh* CMeshStorage::mesh(int key) const
{
  if (!meshes_.contains(key)) return nullptr;
  return meshes_.data(key);
}

CMesh* CMeshStorage::meshClone(int key) const
{
  if (!meshes_.contains(key)) return nullptr;
  CMesh* outMesh = new CMesh;
  CMesh* mesh = meshes_.data(key);
  Append<CMesh, CMesh>::MeshCopy(*outMesh, *mesh);
  return outMesh;
}

void CMeshStorage::deleteAll()
{
  for (auto key : meshes_.keys()) {
    delete meshes_.data(key);
  }
}

bool CMeshStorage::hasMesh(int key) const
{
  return meshes_.contains(key);
}

bool CMeshStorage::hasQualityMap(int key) const
{
  return meshes_.contains(key);
}
