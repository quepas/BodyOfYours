#include "CMeshStorage.h"
#include "MeshProcessing.h"

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
  if (!MeshProcessing::loadMeshFromFile<CMesh>(filename, mesh, cleanData)) {
    qDebug() << "[ERROR@CMeshStorage] Failure adding mesh to MeshStorage" << filename;
    return false;
  }
  meshes_.insert(key, mesh);
  return true;
}

bool CMeshStorage::loadQualityMap(QString filename, int key)
{
  QVector<float> quality;
  if (!MeshProcessing::loadQualityMapFromFile<QVector<float>>(filename, quality)) {
    qDebug() << "[ERROR@CMeshStorage] Failure adding quality map to MeshStorage" << filename;
    return false;
  }
  qualityMaps_.insert(key, quality);
  return true;
}

CMeshStorage::~CMeshStorage()
{
  deleteAll();
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

QVector<float> CMeshStorage::qualityMap(int key) const
{
  return qualityMaps_.data(key);
}
