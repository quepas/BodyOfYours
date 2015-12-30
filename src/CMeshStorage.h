#pragma once

#include "LimitedDataStorage.h"
#include "MeshDef.h"

#include <map>
#include <memory>
#include <QString>

/*
 * WARNING: mesh or quality map with negative key are consider as 'helper entities'
 */
class CMeshStorage
{
public:
  CMeshStorage();
  ~CMeshStorage();

  bool loadMesh(QString filename, int key, bool cleanData = true);
  bool loadQualityMap(QString filename, int key);

  CMesh* mesh(int key) const;
  CMesh* meshClone(int key) const;

  bool hasMesh(int key) const;
  bool hasQualityMap(int key) const;

  QVector<float> qualityMap(int key) const;

  const LimitedDataStorage<int, CMesh*>& meshes() const { return meshes_; }
  const LimitedDataStorage<int, QVector<float>>& qualityMaps() const { return qualityMaps_; }

  void deleteAll();

private:
  LimitedDataStorage<int, CMesh*> meshes_;
  LimitedDataStorage<int, QVector<float>> qualityMaps_;
};
