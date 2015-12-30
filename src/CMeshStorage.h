#pragma once

#include "LimitedDataStorage.h"
#include "MeshDef.h"

#include <map>
#include <memory>
#include <QString>

struct DeleteMeshPtrStrategy { static void onDestroy(CMesh* mesh) { delete mesh; }};
struct DoNothingStrategy { static void onDestroy(QVector<float>) {}};

typedef LimitedDataStorage<int, CMesh*, DeleteMeshPtrStrategy> Meshes;
typedef LimitedDataStorage<int, QVector<float>, DoNothingStrategy> QualityMaps;

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

  const Meshes& meshes() const { return meshes_; }
  const QualityMaps& qualityMaps() const { return qualityMaps_; }

  void deleteAll();

private:
  Meshes meshes_;
  QualityMaps qualityMaps_;
};
