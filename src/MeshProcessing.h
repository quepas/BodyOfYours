#pragma once

#include "MeshDef.h"

#include <wrap/io_trimesh/import.h>
#include <vcg/complex/algorithms/update/component_ep.h>

#include <QString>

namespace MeshProcessing {
// #MeshProcessing
template <typename MeshT>
bool loadMeshFromFile(QString filename, MeshT* outMesh, bool cleanData = false);
// ~MeshProcessing
}

aiColor4D toOGLColor(vcg::Color4<unsigned char>& color);
void computeDifference(CMesh& reference, CMesh& mesh, CMesh& out);
void computeMirror(CMesh& reference, CMesh& mesh, CMesh& out);
void flipMeshXAxis(CMesh& mesh);
void flipMeshXAxis(CMesh& base, CMesh& out);
void retriveQualityFromMesh(CMesh* mesh, QVector<float>& quality);
void applyQualityToMesh(CMesh& mesh, const QVector<float>& quality);

void generateRandomQualityForMesh(const CMesh& mesh, QVector<float>& qualityOut);
void saveQualityToFile(QString filePath, const QVector<float>& quality);
bool loadQualityFromFile(QString filePath, QVector<float>& quality);

void createDummyFile(QString filePath);

template<typename T>
QString toString(vcg::Point3<T> point) {
  return QString("(%1, %2, %3)").arg(point.X()).arg(point.Y()).arg(point.Z());
}

namespace MeshProcessing {
// #MeshProcessing
template <typename MeshT>
bool loadMeshFromFile(QString filename, MeshT* outMesh, bool cleanData /*= false*/)
{
  int err = vcg::tri::io::Importer<MeshT>::Open(*outMesh, qPrintable(filename));
  if (err) {
    qDebug() << "[ERROR@MeshProcessing] Error in reading" << filename << ":" << vcg::tri::io::Importer<MeshT>::ErrorMsg(err);
    if (vcg::tri::io::Importer<MeshT>::ErrorCritical(err)) qDebug() << "[CRIT@MeshProcessing] It is a very serious error.";
    delete outMesh;
    return false;
  }
  vcg::tri::UpdateNormal<MeshT>::PerVertexNormalized(*outMesh);
  qDebug() << "[INFO@MeshProcessing] Successfully read mesh" << filename;
  if (cleanData) {
    int numDuplicated = vcg::tri::Clean<MeshT>::RemoveDuplicateVertex(*outMesh);
    int numUnref = vcg::tri::Clean<MeshT>::RemoveUnreferencedVertex(*outMesh);
    qDebug() << "Removed" << numDuplicated << "duplicate and" << numUnref << "unreferenced vertices from mesh" << filename;
  }
  return true;
}
// ~MeshProcessing
}
