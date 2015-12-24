#pragma once

#include "MeshDef.h"
#include <QString>

bool openMesh(QString filename, CMesh* out, bool clean_data = false);
bool openMesh(QString filename, CMesh& out, bool clean_data = false);
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
