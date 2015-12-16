#pragma once

#include <QString>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <assimp/color4.h>

class CFace;
class CVertex;
struct UsedTypes : public vcg::UsedTypes<vcg::Use<CFace>::AsFaceType, vcg::Use<CVertex>::AsVertexType>{};
class CVertex : public vcg::Vertex<UsedTypes, vcg::vertex::Coord3d, vcg::vertex::Qualityf, vcg::vertex::Normal3d, vcg::vertex::Color4b, vcg::vertex::BitFlags> {};
class CFace : public vcg::Face<UsedTypes, vcg::face::VertexRef, vcg::face::Normal3d, vcg::face::EdgePlane, vcg::face::Color4b, vcg::face::Mark, vcg::face::BitFlags> {};
class CMesh : public vcg::tri::TriMesh<std::vector<CVertex>, std::vector<CFace>> {};

void openMesh(QString filename, CMesh& out, bool clean_data = false);
aiColor4D toOGLColor(vcg::Color4<unsigned char>& color);
void computeDifference(CMesh& reference, CMesh& mesh, CMesh& out);
void computeMirror(CMesh& reference, CMesh& mesh, CMesh& out);
void flipMeshXAxis(CMesh& mesh);
void flipMeshXAxis(CMesh& base, CMesh& out);
void retriveQualityFromMesh(CMesh& mesh, float*& quality);
void applyQualityToMesh(CMesh& mesh, float* quality);

void createDummyFile(QString filePath);
