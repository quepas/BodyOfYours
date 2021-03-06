#pragma once

#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <assimp/color4.h>

class CFace;
class CVertex;

struct UsedTypes : public vcg::UsedTypes<vcg::Use<CFace>::AsFaceType,
                                         vcg::Use<CVertex>::AsVertexType> {};

class CVertex : public vcg::Vertex<UsedTypes,
                                   vcg::vertex::Coord3d,
                                   vcg::vertex::Qualityf,
                                   vcg::vertex::Normal3d,
                                   vcg::vertex::Color4b,
                                   vcg::vertex::BitFlags> {};

class CFace : public vcg::Face<UsedTypes,
                               vcg::face::VertexRef,
                               vcg::face::Normal3d,
                               vcg::face::EdgePlane,
                               vcg::face::Color4b,
                               vcg::face::Mark,
                               vcg::face::BitFlags> {};

class CMesh : public vcg::tri::TriMesh<std::vector<CVertex>,
                                       std::vector<CFace>> {};
