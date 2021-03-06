#include "MeshProcessing.h"

#include <QDebug>
#include <QFile>
#include <vcg/complex/algorithms/update/bounding.h>
#include "sampling.h"
#include <random>

using vcg::tri::Append;
using vcg::tri::Clean;
using vcg::tri::UpdatePosition;
using vcg::tri::io::Importer;
using vcg::Matrix44d;

aiColor4D toOGLColor(vcg::Color4<unsigned char>& color)
{
  return aiColor4D(color.X() / 255.0f,
                   color.Y() / 255.0f,
                   color.Z() / 255.0f,
                   color.W() / 255.0f);
}

void computeDifference(CMesh& reference, CMesh& mesh, CMesh& out)
{
  int flags = 
    vcg::SamplingFlags::VERTEX_SAMPLING |
    vcg::SamplingFlags::EDGE_SAMPLING |
    vcg::SamplingFlags::FACE_SAMPLING |
    vcg::SamplingFlags::SIMILAR_SAMPLING;
  flags |= vcg::SamplingFlags::SAVE_ERROR;
  if (!(flags & vcg::SamplingFlags::USE_HASH_GRID) && !(flags & vcg::SamplingFlags::USE_AABB_TREE) && !(flags & vcg::SamplingFlags::USE_OCTREE))
    flags |= vcg::SamplingFlags::USE_STATIC_GRID;
  // initialize time info.
  int elapsed_time;
  int t0 = clock();
  // compute face information
  UpdateComponentEP<CMesh>::Set(reference);
  UpdateComponentEP<CMesh>::Set(mesh);

  // set bounding boxes for reference and mesh
  UpdateBounding<CMesh>::Box(reference);
  UpdateBounding<CMesh>::Box(mesh);

  // set Bounding Box.
  vcg::Box3<CMesh::ScalarType> bbox, tmp_bbox_M1 = reference.bbox, tmp_bbox_M2 = mesh.bbox;
  bbox.Add(reference.bbox);
  bbox.Add(mesh.bbox);
  bbox.Offset(bbox.Diag()*0.02);
  reference.bbox = bbox;
  mesh.bbox = bbox;

  vcg::Sampling<CMesh> ForwardSampling(reference, mesh);
  vcg::Sampling<CMesh> BackwardSampling(mesh, reference);

  // print mesh info.
  printf("Mesh info:\n");
  printf(" M1: '%s'\n\tvertices  %7i\n\tfaces     %7i\n\tarea      %12.4f\n", "ref mesh", reference.vn, reference.fn, ForwardSampling.GetArea());
  printf("\tbbox (%7.4f %7.4f %7.4f)-(%7.4f %7.4f %7.4f)\n", tmp_bbox_M1.min[0], tmp_bbox_M1.min[1], tmp_bbox_M1.min[2], tmp_bbox_M1.max[0], tmp_bbox_M1.max[1], tmp_bbox_M1.max[2]);
  printf("\tbbox diagonal %f\n", (float) tmp_bbox_M1.Diag());
  printf(" M2: '%s'\n\tvertices  %7i\n\tfaces     %7i\n\tarea      %12.4f\n", "mesh mesh", mesh.vn, mesh.fn, BackwardSampling.GetArea());
  printf("\tbbox (%7.4f %7.4f %7.4f)-(%7.4f %7.4f %7.4f)\n", tmp_bbox_M2.min[0], tmp_bbox_M2.min[1], tmp_bbox_M2.min[2], tmp_bbox_M2.max[0], tmp_bbox_M2.max[1], tmp_bbox_M2.max[2]);
  printf("\tbbox diagonal %f\n", (float) tmp_bbox_M2.Diag());

  unsigned long n_samples_target = 10* max(reference.fn, mesh.fn);// take 10 samples per face
  bool NumberOfSamples = true;
  double n_samples_per_area_unit = 1;
  double dist1_max, dist2_max;

  // Forward distance.
  printf("\nForward distance (M1 -> M2):\n");
  ForwardSampling.SetFlags(flags);
  if (NumberOfSamples)
  {
    ForwardSampling.SetSamplesTarget(n_samples_target);
    n_samples_per_area_unit = ForwardSampling.GetNSamplesPerAreaUnit();
  }
  else
  {
    ForwardSampling.SetSamplesPerAreaUnit(n_samples_per_area_unit);
    n_samples_target = ForwardSampling.GetNSamplesTarget();
  }
  printf("target # samples      : %lu\ntarget # samples/area : %f\n", n_samples_target, n_samples_per_area_unit);
  ForwardSampling.Hausdorff();
  dist1_max = ForwardSampling.GetDistMax();
  printf("\ndistances:\n  max  : %f (%f  wrt bounding box diagonal)\n", (float) dist1_max, (float) dist1_max / bbox.Diag());
  printf("  mean : %f\n", ForwardSampling.GetDistMean());
  printf("  RMS  : %f\n", ForwardSampling.GetDistRMS());
  printf("# vertex samples %9lu\n", ForwardSampling.GetNVertexSamples());
  printf("# edge samples   %9lu\n", ForwardSampling.GetNEdgeSamples());
  printf("# area samples   %9lu\n", ForwardSampling.GetNAreaSamples());
  printf("# total samples  %9lu\n", ForwardSampling.GetNSamples());
  printf("# samples per area unit: %f\n\n", ForwardSampling.GetNSamplesPerAreaUnit());

  // Backward distance.
  printf("\nBackward distance (M2 -> M1):\n");
  BackwardSampling.SetFlags(flags);
  if (NumberOfSamples)
  {
    BackwardSampling.SetSamplesTarget(n_samples_target);
    n_samples_per_area_unit = BackwardSampling.GetNSamplesPerAreaUnit();
  }
  else
  {
    BackwardSampling.SetSamplesPerAreaUnit(n_samples_per_area_unit);
    n_samples_target = BackwardSampling.GetNSamplesTarget();
  }
  printf("target # samples      : %lu\ntarget # samples/area : %f\n", n_samples_target, n_samples_per_area_unit);
  BackwardSampling.Hausdorff();
  dist2_max = BackwardSampling.GetDistMax();
  printf("\ndistances:\n  max  : %f (%f  wrt bounding box diagonal)\n", (float) dist2_max, (float) dist2_max / bbox.Diag());
  printf("  mean : %f\n", BackwardSampling.GetDistMean());
  printf("  RMS  : %f\n", BackwardSampling.GetDistRMS());
  printf("# vertex samples %9lu\n", BackwardSampling.GetNVertexSamples());
  printf("# edge samples   %9lu\n", BackwardSampling.GetNEdgeSamples());
  printf("# area samples   %9lu\n", BackwardSampling.GetNAreaSamples());
  printf("# total samples  %9lu\n", BackwardSampling.GetNSamples());
  printf("# samples per area unit: %f\n\n", BackwardSampling.GetNSamplesPerAreaUnit());

  elapsed_time = clock() - t0;
  qDebug() << "[INFO] Computing meshes difference took time: " << elapsed_time;

  double mesh_dist_max = max(dist1_max, dist2_max);
  int n_total_sample = ForwardSampling.GetNSamples() + BackwardSampling.GetNSamples();
  printf("\nHausdorff distance: %f (%f  wrt bounding box diagonal)\n", (float) mesh_dist_max, (float) mesh_dist_max / bbox.Diag());
  printf("  Computation time  : %d ms\n", (int) (1000.0*elapsed_time / CLOCKS_PER_SEC));
  printf("  # samples/second  : %f\n\n", (float) n_total_sample / ((float) elapsed_time / CLOCKS_PER_SEC));
  float ColorMin = 1.0f, ColorMax = 1.0f;
  vcg::tri::UpdateColor<CMesh>::PerVertexQualityRamp(reference, ColorMin, ColorMax);
  vcg::tri::UpdateColor<CMesh>::PerVertexQualityRamp(mesh, ColorMin, ColorMax);
}

void computeMirror(CMesh& reference, CMesh& mesh, CMesh& out)
{

}

void flipMeshXAxis(CMesh& base, CMesh& out)
{
  Append<CMesh, CMesh>::MeshCopy(out, base);
  flipMeshXAxis(out);
}

void flipMeshXAxis(CMesh& mesh)
{
  Matrix44d transform;
  transform.SetIdentity();
  transform[0][0] = -1.0;
  UpdatePosition<CMesh>::Matrix(mesh, transform, false);
}

void retriveQualityFromMesh(CMesh* mesh, QVector<float>& quality)
{
  int numVert = mesh->vert.size();
  qDebug() << "Num. of vertex with quality: " << numVert;
  for (CMesh::VertexIterator vi = mesh->vert.begin(); vi != mesh->vert.end(); ++vi) {
    quality.push_back(vi->Q());
  }
}

void applyQualityToMesh(CMesh& mesh, const QVector<float>& quality)
{
  assert(quality.size() == mesh.vert.size());
  int counter = 0;
  for (CMesh::VertexIterator vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi) {
    vi->Q() = quality[counter++];
  }
  vcg::tri::UpdateColor<CMesh>::PerVertexQualityRamp(mesh);
}

void createDummyFile(QString filePath)
{
  QFile file(filePath);
  if (file.open(QIODevice::ReadWrite))
  {
    QTextStream stream(&file);
    stream << "dummy" << endl;
  }
}

void generateRandomQualityForMesh(const CMesh& mesh, QVector<float>& qualityOut)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  int vertNum = mesh.vert.size();
  for (int i = 0; i < vertNum; ++i) {
    qualityOut.push_back(std::generate_canonical<float, 10>(gen));
  }
}

void saveQualityToFile(QString filePath, const QVector<float>& quality)
{
  QFile file(filePath);
  file.open(QIODevice::WriteOnly);
  QDataStream out(&file);
  out << quality;
}

bool loadQualityFromFile(QString filePath, QVector<float>& quality)
{
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) return false;
  QDataStream in(&file);
  in >> quality;
  return true;
}
