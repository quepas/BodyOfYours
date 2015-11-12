#include "MeshProcessing.h"

#include <QDebug>
#include <wrap/io_trimesh/import.h>

using vcg::tri::io::Importer;
using vcg::tri::Clean;

void openMesh(QString filename, CMesh& out, bool clean_data /*= false*/)
{
  int err = Importer<CMesh>::Open(out, qPrintable(filename));
  if (err) {
    qDebug() << "[Error] Error in reading " << filename << ":" << Importer<CMesh>::ErrorMsg(err);
    if (Importer<CMesh>::ErrorCritical(err))
      qDebug() << "[Critical] Serious error.";
  }
  UpdateNormal<CMesh>::PerVertexNormalized(out);
  qDebug() << "[Info] Read mesh " << filename;
  if (clean_data){
    int dup = Clean<CMesh>::RemoveDuplicateVertex(out);
    int unref = Clean<CMesh>::RemoveUnreferencedVertex(out);
    qDebug() << "Removed " << dup << " duplicate and " << unref << " unreferenced vertices from mesh " << filename;
  }
}
