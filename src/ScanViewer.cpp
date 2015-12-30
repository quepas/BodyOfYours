#include "ScanViewer.h"
#include "MeshProcessing.h"

#include <algorithm>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <vcg/complex/append.h>
#include <QDebug>
#include <QSqlRecord>
#include <QSqlTableModel>

ScanViewer::ScanViewer(const CMeshStorage* meshStorage, QWidget* parent, int maxScans)
  : CMeshViewer(parent), meshStorage_(meshStorage)
{

}

ScanViewer::~ScanViewer()
{
  currentScans_.clear();
}

void ScanViewer::debugNumScan()
{
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded scans:" << meshStorage_->meshes().size();
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded quality maps:" << diffs_.size();
}

bool ScanViewer::show(int scanId)
{
  clearDisplay();
  if (meshStorage_->hasMesh(scanId)) {
    currentScans_.insert(scanId, meshStorage_->mesh(scanId));
    refreshDisplay();
    return true;
  }
  return false;
}

bool ScanViewer::show(int scanID, int diffID)
{
  clearDisplay();
  if (meshStorage_->hasMesh(scanID) && meshStorage_->hasMesh(diffID)) {
    auto clone = meshStorage_->meshClone(scanID);
    applyQualityToMesh(*clone, meshStorage_->qualityMap(diffID));
    currentScans_.insert(scanID, clone);
    refreshDisplay();
    return true;
  }
  return false;
}

void ScanViewer::refreshDisplay()
{
  clear();
  std::for_each(currentScans_.begin(), currentScans_.end(), [=](CMesh* mesh) { this->insert(mesh); });
  update();
}

void ScanViewer::clearDisplay()
{
  currentScans_.clear();
}
