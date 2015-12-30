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
  clearDisplay();
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
    currentScans_.append({ scanId, meshStorage_->mesh(scanId), false });
    refreshDisplay();
    return true;
  }
  return false;
}

bool ScanViewer::show(int scanID, int diffID)
{
  clearDisplay();
  if (meshStorage_->hasMesh(scanID) && meshStorage_->hasQualityMap(diffID)) {
    auto clone = meshStorage_->meshClone(scanID);
    applyQualityToMesh(*clone, meshStorage_->qualityMap(diffID));
    currentScans_.append({ scanID, clone, true });
    refreshDisplay();
    return true;
  }
  return false;
}

void ScanViewer::refreshDisplay()
{
  clear();
  std::for_each(currentScans_.begin(), currentScans_.end(), [=](Scan scan) { this->insert(scan.mesh); });
  update();
}

void ScanViewer::clearDisplay()
{
  std::for_each(currentScans_.begin(), currentScans_.end(),
    [=](Scan scan) {
    if (scan.isClone) delete scan.mesh;
  });
  currentScans_.clear();
}
