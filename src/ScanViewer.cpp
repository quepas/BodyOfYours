#include "ScanViewer.h"
#include "MeshProcessing.h"

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

bool ScanViewer::loadDiff(int diffID)
{
  if (diffs_.contains(diffID))
    return true;
  QSqlTableModel diffModel;
  diffModel.setTable("scan_diff");
  diffModel.setFilter(QString("id == %1").arg(diffID));
  diffModel.select();
  if (diffModel.rowCount() == 1) {
    QString diffFileName = diffModel.record(0).value("filename").toString();
    QVector<float> quality;
    if (!MeshProcessing::loadQualityMapFromFile<QVector<float>>(diffFileName, quality)) {
      qDebug() << "[ERROR@ScanViewer] Loading scan diff (quality map) failed" << diffID;
      return false;
    }
    diffs_.insert(diffID, quality);
    return true;
  }
  return false;
}

bool ScanViewer::removeDiff(int diffID)
{
  if (diffs_.contains(diffID)) {
    diffs_.remove(diffID);
    debugNumScan();
    return true;
  }
  return false;
}

void ScanViewer::debugNumScan()
{
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded scans:" << meshStorage_->meshes().size();
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded quality maps:" << diffs_.size();
}

bool ScanViewer::show(int scanID)
{
  clearDisplay();
  if (meshStorage_->hasMesh(scanID)) {
    currentScans_.append(scanID);
    refreshDisplay();
    return true;
  }
  return false;
}

bool ScanViewer::show(int scanID, int diffID)
{
  clearDisplay();
  if (meshStorage_->hasMesh(scanID) && diffs_.contains(diffID)) {
    currentScans_.append(scanID);
    applyQualityToMesh(*meshStorage_->mesh(scanID), diffs_[diffID]);
    refreshDisplay();
    return true;
  }
  return false;
}

void ScanViewer::refreshDisplay()
{
  clear();
  for (int id : currentScans_) {
    insert(meshStorage_->mesh(id));
  }
}

void ScanViewer::clearDisplay()
{
  currentScans_.clear();
}
