#include "ScanViewer.h"
#include "MeshProcessing.h"

#include <QDebug>
#include <QSqlRecord>
#include <QSqlTableModel>

ScanViewer::ScanViewer(QWidget* parent, int maxScans)
  : CMeshViewer(parent)
{

}

ScanViewer::~ScanViewer()
{
  scans_.clear();
  currentScans_.clear();
}

bool ScanViewer::load(int scanID)
{
  if (scans_.contains(scanID))
    return true;
  QSqlTableModel scanModel;
  scanModel.setTable("scan");
  scanModel.setFilter(QString("id == %1").arg(scanID));
  scanModel.select();
  if (scanModel.rowCount() == 1) {
    QString scanFileName = scanModel.record(0).value("filename").toString();
    CMesh* mesh = new CMesh;
    if (!openMesh(scanFileName, mesh)) {
      qDebug() << "[ERROR@ScanViewer] Loading scan failed" << scanID;
      return false;
    }
    scans_.insert(scanID, mesh);
    debugNumScan();
    return true;
  }
  return false;
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
    if (!loadQualityFromFile(diffFileName, quality)) {
      qDebug() << "[ERROR@ScanViewer] Loading scan diff (quality map) failed" << diffID;
      return false;
    }
    diffs_.insert(diffID, quality);
    return true;
  }
  return false;
}

bool ScanViewer::remove(int scanID)
{
  if (scans_.contains(scanID)) {
    delete scans_[scanID];
    scans_.remove(scanID);
    debugNumScan();
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
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded scans:" << scans_.size();
  qDebug() << "[DEBUG@ScanViewer] Num. of loaded quality maps:" << diffs_.size();
}

bool ScanViewer::show(int scanID)
{
  clearDisplay();
  if (scans_.contains(scanID)) {
    currentScans_.append(scanID);
    refreshDisplay();
    return true;
  }
  return false;
}

bool ScanViewer::show(int scanID, int diffID)
{
  clearDisplay();
  if (scans_.contains(scanID) && diffs_.contains(diffID)) {
    currentScans_.append(scanID);
    applyQualityToMesh(*scans_[scanID], diffs_[diffID]);
    refreshDisplay();
    return true;
  }
  return false;
}

void ScanViewer::refreshDisplay()
{
  clear();
  for (int id : currentScans_) {
    insert(scans_[id]);
  }
}

void ScanViewer::clearDisplay()
{
  currentScans_.clear();
}
