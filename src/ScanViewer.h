#pragma once

#include "OGLMeshViewer.h"

class ScanViewer : public CMeshViewer
{
public:
  ScanViewer(QWidget* parent, int maxScans);
  ~ScanViewer();

  bool load(int scanID);
  bool loadDiff(int diffID);
  bool remove(int scanID);
  bool removeDiff(int diffID);
  bool show(int scanID);
  bool show(int scanID, int diffID);
  void clearDisplay();

  QList<int> currentScans() { return currentScans_; }
  bool isDisplayed(int scanID) { return currentScans_.contains(scanID); }
  CMesh* mesh(int scanID) { return scans_[scanID]; }

private:
  QMap<int, CMesh*> scans_;
  QMap<int, QVector<float>> diffs_;
  QList<int> currentScans_;

  void refreshDisplay();
  void debugNumScan();
};
