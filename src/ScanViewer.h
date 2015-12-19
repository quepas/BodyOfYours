#pragma once

#include "OGLMeshViewer.h"

class ScanViewer : public CMeshViewer
{
public:
  ScanViewer(QWidget* parent, int maxScans);
  ~ScanViewer();

  bool load(int scanID);
  bool remove(int scanID);
  bool show(int scanID);
  void clearDisplay();

  QList<int> currentScans() { return currentScans_; }
  bool isDisplayed(int scanID) { return currentScans_.contains(scanID); }
  CMesh* mesh(int scanID) { return scans_[scanID]; }

private:
  QMap<int, CMesh*> scans_;
  QList<int> currentScans_;

  void refreshDisplay();
  void debugNumScan();
};
