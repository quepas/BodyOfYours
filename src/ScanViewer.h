#pragma once

#include "CMeshStorage.h"
#include "OGLMeshViewer.h"

class ScanViewer : public CMeshViewer
{
public:
  ScanViewer(const CMeshStorage* meshStorage, QWidget* parent, int maxScans);
  ~ScanViewer();

  bool loadDiff(int diffID);
  bool removeDiff(int diffID);
  bool show(int scanID);
  bool show(int scanID, int diffID);
  void clearDisplay();

  QList<int> currentScans() { return currentScans_; }
  bool isDisplayed(int scanID) { return currentScans_.contains(scanID); }

  enum ID {
    FULL_VIEWER,
    MINI_VIEWER
  };

private:
  QMap<int, QVector<float>> diffs_;
  QList<int> currentScans_;
  const CMeshStorage* meshStorage_;

  void refreshDisplay();
  void debugNumScan();
};
