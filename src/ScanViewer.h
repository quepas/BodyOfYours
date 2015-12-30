#pragma once

#include "CMeshStorage.h"
#include "OGLMeshViewer.h"

class ScanViewer : public CMeshViewer
{
public:
  ScanViewer(const CMeshStorage* meshStorage, QWidget* parent, int maxScans);
  ~ScanViewer();

  bool show(int scanID);
  bool show(int scanID, int diffID);
  void clearDisplay();

  enum ID {
    FULL_VIEWER,
    MINI_VIEWER
  };

private:
  QMap<int, QVector<float>> diffs_;
  QMap<int, CMesh*> currentScans_;
  const CMeshStorage* meshStorage_;

  void refreshDisplay();
  void debugNumScan();
};
