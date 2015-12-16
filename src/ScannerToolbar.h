#pragma once

#include "PatientTreeWidget.h"
#include <QAction>
#include <QToolBar>

class ScannerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScannerToolbar(PatientTreeWidget* patient_tree, QWidget* parent = nullptr);
  ~ScannerToolbar();

  void setEnabled(bool enabled);

private:
  QAction* start_recon_;
  QAction* stop_recon_;
};
