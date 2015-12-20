#pragma once

#include "ScanViewer.h"
#include "PatientTreeWidget.h"
#include <QAction>
#include <QToolBar>

class PatientTreeToolbar : public QToolBar
{
  Q_OBJECT
public:
  PatientTreeToolbar(PatientTreeWidget* patient_widget, QWidget* parent = nullptr);
  ~PatientTreeToolbar();

private:
  QAction* add_patient_;
  QAction* add_examination_;
  QAction* remove_item_;
  QAction* calculate_diff_;
  QAction* calculate_mirror_;
  QAction* showScan_;

signals:
  void calculateDiff();
  void calculateMirror();
  void displayScan(ScanViewer::ID viwerID, int id);
};
