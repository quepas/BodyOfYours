#pragma once

#include "PatientTreeWidget.h"
#include <QAction>
#include <QToolBar>

class PatientWidgetToolbar : public QToolBar
{
  Q_OBJECT
public:
  PatientWidgetToolbar(PatientTreeWidget* patient_widget, QWidget* parent = nullptr);
  ~PatientWidgetToolbar();

private:
  QAction* add_patient_;
  QAction* add_examination_;
  QAction* remove_item_;
  QAction* calculate_diff_;
  QAction* calculate_mirror_;
  QAction* show_scan_;

signals:
  void calculateDiff();
  void calculateMirror();
};
