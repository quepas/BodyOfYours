#pragma once

#include <QAction>
#include <QToolBar>

class PatientWidgetToolbar : public QToolBar
{
  Q_OBJECT
public:
  PatientWidgetToolbar(QWidget* parent = nullptr);
  ~PatientWidgetToolbar();

private:
  QAction* add_patient_;
  QAction* add_examination_;
  QAction* remove_item_;
  QAction* calculate_diff_;
  QAction* calculate_mirror_;

signals:
  void addNewPatient();
  void addNewExamination();
  void removeItem();
  void calculateDiff();
  void calculateMirror();
};
