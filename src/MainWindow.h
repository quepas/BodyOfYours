#pragma once

#include <QMainWindow>
#include <QStackedLayout>
#include <QGridLayout>
#include <QTabWidget>

#include "Viewer.h"
#include "PatientWidget.h"
#include "patientform.h"
#include "examinationform.h"
#include "Scanner.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
  ~MainWindow();

public slots:
  void openScan(QString filename);
  void calculateDiff();
  void calculateMirror();
  void onItemSelected(QTreeWidgetItem* current, QTreeWidgetItem* previous);

  void addPatient();
  void addExam();

private:
  Viewer* viewer_;
  Scanner* scanner_;
  QGridLayout* main_layout;
  QStackedLayout* stacked_layout_;
  PatientForm* patient_form_;
  ExaminationForm* exam_form_;
  PatientWidget* patient_widget_;
  QTabWidget* viewport_tabs_;
};
