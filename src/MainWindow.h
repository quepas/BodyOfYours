#pragma once

#include <QMainWindow>
#include <QStackedLayout>
#include <QGridLayout>
#include <QTabWidget>

#include "MeshViewer.h"
#include "PatientWidget.h"
#include "patientform.h"
#include "examinationform.h"
#include "Scanner.h"
#include "FormViewer.h"

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
  MeshViewer* viewer_;
  Scanner* scanner_;
  QGridLayout* main_layout;
  PatientWidget* patient_widget_;
  QTabWidget* viewport_tabs_;
  FormViewer* form_viewer_;
};
