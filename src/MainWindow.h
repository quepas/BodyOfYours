#pragma once

#include <QMainWindow>
#include <QStackedLayout>
#include <QGridLayout>
#include <QTabWidget>
#include <QSqlTableModel>

#include "ActionHub.h"
#include "ScanViewer.h"
#include "MeshDifferenceDlg.h"
#include "PatientTreeWidget.h"
#include "Scanner.h"
#include "FormWidget.h"

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

private:
  ScanViewer* viewer_;
  ScanViewer* miniScanViewer_;
  Scanner* scanner_;
  QGridLayout* main_layout;
  PatientTreeWidget* patient_widget_;
  QTabWidget* viewport_tabs_;
  ActionHub* action_hub_;
  FormWidget* main_form_;
  MeshDifferenceDlg* meshDiffDlg_;
  // SQL models
  QSqlTableModel* patient_model_;
  QSqlTableModel* exam_model_;
  QSqlTableModel* scan_model_;
  QSqlTableModel* scanDiffModel_;

  void initModels();
};
