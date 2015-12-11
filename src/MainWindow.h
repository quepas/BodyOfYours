#pragma once

#include <QMainWindow>
#include <QStackedLayout>
#include <QGridLayout>
#include <QTabWidget>

#include "ActionHub.h"
#include "MeshViewer.h"
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
  MeshViewer* viewer_;
  Scanner* scanner_;
  QGridLayout* main_layout;
  PatientTreeWidget* patient_widget_;
  QTabWidget* viewport_tabs_;
  ActionHub* action_hub_;
  FormWidget* main_form_;
};
