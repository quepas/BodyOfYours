#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "data/patient.h"
#include "addpatientdialog.h"
#include "scans_viewer.h"
#include "scans_tree.h"
#include "scans_data_tree.h"
#include "scanner_3d.h"
#include "scanningwindow.h"
#include "reme_scanner_3d.h"

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_scansTree_doubleClicked(const QModelIndex& index);
  void on_computingDevicesComboBox_currentIndexChanged(int index);
  void on_scanButton_clicked();
  void on_addPatientButton_clicked();
  void on_removePatientButton_clicked();

  void CreatePatientSlot(Patient data);

private:
  Ui::MainWindow *ui;
  ScanningWindow *scanning_window_;
  AddPatientDialog *add_patient_dialog_;
  ScansViewer* scans_viewer_;
  ScansTree* scans_tree_;
  ScansDataTree* scans_data_tree_;
  Scanner3D* scanner3d_;

};

#endif // MAINWINDOW_H
