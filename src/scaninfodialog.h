#ifndef SCANINFODIALOG_H
#define SCANINFODIALOG_H

#include "scanning_3d.h"
#include "data/patient.h"

#include <QDialog>

namespace Ui {
  class ScanInfoDialog;
}

class ScanInfoDialog : public QDialog
{
  Q_OBJECT

public:
  // create purpose
  ScanInfoDialog(Scanning3D* scanning, Patient patient, QWidget *parent = 0);
  // update purpose
  ScanInfoDialog(Scanning3D* scanning, Patient patient, Scan scan, QWidget *parent = 0);
  ~ScanInfoDialog();

  void ClearForm();

signals:
  void UpdatePatientSignal(Patient data);

private slots:
  void on_okButton_clicked();
  void on_cancelButton_clicked();

private:
  Ui::ScanInfoDialog *ui;
  Scan scan_;
  Patient patient_;
  Scanning3D* scanning_;
  bool update_mode_;

};

#endif // SCANINFODIALOG_H
