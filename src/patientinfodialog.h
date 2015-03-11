#ifndef PATIENTINFODIALOG_H
#define PATIENTINFODIALOG_H

#include "data/patient.h"
#include <QDialog>

namespace Ui {
  class PatientInfoDialog;
}

class PatientInfoDialog : public QDialog
{
  Q_OBJECT

public:
  explicit PatientInfoDialog(QWidget *parent = 0);
  PatientInfoDialog(Patient patient, QWidget *parent = 0);
  ~PatientInfoDialog();

  void ClearForm();

private slots:
  void on_addPatientButton_clicked();
  void on_cancelAddPatientButton_clicked();

signals:
  void UpdatePatientSignal(Patient data);
  void CreatePatientSignal(Patient data);

private:
  Ui::PatientInfoDialog *ui;
  Patient patient_;
};

#endif // ADDPATIENTDIALOG_H
