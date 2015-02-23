#ifndef ADDPATIENTDIALOG_H
#define ADDPATIENTDIALOG_H

#include "patient_data.h"
#include <QDialog>

namespace Ui {
  class AddPatientDialog;
}

class AddPatientDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AddPatientDialog(QWidget *parent = 0);
  AddPatientDialog(PatientData patient_data, QWidget *parent = 0);
  ~AddPatientDialog();

  void ClearData();

private slots:
  void on_addPatientButton_clicked();
  void on_cancelAddPatientButton_clicked();

signals:
  void AddPatientSignal(PatientData data);

private:
  Ui::AddPatientDialog *ui;
};

#endif // ADDPATIENTDIALOG_H
