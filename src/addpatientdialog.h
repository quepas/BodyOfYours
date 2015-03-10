#ifndef ADDPATIENTDIALOG_H
#define ADDPATIENTDIALOG_H

#include "data/patient.h"
#include <QDialog>

namespace Ui {
  class AddPatientDialog;
}

class AddPatientDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AddPatientDialog(QWidget *parent = 0);
  AddPatientDialog(Patient patient, QWidget *parent = 0);
  ~AddPatientDialog();

  void ClearData();

private slots:
  void on_addPatientButton_clicked();
  void on_cancelAddPatientButton_clicked();

signals:
  void CreatePatientSignal(Patient data);

private:
  Ui::AddPatientDialog *ui;
  bool only_edit_;
};

#endif // ADDPATIENTDIALOG_H
