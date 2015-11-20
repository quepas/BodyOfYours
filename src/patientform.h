#ifndef PATIENTFORM_H
#define PATIENTFORM_H

#include <QWidget>

namespace Ui {
  class PatientForm;
}

class PatientForm : public QWidget
{
  Q_OBJECT

public:
  explicit PatientForm(QWidget *parent = 0);
  ~PatientForm();

  void setData(QString name, QString surname);
private:
  Ui::PatientForm *ui;
};

#endif // PATIENTFORM_H
