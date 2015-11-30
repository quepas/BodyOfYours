#ifndef PATIENTFORM_H
#define PATIENTFORM_H

#include "PatientData.h"
#include "FormButtons.h"

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>

namespace Ui {
  class PatientForm;
}

class PatientForm : public QWidget
{
  Q_OBJECT

public:
  explicit PatientForm(QWidget *parent = 0);
  ~PatientForm();

  void setData(const PatientData& data);
  void clear();

  void setShowState(bool show_state);

public slots:
  void onButtonClicked(int button);

signals:
  void savePatient(PatientData name);
  void deletePatient();

private:
  Ui::PatientForm *ui;

  FormButtons* form_buttons_;
};

#endif // PATIENTFORM_H
