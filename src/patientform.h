#pragma once

#include "Form.h"
#include "PatientData.h"
#include "FormButtons.h"

#include <QWidget>
#include <QFormLayout>
#include <QLineEdit>
#include <QVBoxLayout>

class PatientForm : public Form
{
  Q_OBJECT

public:
  explicit PatientForm(QWidget *parent = 0);
  ~PatientForm();

  void fill(const PatientData& data);
  void clear();

public slots:
  void onButtonClicked(int button);

signals:
  void savePatient(PatientData name);
  void deletePatient();

private:
  QLineEdit* name;
  QLineEdit* surname;
  QLineEdit* pesel;
};
