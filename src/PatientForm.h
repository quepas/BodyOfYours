#pragma once

#include <QLineEdit>
#include "FormWidget.h"

class PatientForm : public FormWidget
{
  Q_OBJECT
public:
  PatientForm(QWidget* parent);
  ~PatientForm();

private:
  QLineEdit* name_;
  QLineEdit* surname_;
  QLineEdit* pesel_;
};
