#pragma once

#include <QLineEdit>
#include "FormWidget.h"

class PatientForm : public FormWidget
{
  Q_OBJECT
public:
  PatientForm(QSqlTableModel* model, QWidget* parent);
  ~PatientForm();

private:
  QLineEdit* name_;
  QLineEdit* surname_;
  QLineEdit* pesel_;
};
