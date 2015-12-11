#pragma once

#include <QLineEdit>
#include "FormWidget.h"

class PForm : public FormWidget
{
  Q_OBJECT
public:
  PForm(QWidget* parent);
  ~PForm();

private:
  QLineEdit* name_;
  QLineEdit* surname_;
  QLineEdit* pesel_;
};
