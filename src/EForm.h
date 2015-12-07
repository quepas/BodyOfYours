#pragma once

#include <QWidget>
#include "FormWidget.h"

#include <QLineEdit>

class EForm : public FormWidget
{
  Q_OBJECT
public:
  EForm(QWidget* parent);
  ~EForm();

private:
  QLineEdit* examName_;
  QLineEdit* scanName_;
};
