#pragma once

#include <QDataWidgetMapper>
#include <QFormLayout>
#include <QLineEdit>
#include <QSqlTableModel>
#include <QWidget>
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
