#pragma once

#include "FormWidget.h"
#include <QLineEdit>

class ExaminationForm : public FormWidget
{
  Q_OBJECT
public:
  ExaminationForm(QSqlTableModel* model, QWidget* parent);
  ~ExaminationForm();

private:
  QLineEdit* examName_;
  QLineEdit* scanName_;
};
