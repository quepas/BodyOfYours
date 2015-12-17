#pragma once

#include "FormWidget.h"
#include <QLineEdit>

class ScanForm : public FormWidget
{
  Q_OBJECT
public:
  ScanForm(QSqlTableModel* model, QWidget* parent);
  ~ScanForm();

private:
  QLineEdit* scanName_;
  QLineEdit* scanFilePath_;
};
