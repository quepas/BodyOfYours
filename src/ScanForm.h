#pragma once

#include "FormWidget.h"
#include <QLineEdit>
#include <QTableView>

class ScanForm : public FormWidget
{
  Q_OBJECT
public:
  ScanForm(QSqlTableModel* model, QWidget* parent);
  ~ScanForm();

public slots:
  virtual void setCurrentRowWithId(int rowId);

signals:
  void displayMeshWithQualityMap(int refMeshID, int diffID);

private:
  QLineEdit* scanName_;
  QLineEdit* scanFilePath_;
  QTableView* scanDiffTable_;
  QSqlTableModel* scanDiffModel_;

  void setupScanDiffTable(int scanDiffID);
};
