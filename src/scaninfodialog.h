#ifndef SCANINFODIALOG_H
#define SCANINFODIALOG_H

#include "data/scan.h"

#include <QDialog>

namespace Ui {
  class ScanInfoDialog;
}

class ScanInfoDialog : public QDialog
{
  Q_OBJECT

public:
  // create purpose
  ScanInfoDialog(Patient owner, QWidget *parent = 0);
  // update purpose
  ScanInfoDialog(Patient owner, Scan scan, QWidget *parent = 0);
  ~ScanInfoDialog();

  void ClearForm();

private slots:
  void on_okButton_clicked();
  void on_cancelButton_clicked();

private:
  Ui::ScanInfoDialog *ui;
  Scan scan_;
};

#endif // SCANINFODIALOG_H
