#ifndef SCANINFODIALOG_H
#define SCANINFODIALOG_H

#include <QDialog>

namespace Ui {
  class ScanInfoDialog;
}

class ScanInfoDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ScanInfoDialog(QWidget *parent = 0);
  ~ScanInfoDialog();

private slots:
  void on_okButton_clicked();

  void on_cancelButton_clicked();

private:
  Ui::ScanInfoDialog *ui;
};

#endif // SCANINFODIALOG_H
