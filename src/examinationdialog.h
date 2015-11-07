#ifndef EXAMINATIONDIALOG_H
#define EXAMINATIONDIALOG_H

#include <QDialog>

struct ExaminationData;

namespace Ui {
  class ExaminationDialog;
}

class ExaminationDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ExaminationDialog(QWidget *parent = 0);
  ~ExaminationDialog();

private slots:
  void on_buttonBox_accepted();

  void on_buttonBox_rejected();

signals:
  void saveExamination(ExaminationData data);

private:
  Ui::ExaminationDialog *ui;
};

#endif // EXAMINATIONDIALOG_H
