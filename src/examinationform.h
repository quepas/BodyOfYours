#ifndef EXAMINATIONFORM_H
#define EXAMINATIONFORM_H

#include <QWidget>

namespace Ui {
  class ExaminationForm;
}

class ExaminationForm : public QWidget
{
  Q_OBJECT

public:
  explicit ExaminationForm(QWidget *parent = 0);
  ~ExaminationForm();

  void setName(QString name);
private:
  Ui::ExaminationForm *ui;
};

#endif // EXAMINATIONFORM_H
