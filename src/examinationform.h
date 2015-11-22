#ifndef EXAMINATIONFORM_H
#define EXAMINATIONFORM_H

#include "ExaminationData.h"

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>

namespace Ui {
  class ExaminationForm;
}

class ExaminationForm : public QWidget
{
  Q_OBJECT

public:
  explicit ExaminationForm(QWidget *parent = 0);
  ~ExaminationForm();

  void setData(const ExaminationData& data);
  void clear();

public slots:
  void onSave();
  void onClear();

signals:
  void saveExam(ExaminationData data);

private:
  Ui::ExaminationForm *ui;

  QPushButton* save_button_;
  QPushButton* clear_button_;
  QHBoxLayout* buttons_layout_;
};

#endif // EXAMINATIONFORM_H
