#pragma once

#include "ExaminationData.h"
#include "Form.h"
#include <QWidget>
#include <QLineEdit>

class ExaminationForm : public Form
{
  Q_OBJECT

public:
  explicit ExaminationForm(QWidget *parent = 0);
  ~ExaminationForm();

  void fill(const ExaminationData& data);
  void clear();

public slots:
  void onButtonClicked(int button);

signals:
  void saveExam(ExaminationData data);

private:
  QLineEdit* name;
  QLineEdit* scan_name;
};
