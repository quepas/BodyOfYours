#include "examinationform.h"
#include "ui_examinationform.h"

ExaminationForm::ExaminationForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ExaminationForm)
{
  ui->setupUi(this);
}

ExaminationForm::~ExaminationForm()
{
  delete ui;
}

void ExaminationForm::setData(const ExaminationData& data)
{
  ui->nameLineEdit->setText(data.name);
}
