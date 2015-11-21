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

void ExaminationForm::setName(QString name)
{
  ui->nameLineEdit->setText(name);
}
