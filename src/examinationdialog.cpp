#include "examinationdialog.h"
#include "ui_examinationdialog.h"
#include "ExaminationItem.h"

#include <QDebug>

ExaminationDialog::ExaminationDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ExaminationDialog)
{
  ui->setupUi(this);
}

ExaminationDialog::~ExaminationDialog()
{
  delete ui;
}

void ExaminationDialog::on_buttonBox_accepted()
{
  qDebug() << "Accepted";
  ExaminationData data;
  data.name = ui->examName->text();
  emit saveExamination(data);
}

void ExaminationDialog::on_buttonBox_rejected()
{
  qDebug() << "Rejected";
}
