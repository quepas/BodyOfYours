#include "examinationform.h"
#include "ui_examinationform.h"

#include <QDebug>

ExaminationForm::ExaminationForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ExaminationForm)
{
  ui->setupUi(this);
  save_button_ = new QPushButton(tr("Zapisz"));
  connect(save_button_, SIGNAL(clicked()), this, SLOT(onSave()));
  clear_button_ = new QPushButton(tr("Wyczysc"));
  connect(clear_button_, SIGNAL(clicked()), this, SLOT(onClear()));

  buttons_layout_ = new QHBoxLayout;
  //buttons_layout_->addStretch(1);
  buttons_layout_->addWidget(save_button_);
  buttons_layout_->addWidget(clear_button_);

  setLayout(buttons_layout_);
}

ExaminationForm::~ExaminationForm()
{
  delete save_button_;
  delete clear_button_;
  delete buttons_layout_;
  delete ui;
}

void ExaminationForm::setData(const ExaminationData& data)
{
  ui->nameLineEdit->setText(data.name);
  ui->scanNameLineEdit->setText(data.scan_name);
}

void ExaminationForm::clear()
{
  ui->nameLineEdit->clear();
  ui->descLineEdit->clear();
  ui->scanNameLineEdit->clear();
  ui->dateDateTimeEdit->clear();
}

void ExaminationForm::onSave()
{
  qDebug() << "ExaminationForm::onSave()";
  ExaminationData data;
  data.name = ui->nameLineEdit->text();
  data.scan_name = ui->scanNameLineEdit->text();
  emit saveExam(data);
}

void ExaminationForm::onClear()
{
  qDebug() << "ExaminationForm::onClear()";
  clear();
}
