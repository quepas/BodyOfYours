#include "Form.h"

Form::Form(QWidget* parent /*= nullptr*/)
  : QWidget(parent)
{
  main_ = new QVBoxLayout(this);
  form_widget_ = new QWidget(this);
  form_ = new QFormLayout(form_widget_);
  main_->addWidget(form_widget_);
  buttons_ = new FormButtons(this);
  main_->addWidget(buttons_->toQWidget());
  main_->addStretch();
}

Form::~Form()
{
  delete buttons_;
  delete form_;
  delete form_widget_;
  delete main_;
}

void Form::setEnabled(bool enabled)
{
  form_widget_->setEnabled(enabled);
}
