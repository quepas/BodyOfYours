#pragma once

#include "FormButtons.h"
#include <QFormLayout>
#include <QVBoxLayout>
#include <QWidget>

class Form : public QWidget
{
  Q_OBJECT
public:
  explicit Form(QWidget* parent = nullptr);
  ~Form();

  void setEnabled(bool enabled);

protected:
  QFormLayout* form_;
  QWidget* form_widget_;
  QVBoxLayout* main_;
  FormButtons* buttons_;
};