#include "StackedFormWidget.h"

#include "PForm.h"
#include "EForm.h"

StackedFormWidget::StackedFormWidget(QWidget* parent /*= nullptr*/)
  : QStackedWidget(parent)
{
  widgets_.append(new PForm(this));
  widgets_.append(new EForm(this));
  addWidget(widgets_[0]);
  addWidget(widgets_[1]);
}

StackedFormWidget::~StackedFormWidget()
{
  for (int i = 0; i < widgets_.size(); ++i) {
    delete widgets_[i];
  }
}

void StackedFormWidget::switchTo(FormID id, int dataRowId)
{
  setCurrentIndex(id);
  widgets_[id]->setCurrentRowWithId(dataRowId);
}

void StackedFormWidget::switchTo(FormID id)
{
  setCurrentIndex(id);
  widgets_[id]->addRow();
}
