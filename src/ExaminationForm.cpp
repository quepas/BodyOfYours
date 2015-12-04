#include "ExaminationForm.h"
#include "ExaminationData.h"
#include "GuiActions.h"

#include <QDebug>

ExaminationForm::ExaminationForm(QWidget *parent)
  : Form(parent)
{
  name = new QLineEdit(this);
  scan_name = new QLineEdit(this);
  form_->addRow(tr("Nazwa"), name);
  form_->addRow(tr("Plik skanu"), scan_name);
  connect(buttons_, SIGNAL(buttonClicked(int)), this, SLOT(onButtonClicked(int)));
}

ExaminationForm::~ExaminationForm()
{
  delete name;
  delete scan_name;
}

void ExaminationForm::fill(const ExaminationData& data)
{
  name->setText(data.name);
  scan_name->setText(data.scan_name);
  setEnabled(false);
  buttons_->setShowFormState();
}

void ExaminationForm::clear()
{
  name->clear();
  scan_name->clear();
  setEnabled(true);
  buttons_->setNewFormState();
}

void ExaminationForm::onButtonClicked(int button)
{
  switch (button) {
  case FormButtons::SAVE: {
    ExaminationData data;
    data.name = name->text();
    data.scan_name = scan_name->text();
    emit saveExam(data);
    setEnabled(false);
    break;
  }
  case FormButtons::MODIFY:
    setEnabled(false);
    break;
  case FormButtons::CLEAR:
    clear();
    break;
  case FormButtons::CANCEL:
    break;
  case FormButtons::REMOVE:
    clear();
    ActionHub::trigger(ActionDeleteCurrentItem::TYPE());
    break;
  case FormButtons::LOCK:
    setEnabled(false);
    break;
  case FormButtons::UNLOCK:
    setEnabled(true);
    break;
  default:
    break;
  }
}
