#include "PatientWidgetToolbar.h"

#include <QAction>
#include <QDebug>

PatientWidgetToolbar::PatientWidgetToolbar(QWidget* parent /*= nullptr*/)
  : QToolBar(parent)
{
  add_patient_ = addAction(QIcon("icon/broken8.png"), tr("Nowy pacjent"));
  add_patient_->setToolTip(tr("Nowy pacjent"));
  add_examination_ = addAction(QIcon("icon/stethoscope1.png"), tr("Nowe badanie"));
  add_examination_->setToolTip(tr("Nowe badanie"));
  remove_item_ = addAction(QIcon("icon/remove22.png"), tr("Usun element"));
  remove_item_->setToolTip(tr("Usun element"));
  calculate_diff_ = addAction(QIcon("icon/woman52.png"), tr("Roznica skanow"));
  calculate_diff_->setToolTip(tr("Roznica skanow"));
  calculate_mirror_ = addAction(QIcon("icon/two25.png"), tr("Odbicie lustrzane skanow"));
  calculate_mirror_->setToolTip(tr("Odbicie lustrzane skanow"));

  connect(add_patient_, &QAction::triggered, [=]{ emit addNewPatient(); });
  connect(add_examination_, &QAction::triggered, [=]{ emit addNewExamination(); });
  connect(remove_item_, &QAction::triggered, [=]{ emit removeItem(); });
  connect(calculate_diff_, &QAction::triggered, [=]{ emit calculateDiff(); });
  connect(calculate_mirror_, &QAction::triggered, [=]{ emit calculateMirror(); });
}

PatientWidgetToolbar::~PatientWidgetToolbar()
{
  delete add_examination_;
  delete add_patient_;
  delete remove_item_;
  delete calculate_diff_;
  delete calculate_mirror_;
}
