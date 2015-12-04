#include "PatientTreeToolbar.h"

#include <QAction>
#include <QDebug>
#include "GuiActions.h"
#include "ActionHub.h"

PatientTreeToolbar::PatientTreeToolbar(PatientTreeWidget* patient_widget, QWidget* parent /*= nullptr*/) : QToolBar(parent)
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
  show_scan_ = addAction(QIcon(tr("icon/radiography.png")), tr("Pokaz skan"));
  show_scan_->setToolTip(tr("Wyswietl skan"));

  connect(add_patient_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionAddNewPatient::TYPE());
  });
  connect(add_examination_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionAddNewExamination::TYPE());
  });
  connect(remove_item_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionDeleteCurrentItem::TYPE());
  });
  connect(calculate_diff_, &QAction::triggered, [=]{ emit calculateDiff(); });
  connect(calculate_mirror_, &QAction::triggered, [=]{ emit calculateMirror(); });
  connect(show_scan_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionShowScanMesh::TYPE());
  });
  connect(patient_widget, &PatientTreeWidget::currentItemChanged, [=](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
    if (current) {
      bool is_patient = PatientTreeItem::isPatient(current);
      add_examination_->setEnabled(is_patient);
      calculate_diff_->setEnabled(!is_patient);
      calculate_mirror_->setEnabled(!is_patient);
      show_scan_->setEnabled(!is_patient);
    }
  });
}

PatientTreeToolbar::~PatientTreeToolbar()
{
  delete add_examination_;
  delete add_patient_;
  delete remove_item_;
  delete calculate_diff_;
  delete calculate_mirror_;
  delete show_scan_;
}
