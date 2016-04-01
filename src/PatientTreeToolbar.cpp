#include "PatientTreeToolbar.h"

#include <QAction>
#include <QDebug>
#include "GuiActions.h"
#include "ActionHub.h"

PatientTreeToolbar::PatientTreeToolbar(PatientTreeWidget* patient_widget, QWidget* parent /*= nullptr*/) : QToolBar(parent)
{
  add_patient_ = addAction(QIcon("icon/broken8.png"), tr(":new_patient"));
  add_patient_->setToolTip(tr(":new_patient_tooltip"));
  add_examination_ = addAction(QIcon("icon/stethoscope1.png"), tr(":new_examination"));
  add_examination_->setToolTip(tr(":new_examination_tooltip"));
  remove_item_ = addAction(QIcon("icon/remove22.png"), tr(":remove_element"));
  remove_item_->setToolTip(tr(":remove_element_tooltip"));
  calculate_diff_ = addAction(QIcon("icon/woman52.png"), tr(":scans_difference"));
  calculate_diff_->setToolTip(tr(":scans_difference_tooltip"));
  calculate_mirror_ = addAction(QIcon("icon/two25.png"), tr(":scans_mirror"));
  calculate_mirror_->setToolTip(tr(":scans_mirror_tooltip"));
  showScan_ = addAction(QIcon(tr("icon/radiography.png")), tr(":show_scan"));
  showScan_->setToolTip(tr(":show_scan_tooltip"));
  setDisabledExceptNewPatient(true);

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
  connect(showScan_, &QAction::triggered, [=]{
    auto item = patient_widget->currentItem();
    if (PatientTreeItem::isScan(item)) {
      emit displayScan(ScanViewer::ID::FULL_VIEWER, PatientTreeItem::getId(item));
    }
  });
  connect(patient_widget, &PatientTreeWidget::currentItemChanged, [=](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
    if (current) {
      remove_item_->setEnabled(true);
      bool is_patient = PatientTreeItem::isPatient(current);
      add_examination_->setEnabled(is_patient);
      bool isScan = PatientTreeItem::isScan(current);
      calculate_diff_->setEnabled(isScan);
      calculate_mirror_->setEnabled(isScan);
      showScan_->setEnabled(isScan);
    }
    else {
      setDisabledExceptNewPatient(true);
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
  delete showScan_;
}

void PatientTreeToolbar::setDisabledExceptNewPatient(bool disabled)
{
  add_examination_->setDisabled(disabled);
  remove_item_->setDisabled(disabled);
  calculate_diff_->setDisabled(disabled);
  calculate_mirror_->setDisabled(disabled);
  showScan_->setDisabled(disabled);
}
