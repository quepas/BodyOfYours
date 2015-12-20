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
  showScan_ = addAction(QIcon(tr("icon/radiography.png")), tr("Pokaz skan"));
  showScan_->setToolTip(tr("Wyswietl skan"));

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
      bool is_patient = PatientTreeItem::isPatient(current);
      add_examination_->setEnabled(is_patient);
      bool isScan = PatientTreeItem::isScan(current);
      calculate_diff_->setEnabled(isScan);
      calculate_mirror_->setEnabled(isScan);
      showScan_->setEnabled(isScan);
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
