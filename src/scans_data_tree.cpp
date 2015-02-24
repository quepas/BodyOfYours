#include "scans_data_tree.h"
#include "resources.h"
#include "addpatientdialog.h"

#include <QAction>

ScansDataTree::ScansDataTree(QTreeView* tree_view, ScanningWindow* scanning_window)
  : view_(tree_view),
    model_(new ScansTreeModel(nullptr)),
    scanning_window_(scanning_window)
{
  view_->setModel(model_);
  view_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(view_, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(CustomContextMenuSlot(const QPoint&)));
  InitPatientContextMenu();
  InitScanContextMenu();
}

ScansDataTree::~ScansDataTree()
{
  delete model_;
}

void ScansDataTree::CustomContextMenuSlot(const QPoint& point)
{
  QModelIndex index = view_->indexAt(point);
  if (index.isValid() && index.parent() == model_->invisibleRootItem()->index()) {
    patient_context_menu_->exec(view_->mapToGlobal(point));
  } else {
    scan_context_menu_->exec(view_->mapToGlobal(point));
  }
}

void ScansDataTree::InitPatientContextMenu()
{
  patient_context_menu_ = new QMenu(view_);
  QAction* remove_patient = new QAction(QIcon(Resources::ICON_REMOVE), "Remove patient", nullptr);
  QAction* modify_patient = new QAction(QIcon(Resources::ICON_MODIFY), "Modify patient", nullptr);
  scan_patient_ = new QAction(QIcon(Resources::ICON_CREATE), "Make new scan", nullptr);
  scan_patient_->setEnabled(false);
  patient_context_menu_->addAction(remove_patient);
  patient_context_menu_->addAction(modify_patient);
  patient_context_menu_->addAction(scan_patient_);
  connect(remove_patient, SIGNAL(triggered()), this, SLOT(RemoveSelectedSlot()));
  connect(scan_patient_, SIGNAL(triggered()), this, SLOT(ScanPatient()));
  connect(modify_patient, SIGNAL(triggered()), this, SLOT(ModifyPatient()));
}

void ScansDataTree::InitScanContextMenu()
{
  scan_context_menu_ = new QMenu(view_);
  QAction* remove_scan = new QAction(QIcon(Resources::ICON_REMOVE), "Remove scan", nullptr);
  scan_context_menu_->addAction(remove_scan);
}

bool ScansDataTree::RemoveSelected()
{
  QModelIndex index = view_->currentIndex();
  if (index.isValid()) {
    model_->RemovePatient(index);
  }
  return false;
}

void ScansDataTree::RemoveSelectedSlot()
{
  RemoveSelected();
}

void ScansDataTree::SetScanActionEnable(bool enabled)
{
  scan_patient_->setEnabled(enabled);
}

void ScansDataTree::ScanPatient()
{
  scanning_window_->show();
  scanning_window_->StartGrabbingData();
}

void ScansDataTree::ModifyPatient()
{
  QModelIndex index = view_->currentIndex();
  if (index.isValid()) {
    QString text = model_->itemFromIndex(index)->text();
    foreach(PatientData data, model_->patient_data()) {
      if (text == data.name) {
        AddPatientDialog* dialog = new AddPatientDialog(data);
        dialog->show();
        return;
      }
    }
  }
}
