#include "scans_data_tree.h"


ScansDataTree::ScansDataTree(QTreeView* tree_view)
  : view_(tree_view),
    model_(new ScansTreeModel(nullptr))
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
  if (index.isValid()) {
    patient_context_menu_->exec(view_->mapToGlobal(point));
  }
}

void ScansDataTree::InitPatientContextMenu()
{
  patient_context_menu_ = new QMenu(view_);
  patient_context_menu_->addAction("Patient dummy action");
}

void ScansDataTree::InitScanContextMenu()
{
  scan_context_menu_ = new QMenu(view_);
  scan_context_menu_->addAction("Scan dummy action");
}
