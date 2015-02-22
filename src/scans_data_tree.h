#pragma once

#include "scans_tree_model.h"

#include <QObject>
#include <QMenu>
#include <QTreeView>

class ScansDataTree : public QObject
{
  Q_OBJECT
public:
  ScansDataTree(QTreeView* tree_view);
  ~ScansDataTree();

  ScansTreeModel* model() { return model_; }
  bool RemoveSelected();
  void SetScanActionEnable(bool enabled);

private slots:
  void CustomContextMenuSlot(const QPoint& point);
  void RemoveSelectedSlot();

private:
  QTreeView* view_;
  ScansTreeModel* model_;
  QMenu *patient_context_menu_, *scan_context_menu_;
  QAction* scan_patient_;

  void InitPatientContextMenu();
  void InitScanContextMenu();
};
