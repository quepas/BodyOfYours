#pragma once

#include "scanningwindow.h"
#include "scans_tree_model.h"

#include <QObject>
#include <QMenu>
#include <QTreeView>

class ScansDataTree : public QObject
{
  Q_OBJECT
public:
  ScansDataTree(QTreeView* tree_view, ScanningWindow* scanning_window);
  ~ScansDataTree();

  ScansTreeModel* model() { return model_; }
  bool RemoveSelected();
  void SetScanActionEnable(bool enabled);

private slots:
  void CustomContextMenuSlot(const QPoint& point);
  void RemoveSelectedSlot();
  void ScanPatient();

private:
  QTreeView* view_;
  ScansTreeModel* model_;
  QMenu *patient_context_menu_, *scan_context_menu_;
  QAction* scan_patient_;
  ScanningWindow* scanning_window_;

  void InitPatientContextMenu();
  void InitScanContextMenu();
};
