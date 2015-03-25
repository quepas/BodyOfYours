#pragma once

#include "scanningwindow.h"
#include "patient_tree_model.h"

#include <QObject>
#include <QMenu>
#include <QTreeView>

class ScansDataTree : public QTreeView
{
  Q_OBJECT
public:
  ScansDataTree(QWidget* parent = 0);
  ~ScansDataTree();

  void set_scanning_window(ScanningWindow* scanning_window) { scanning_window_ = scanning_window; }
  PatientTreeModel* model() { return model_; }
  bool RemoveSelected();
  void SetScanActionEnable(bool enabled);

private slots:
  void CustomContextMenuSlot(const QPoint& point);
  void RemoveSelectedSlot();
  void ScanPatient();
  void ModifyPatient();
  void UpdatePatientSlot(Patient patient);

  void RemoveScanSlot();
  void ModifyScanSlot();
  void VisualizeScanSlot();
  void SmoothScanSlot();

signals:
  void VisualizeScanSignal(QString scan_full_path);

private:
  PatientTreeModel* model_;
  QMenu *patient_context_menu_, *scan_context_menu_;
  QAction* scan_patient_;
  ScanningWindow* scanning_window_;

  void InitPatientContextMenu();
  void InitScanContextMenu();
};
