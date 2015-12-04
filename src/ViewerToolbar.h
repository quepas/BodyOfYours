#pragma once

#include "MeshViewer.h"
#include <QAction>
#include <QToolBar>

class ViewerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ViewerToolbar(MeshViewer* viewer, QWidget* parent);
  ~ViewerToolbar();

private:
  QAction* open_mesh_;
  QAction* open_mesh_quality_maps_;
  QAction* clear_viewer_;
  QAction* previous_quality_map_;
  QAction* next_quality_map_;
  MeshViewer* viewer_;

  bool openMeshFromFile();

signals:
  void showTabWithIndex(int);
};