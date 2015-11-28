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
  QAction* clear_viewer_;
  MeshViewer* viewer_;

  bool openMeshFromFile();

signals:
  void showTabWithIndex(int);
};