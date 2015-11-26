#pragma once

#include "Viewer.h"
#include <QAction>
#include <QToolBar>

class ViewerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ViewerToolbar(Viewer* viewer, QWidget* parent);
  ~ViewerToolbar();

private:
  QAction* open_mesh_;
  QAction* clear_viewer_;
  Viewer* viewer_;

  void openMeshFromFile();
};