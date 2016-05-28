#pragma once

#include <QAction>
#include <QToolBar>

class ScanOperationToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScanOperationToolbar(QWidget* parent = nullptr);

private:
  QAction* compute_diff_;
  QAction* compute_mirror_;
  QAction* show_;
};
