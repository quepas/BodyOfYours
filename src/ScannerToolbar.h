#pragma once

#include <QAction>
#include <QToolBar>

class ScannerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScannerToolbar(QWidget* parent = nullptr);
  ~ScannerToolbar();

private:
  QAction* start_recon_;
  QAction* stop_recon_;
};
