#pragma once

#include "Scanner.h"
#include <QAction>
#include <QToolBar>

class ScannerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScannerToolbar(Scanner * scanner, QWidget* parent = nullptr);
  ~ScannerToolbar();

private:
  QAction* start_recon_;
  QAction* stop_recon_;
};
