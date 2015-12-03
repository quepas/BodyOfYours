#include "ScannerToolbar.h"
#include "GuiActions.h"

ScannerToolbar::ScannerToolbar(QWidget* parent /*= nullptr*/)
  : QToolBar(parent)
{
  start_recon_ = addAction(QIcon("icon/player2.png"), tr("Rozpocznij rekonstrukcje"));
  start_recon_->setToolTip(tr("Rozpocznij rekonstrukcje"));
  stop_recon_ = addAction(QIcon("icon/stop48.png"), tr("Zatrzymaj rekonstrukcje"));
  stop_recon_->setToolTip(tr("Zatrzymaj rekonstrukcje"));

  connect(start_recon_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionStartReconstruction::TYPE());
  });
  connect(stop_recon_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionStopReconstruction::TYPE());
  });
}

ScannerToolbar::~ScannerToolbar()
{
  delete stop_recon_;
  delete start_recon_;
}
