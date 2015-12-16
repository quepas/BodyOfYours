#include "ScannerToolbar.h"
#include "GuiActions.h"
#include "MeshProcessing.h"

#include <QCryptographicHash>
#include <QDebug>

ScannerToolbar::ScannerToolbar(PatientTreeWidget* patient_tree, QWidget* parent /*= nullptr*/)
  : QToolBar(parent)
{
  start_recon_ = addAction(QIcon("icon/player2.png"), tr("Rozpocznij rekonstrukcje"));
  start_recon_->setToolTip(tr("Rozpocznij rekonstrukcje"));
  stop_recon_ = addAction(QIcon("icon/stop48.png"), tr("Zatrzymaj rekonstrukcje"));
  stop_recon_->setToolTip(tr("Zatrzymaj rekonstrukcje"));

  setEnabled(false);
  connect(patient_tree, &PatientTreeWidget::itemClicked, [=](QTreeWidgetItem* item, int column) {
    setEnabled(PatientTreeItem::isExamination(item));
  });
  connect(start_recon_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionStartReconstruction::TYPE());
  });
  connect(stop_recon_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionStopReconstruction::TYPE());
    // save
    QByteArray time = QTime::currentTime().toString().toLocal8Bit();
    QString filePath = QString(QCryptographicHash::hash(time, QCryptographicHash::Md5).toHex());
    createDummyFile("data/" + filePath);
  });
}

ScannerToolbar::~ScannerToolbar()
{
  delete stop_recon_;
  delete start_recon_;
}

void ScannerToolbar::setEnabled(bool enabled)
{
  start_recon_->setEnabled(enabled);
  stop_recon_->setEnabled(enabled);
}
