#include "ScannerToolbar.h"
#include "GuiActions.h"
#include "MeshProcessing.h"

#include <QCryptographicHash>
#include <QDebug>

ScannerToolbar::ScannerToolbar(QWidget* parent /*= nullptr*/) : QToolBar(parent)
{
  start_recon_ = addAction(QIcon("icon/player2.png"), tr("Rozpocznij rekonstrukcje"));
  start_recon_->setToolTip(tr("Rozpocznij rekonstrukcje"));
  stop_recon_ = addAction(QIcon("icon/stop48.png"), tr("Zatrzymaj rekonstrukcje"));
  stop_recon_->setToolTip(tr("Zatrzymaj rekonstrukcje"));

  setEnabled(false);
  connect(start_recon_, &QAction::triggered, [=]{
    emit startReconstruction();
  });
  connect(stop_recon_, &QAction::triggered, [=]{
    QByteArray time = QTime::currentTime().toString().toLocal8Bit();
    QString filePath = QString(QCryptographicHash::hash(time, QCryptographicHash::Md5).toHex());
    QString fullMeshFilePath = "data/" + filePath;
    createDummyFile(fullMeshFilePath);
    emit stopReconstruction(fullMeshFilePath);
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
