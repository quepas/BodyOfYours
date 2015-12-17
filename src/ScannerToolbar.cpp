#include "ScannerToolbar.h"
#include "GuiActions.h"
#include "MeshProcessing.h"

#include <QCryptographicHash>
#include <QFileDialog>
#include <QDebug>

ScannerToolbar::ScannerToolbar(QWidget* parent /*= nullptr*/) : QToolBar(parent)
{
  start_recon_ = addAction(QIcon("icon/player2.png"), tr("Rozpocznij rekonstrukcje"));
  start_recon_->setToolTip(tr("Rozpocznij rekonstrukcje"));
  stop_recon_ = addAction(QIcon("icon/stop48.png"), tr("Zatrzymaj rekonstrukcje"));
  stop_recon_->setToolTip(tr("Zatrzymaj rekonstrukcje"));
  addExternalMesh_ = addAction(QIcon("icon/film63.png"), tr("Dodaj zewnetrzny skan"));
  addExternalMesh_->setToolTip(tr("Dodaj zewnetrzny skan"));

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
  connect(addExternalMesh_, &QAction::triggered, [=] {
    // find external mesh
    QString filename = QFileDialog::
      getOpenFileName(this,
      tr("Otworz plik ze skanem"),
      QString("data/"),
      tr("Skany 3D w formacie PLY (*.ply); Wszystkie pliki (*)"));
    if (!filename.isEmpty()) emit stopReconstruction(filename);
    // TODO: copy external scan to local 'data' directory, change filename
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
  addExternalMesh_->setEnabled(enabled);
}
