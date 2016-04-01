#include "ScannerToolbar.h"
#include "GuiActions.h"
#include "MeshProcessing.h"

#include <QCryptographicHash>
#include <QFileDialog>
#include <QDebug>

ScannerToolbar::ScannerToolbar(QWidget* parent /*= nullptr*/) : QToolBar(parent)
{
  numSensorLabel_ = new QLabel(tr(":no_sensors"));
  addWidget(numSensorLabel_);
  addSeparator();
  start_recon_ = addAction(QIcon("icon/player2.png"), tr(":start_reconstruction"));
  start_recon_->setToolTip(tr(":start_reconstruction_tooltip"));
  stop_recon_ = addAction(QIcon("icon/stop48.png"), tr(":stop_reconstruction"));
  stop_recon_->setToolTip(tr(":stop_reconstruction_tooltip"));
  addExternalMesh_ = addAction(QIcon("icon/film63.png"), tr(":add_external_scan"));
  addExternalMesh_->setToolTip(tr(":add_external_scan_tooltip"));
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
      tr(":open_file_with_scan"),
      QString("data/"),
      tr(":scans_3d_ply_or_any_other_files")); // Skany 3D w formacie PLY (*.ply); Wszystkie pliki (*)
    if (!filename.isEmpty()) emit stopReconstruction(filename);
    // TODO: copy external scan to local 'data' directory, change filename
  });
}

ScannerToolbar::~ScannerToolbar()
{
  delete stop_recon_;
  delete start_recon_;
  delete numSensorLabel_;
}

void ScannerToolbar::setEnabled(bool enabled)
{
  start_recon_->setEnabled(enabled);
  stop_recon_->setEnabled(enabled);
  addExternalMesh_->setEnabled(enabled);
}

void ScannerToolbar::showNumSensor(int numSensor)
{
  if (numSensor > 0)
    numSensorLabel_->setText(QString(":sensor_num").arg(numSensor)); // Sensory: %1
}
