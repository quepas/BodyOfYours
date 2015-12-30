#include "ViewerToolbar.h"

#include "GuiActions.h"
#include "MeshProcessing.h"
#include <QDebug>
#include <QFileDialog>

ViewerToolbar::ViewerToolbar(ScanViewer* viewer, QWidget* parent) : QToolBar(parent), viewer_(viewer)
{
  open_mesh_ = addAction(QIcon("icon/eye106.png"), tr("Otworz siatke 3D"));
  open_mesh_->setToolTip(tr("Otworz siatke 3D"));
  clear_viewer_ = addAction(QIcon("icon/warning39.png"), tr("Wyczysc widok wizualizacji"));
  clear_viewer_->setToolTip(tr("Wyczysc widok wizualizacji"));

  previous_quality_map_ = addAction(QIcon("icon/window58.png"), tr("Poprzednia mapa kolorow"));
  previous_quality_map_->setToolTip(tr("Poprzednia mapa kolorow"));
  next_quality_map_ = addAction(QIcon("icon/expand43.png"), tr("Nastepna mapa kolorow"));
  previous_quality_map_->setToolTip(tr("Nastepna mapa kolorow"));

  connect(open_mesh_, &QAction::triggered, [=]{
    if (openMeshFromFile()) {
      emit showTabWithIndex(1);
    }
  });
  connect(clear_viewer_, &QAction::triggered, [=]{
    ActionHub::trigger(ActionMeshViewerClear::TYPE());
    emit showTabWithIndex(1);
  });
  connect(previous_quality_map_, &QAction::triggered, [=]{
    qDebug() << "Prev quality map";
    auto mesh = viewer_->last();
    if (mesh) {
      QVector<float> quality;
      generateRandomQualityForMesh(*mesh, quality);
      applyQualityToMesh(*mesh, quality);
      viewer_->update();
    }
  });
  connect(next_quality_map_, &QAction::triggered, [=]{
    qDebug() << "Next quality map";
    auto mesh = viewer_->last();
    if (mesh) {
      QVector<float> quality;
      generateRandomQualityForMesh(*mesh, quality);
      applyQualityToMesh(*mesh, quality);
      viewer_->update();
    }
  });
}

ViewerToolbar::~ViewerToolbar()
{
  delete open_mesh_;
  delete clear_viewer_;
}

bool ViewerToolbar::openMeshFromFile()
{
  QString filename = QFileDialog::
    getOpenFileName(this,
        tr("Otworz plik z modelem 3D"),
        QString(),
        tr("Modele 3D w formacie PLY (*.ply); Wszystkie pliki (*)"));
  if (!filename.isEmpty()) {
    qDebug() << "[INFO] Opening mesh from file: " << filename;
    CMesh* mesh = new CMesh;
    MeshProcessing::loadMeshFromFile<CMesh>(filename, mesh);
    viewer_->insert(mesh);
    return true;
  }
  return false;
}
