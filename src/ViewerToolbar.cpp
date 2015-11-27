#include "ViewerToolbar.h"

#include "MeshProcessing.h"
#include <QDebug>
#include <QFileDialog>

ViewerToolbar::ViewerToolbar(Viewer* viewer, QWidget* parent) : QToolBar(parent), viewer_(viewer)
{
  open_mesh_ = addAction(QIcon("icon/eye106.png"), tr("Otworz siatke 3D"));
  open_mesh_->setToolTip(tr("Otworz siatke 3D"));
  clear_viewer_ = addAction(QIcon("icon/warning39.png"), tr("Wyczysc widok wizualizacji"));
  clear_viewer_->setToolTip(tr("Wyczysc widok wizualizacji"));

  connect(open_mesh_, &QAction::triggered, [=]{
    if (openMeshFromFile()) {
      emit showTabWithIndex(1);
    }
  });
  connect(clear_viewer_, &QAction::triggered, [=]{
    viewer->clearMesh();
    emit showTabWithIndex(1);
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
    openMesh(filename, *mesh);
    viewer_->addMesh("open_mesh", mesh);
    return true;
  }
  return false;
}
