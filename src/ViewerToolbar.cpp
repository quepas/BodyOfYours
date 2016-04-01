#include "ViewerToolbar.h"

#include "GuiActions.h"
#include "MeshProcessing.h"
#include <QDebug>
#include <QFileDialog>

ViewerToolbar::ViewerToolbar(ScanViewer* viewer, QWidget* parent) : QToolBar(parent), viewer_(viewer)
{
  open_mesh_ = addAction(QIcon("icon/eye106.png"), tr(":open_mesh_3d"));
  open_mesh_->setToolTip(tr(":open_mesh_3d_tooltip"));
  clear_viewer_ = addAction(QIcon("icon/warning39.png"), tr(":clean_visualization_view"));
  clear_viewer_->setToolTip(tr(":clean_visualization_view_tooltip"));

  previous_quality_map_ = addAction(QIcon("icon/window58.png"), tr(":previous_quality_map"));
  previous_quality_map_->setToolTip(tr(":previous_quality_map_tooltip"));
  next_quality_map_ = addAction(QIcon("icon/expand43.png"), tr(":next_quality_map"));
  previous_quality_map_->setToolTip(tr(":next_quality_map_tooltip"));

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
        tr(":open_file_with_scan"),
        QString(),
        tr(":scans_3d_ply_or_any_other_files")); // Modele 3D w formacie PLY (*.ply); Wszystkie pliki (*)
  if (!filename.isEmpty()) {
    qDebug() << "[INFO] Opening mesh from file: " << filename;
    CMesh* mesh = new CMesh;
    MeshProcessing::loadMeshFromFile<CMesh>(filename, mesh);
    viewer_->insert(mesh);
    return true;
  }
  return false;
}
