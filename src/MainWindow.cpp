#include "MainWindow.h"

#include <QDebug>
#include <QAction>
#include <QToolBar>

#include "Database.h"
#include "MeshProcessing.h"
#include "PatientTreeItem.h"
#include "PatientTreeToolbar.h"
#include "ScannerToolbar.h"
#include "ScannerViewer.h"
#include "ViewerToolbar.h"

MainWindow::MainWindow() :
  scanner_(nullptr)
#ifndef _DEBUG
  ,
  viewer_(new MeshViewer())
#endif
{
  Database db("database.db");
  db.createScheme();
  form_viewer_ = new FormViewer(this);
  patient_widget_ = new PatientTreeWidget(form_viewer_, Database::selectPatient());
  connect(patient_widget_, SIGNAL(openScan(QString)), this, SLOT(openScan(QString)));
  QGridLayout* grid = new QGridLayout;
  grid->addWidget(patient_widget_, 0, 0, 2, 1);
  patient_widget_->setMaximumWidth(300);
  scanner_ = new Scanner(this);
  QWidget* viewport = new QWidget;
  ScannerViewer* scanner_viewer = new ScannerViewer(scanner_, this);
  viewport_tabs_ = new QTabWidget(this);
  viewport_tabs_->addTab(form_viewer_, tr("Formatki"));
#ifndef _DEBUG
  viewport_tabs_->addTab(viewer_, tr("Wizualizacja"));
#else
  viewport_tabs_->addTab(new QLabel(tr("Debug mode")), tr("Wizualizacja"));
#endif
  viewport_tabs_->addTab(scanner_viewer, tr("Podglad"));
  grid->addWidget(viewport_tabs_, 0, 1, 2, 1);
  QWidget* central_widget = new QWidget;
  central_widget->setLayout(grid);
  setCentralWidget(central_widget);

  resize(1366, 768);
  showMaximized();

  QToolBar* patient_toolbar = new QToolBar(this);
  addToolBar(patient_toolbar);

  auto* patient_widget_toolbar = new PatientTreeToolbar(patient_widget_, this);
  addToolBar(patient_widget_toolbar);

  auto scanner_toolbar = new ScannerToolbar(scanner_, this);
  addToolBar(scanner_toolbar);

  ViewerToolbar* viewer_toolbar = new ViewerToolbar(viewer_, this);
  addToolBar(viewer_toolbar);
  connect(viewer_toolbar, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
  connect(patient_widget_, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
  connect(patient_widget_toolbar, SIGNAL(calculateDiff()), this, SLOT(calculateDiff()));
  connect(patient_widget_toolbar, SIGNAL(calculateMirror()), this, SLOT(calculateMirror()));
}

MainWindow::~MainWindow()
{
  // Delete all allocated data
  delete patient_widget_;
}

void MainWindow::openScan(QString filename)
{
  qDebug() << "[INFO] Opening scan: " << filename;
  viewer_->clearMesh();
  CMesh* mesh = new CMesh;
  openMesh(filename, *(mesh));
  viewer_->addMesh("open&show", mesh);
}

void MainWindow::calculateDiff()
{
  auto items = patient_widget_->selectedItems();
  if (items.size() == 2) {
    int idA = PatientTreeItem::getId(items[0]);
    int idB = PatientTreeItem::getId(items[1]);

    ExaminationData data;
    Database::selectExamination(idA, data);
    CMesh meshA;
    openMesh("data/" + data.scan_name, meshA);
    Database::selectExamination(idB, data);
    CMesh meshB;
    openMesh("data/" + data.scan_name, meshB);
    CMesh out;
    computeDifference(meshA, meshB, out);
    auto* pmesh = new CMesh;
    vcg::tri::Append<CMesh, CMesh>::MeshCopy(*(pmesh), meshB);
    viewer_->clearMesh();
    viewer_->addMesh("diff", pmesh);
    viewport_tabs_->setCurrentIndex(1);
  }
}

void MainWindow::calculateMirror()
{
  CMesh* mesh = viewer_->getLastMesh();
  if (mesh) {
    flipMeshXAxis(*mesh);
    viewer_->update();
  }
}
