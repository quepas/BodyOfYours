#include "MainWindow.h"

#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QAbstractItemModel>

#include "GuiActions.h"
#include "Database.h"
#include "MeshDifferenceDlg.h"
#include "MeshProcessing.h"
#include "PatientTreeItem.h"
#include "PatientTreeToolbar.h"
#include "ScannerToolbar.h"
#include "ScannerViewer.h"
#include "ViewerToolbar.h"
#include "StackedFormWidget.h"

MainWindow::MainWindow() :
  scanner_(nullptr)
#ifndef _DEBUG
  ,
  viewer_(new MeshViewer())
#endif
{
  main_form_ = nullptr;
  Database db("database.db");
  db.createScheme();
  initModels();
  StackedFormWidget* stack = new StackedFormWidget(patient_model_, exam_model_, scan_model_);
  patient_widget_ = new PatientTreeWidget(patient_model_, exam_model_, stack, Database::selectPatient());
  connect(patient_widget_, SIGNAL(openScan(QString)), this, SLOT(openScan(QString)));
  QGridLayout* grid = new QGridLayout;
  grid->addWidget(patient_widget_, 0, 0, 2, 1);
  patient_widget_->setMaximumWidth(300);
  scanner_ = new Scanner(this);
  QWidget* viewport = new QWidget;
  ScannerViewer* scanner_viewer = new ScannerViewer(scanner_, this);
  viewport_tabs_ = new QTabWidget(this);
  main_form_ = nullptr;
  viewport_tabs_->addTab(stack, tr("Formatki"));
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

  resize(800, 600);
  //showMaximized();

  QToolBar* patient_toolbar = new QToolBar(this);
  addToolBar(patient_toolbar);

  auto* patient_widget_toolbar = new PatientTreeToolbar(patient_widget_, this);
  addToolBar(patient_widget_toolbar);

  auto scanner_toolbar = new ScannerToolbar(this);
  addToolBar(scanner_toolbar);

  ViewerToolbar* viewer_toolbar = new ViewerToolbar(viewer_, this);
  addToolBar(viewer_toolbar);
  connect(viewer_toolbar, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
  connect(patient_widget_, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
  connect(patient_widget_toolbar, SIGNAL(calculateDiff()), this, SLOT(calculateDiff()));
  connect(patient_widget_toolbar, SIGNAL(calculateMirror()), this, SLOT(calculateMirror()));

  ActionHub::addAction(new ActionAddNewPatient(this, stack));
  ActionHub::addAction(new ActionAddNewExamination(this, stack));
  ActionHub::addAction(new ActionAddNewScan(this, stack));
  ActionHub::addAction(new ActionDeleteCurrentItem(this, patient_widget_));
  ActionHub::addAction(new ActionStartReconstruction(this, scanner_));
  ActionHub::addAction(new ActionStopReconstruction(this, scanner_));
  ActionHub::addAction(new ActionMeshViewerClear(this, viewer_));

  // ScannerToolbar -> Scanner
  connect(scanner_toolbar, SIGNAL(startReconstruction()), scanner_, SLOT(startReconstruction()));
  connect(scanner_toolbar, SIGNAL(stopReconstruction(QString)), scanner_, SLOT(stopReconstruction()));

  // ScannerToolbar -> StackedFormWidget
  connect(scanner_toolbar, &ScannerToolbar::stopReconstruction, [=](QString meshFilePath) {
    qDebug() << "[Info] Reconstruction stoped. Mesh save in: " << meshFilePath;
    stack->switchTo(StackedFormWidget::SCAN_FORM);
  });

  // PatientTreeWidget -> ScannerToolbar
  connect(patient_widget_, &PatientTreeWidget::itemClicked, [=] (QTreeWidgetItem* item, int column) {
    scanner_toolbar->setEnabled(PatientTreeItem::isExamination(item));
  });
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
  auto item = patient_widget_->currentItem();
  if (item && PatientTreeItem::isExamination(item)) {
    qDebug() << item->text(0);
    auto parent = item->parent();
    qDebug() << "\tparent: " << parent->text(0);
    QStringList options;
    for (int i = 0; i < parent->childCount(); ++i) {
      auto child = parent->child(i);
      if (child != item) {
        QString entry = parent->child(i)->text(0);
        qDebug() << "\t\tchild: " << entry;
        options.append(entry);
      }
    }
    (new MeshDifferenceDlg(this, options))->show();
  }
  /*
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
  }*/
}

void MainWindow::calculateMirror()
{
  CMesh* mesh = viewer_->getLastMesh();
  if (mesh) {
    flipMeshXAxis(*mesh);
    viewer_->update();
  }
}

void MainWindow::initModels()
{
  patient_model_ = new QSqlTableModel;
  patient_model_->setTable("patient");
  patient_model_->select();
  exam_model_ = new QSqlTableModel;
  exam_model_->setTable("examination");
  exam_model_->select();
  scan_model_ = new QSqlTableModel;
  scan_model_->setTable("scan");
  scan_model_->select();
}
