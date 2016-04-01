#include "MainWindow.h"

#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QCryptographicHash>
#include <QAbstractItemModel>
#include <QSqlError>
#include <QSplitter>

#include "GuiActions.h"
#include "Database.h"
#include "MeshProcessing.h"
#include "ModelHelper.h"
#include "PatientTreeItem.h"
#include "PatientTreeToolbar.h"
#include "ScannerToolbar.h"
#include "ScannerViewer.h"
#include "ViewerToolbar.h"
#include "StackedFormWidget.h"

MainWindow::MainWindow() :
  scanner_(nullptr),
  meshDiffDlg_(nullptr)
{
  main_form_ = nullptr;
  Database db("database.db");
  db.createScheme();
  initModels();

  // Mesh & viewers
  meshStorage_ = new CMeshStorage();
#ifndef _DEBUG
  viewer_ = new ScanViewer(meshStorage_, this, 10);
  miniScanViewer_ = new ScanViewer(meshStorage_, this, 10);
#endif

  SQLTableModelHandler handler(patient_model_, exam_model_, scan_model_, scanDiffModel_);
  StackedFormWidget* stack = new StackedFormWidget(handler);
  patient_widget_ = new PatientTreeWidget(handler, stack, Database::selectPatient());
  connect(patient_widget_, SIGNAL(openScan(QString)), this, SLOT(openScan(QString)));
  QGridLayout* grid = new QGridLayout;
  grid->addWidget(patient_widget_, 0, 0, 2, 1);
  patient_widget_->setMaximumWidth(300);
  scanner_ = new Scanner(this);
  QWidget* viewport = new QWidget;
  ScannerViewer* scanner_viewer = new ScannerViewer(scanner_->numSensor(), this);
  viewport_tabs_ = new QTabWidget(this);
  main_form_ = nullptr;

  QSplitter* splitter = new QSplitter;
  splitter->addWidget(stack);
#ifndef _DEBUG
  splitter->addWidget(miniScanViewer_);
#endif
  viewport_tabs_->addTab(splitter, tr(":forms"));
#ifndef _DEBUG
  viewport_tabs_->addTab(viewer_, tr(":visualization"));
#else
  viewport_tabs_->addTab(new QLabel(tr(":debug_mode")), tr(":visualization"));
#endif
  viewport_tabs_->addTab(scanner_viewer, tr(":quick_view"));
  grid->addWidget(viewport_tabs_, 0, 1, 2, 1);
  QWidget* central_widget = new QWidget;
  central_widget->setLayout(grid);
  setCentralWidget(central_widget);

  resize(800, 600);
  //showMaximized();

  auto* patient_widget_toolbar = new PatientTreeToolbar(patient_widget_, this);
  addToolBar(patient_widget_toolbar);
  auto scanner_toolbar = new ScannerToolbar(this);
  addToolBar(scanner_toolbar);
  scanner_toolbar->showNumSensor(scanner_->numSensor());

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
    qDebug() << "[Info] Reconstruction stoped. Mesh saved in: " << meshFilePath;
    auto item = patient_widget_->currentItem();
    if (PatientTreeItem::isExamination(item)) {
      int examID = PatientTreeItem::getId(item);
      int scanRowNum = scan_model_->rowCount();
      scan_model_->insertRow(scanRowNum);
      scan_model_->setData(scan_model_->index(scanRowNum, scan_model_->fieldIndex("exam_id")), examID);
      scan_model_->setData(scan_model_->index(scanRowNum, scan_model_->fieldIndex("filename")), meshFilePath);
      scan_model_->submitAll();
      int scanID = scan_model_->data(scan_model_->index(scanRowNum, 0)).toInt();
      stack->switchTo(StackedFormWidget::SCAN_FORM, scanID);
    }
    else {
      qDebug() << "[WARN] Trying to save scan when non-examination item is selected.";
    }
  });

  // Scanner -> ScannerViewer
  connect(scanner_, SIGNAL(sendImages(QList<ImageData>, QList<ImageData>)), scanner_viewer, SLOT(displayImages(QList<ImageData>, QList<ImageData>)));
  // Scanner -> ScannerToolbar
  

  // StackedFormWidget -> ...
  connect(stack, &StackedFormWidget::displayScan, [=](int scanID, int diffId) {
    // Load mesh first
    if (!meshStorage_->hasMesh(scanID)) {
      QString filename;
      if (!ModelHelper::findScanFilename(scanID, filename)) {
        qDebug() << "[ERROR@MainWindow->OnDisplayScan] Couldn't find scan with ID" << scanID;
        return;
      };
      if (filename.isEmpty()) return;
      meshStorage_->loadMesh(filename, scanID);
    }
    // Load quality map
    if (!meshStorage_->hasQualityMap(diffId)) {
      QString filename;
      if (!ModelHelper::findScanDiffFilename(diffId, filename)) {
        qDebug() << "[ERROR@MainWindow->OnDisplayScan] Couldn't find scan diff with ID" << diffId;
        return;
      }
      if (filename.isEmpty()) return;
      meshStorage_->loadQualityMap(filename, diffId);
    }
    miniScanViewer_->show(scanID, diffId);
    //viewport_tabs_->setCurrentIndex(1);
  });

  // PatientTreeWidget -> ScannerToolbar
  connect(patient_widget_, &PatientTreeWidget::itemClicked, [=] (QTreeWidgetItem* item, int column) {
    scanner_toolbar->setEnabled(PatientTreeItem::isExamination(item));
  });

  auto onDisplayScan = [=](ScanViewer::ID viewerID, int scanID) {
    // Load mesh first
    if (!meshStorage_->hasMesh(scanID)) {
      QString filename;
      if (!ModelHelper::findScanFilename(scanID, filename)) {
        qDebug() << "[ERROR@MainWindow->OnDisplayScan] Couldn't find scan with ID" << scanID;
        return;
      };
      qDebug() << "[INFO@MainWindow->OnDisplayScan] Loading scan" << filename;
      if (filename.isEmpty()) return;
      meshStorage_->loadMesh(filename, scanID);
    }
    switch (viewerID) {
    case ScanViewer::FULL_VIEWER:
      viewer_->show(scanID);
      viewport_tabs_->setCurrentIndex(1);
    case ScanViewer::MINI_VIEWER:
      miniScanViewer_->show(scanID);
      break;
    }
  };
  connect(patient_widget_, &PatientTreeWidget::displayScan, onDisplayScan);
  // PatientTreeToolbar -> MeshViewer
  connect(patient_widget_toolbar, &PatientTreeToolbar::displayScan, onDisplayScan);
}

MainWindow::~MainWindow()
{
  // Delete all allocated data
  delete patient_widget_;
}

void MainWindow::openScan(QString filename)
{
  qDebug() << "[INFO] Opening scan: " << filename;
  viewer_->clear();
  CMesh* mesh = new CMesh;
  MeshProcessing::loadMeshFromFile<CMesh>(filename, mesh);
  viewer_->insert(mesh);
}

void MainWindow::calculateDiff()
{
  delete meshDiffDlg_;
  auto item = patient_widget_->currentItem();
  if (item && PatientTreeItem::isScan(item)) {
    auto parent = item->parent();
    QMap<int, QString> options;
    for (int i = 0; i < parent->childCount(); ++i) {
      auto child = parent->child(i);
      QString entry = child->text(0);
      options.insert(PatientTreeItem::getId(child), child->text(0));
    }
    meshDiffDlg_ = new MeshDifferenceDlg(this, options, PatientTreeItem::getId(item));
    connect(meshDiffDlg_, &MeshDifferenceDlg::calculateDiff, [=](int refScanID, int compScanID) {
      qDebug() << "RefScanID: " << refScanID << "; compScanID: " << compScanID;
      int scanDiffRow = scanDiffModel_->rowCount();
      scanDiffModel_->insertRow(scanDiffRow);
      scanDiffModel_->setData(scanDiffModel_->index(scanDiffRow, scanDiffModel_->fieldIndex("ref_id")), refScanID);
      scanDiffModel_->setData(scanDiffModel_->index(scanDiffRow, scanDiffModel_->fieldIndex("comp_id")), compScanID);
      QByteArray time = QTime::currentTime().toString().toLocal8Bit();
      QString filePath = QString(QCryptographicHash::hash(time, QCryptographicHash::Md5).toHex());
      QString fullMeshFilePath = "data/diff_" + filePath;
      // generate quality map and save it
      int scansNum = scan_model_->rowCount();
      QString fileName;
      for (int i = 0; i < scansNum; ++i) {
        auto record = scan_model_->record(i);
        if (record.value("id").toInt() == refScanID) {
          fileName = record.value("filename").toString();
          break;
        }
      }
      // load mesh for it
      CMesh mesh;
      MeshProcessing::loadMeshFromFile<CMesh>(fileName, &mesh);
      // save it
      QVector<float> quality;
      generateRandomQualityForMesh(mesh, quality);
      saveQualityToFile(fullMeshFilePath, quality);
      scanDiffModel_->setData(scanDiffModel_->index(scanDiffRow, scanDiffModel_->fieldIndex("filename")), fullMeshFilePath);
      scanDiffModel_->submitAll();
    });
    meshDiffDlg_->show();
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
  CMesh* mesh = viewer_->last();
  if (mesh) {
    flipMeshXAxis(*mesh);
    viewer_->update();
  }
}

void MainWindow::initModels()
{
  patient_model_ = new QSqlTableModel;
  patient_model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  patient_model_->setTable("patient");
  patient_model_->select();
  exam_model_ = new QSqlTableModel;
  exam_model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  exam_model_->setTable("examination");
  exam_model_->select();
  scan_model_ = new QSqlTableModel;
  scan_model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  scan_model_->setTable("scan");
  scan_model_->select();
  scanDiffModel_ = new QSqlTableModel;
  scanDiffModel_->setTable("scan_diff");
  scanDiffModel_->select();
}
