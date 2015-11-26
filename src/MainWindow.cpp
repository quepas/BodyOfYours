#include "MainWindow.h"

#include <QDebug>
#include <QAction>
#include <QTabWidget>
#include <QToolBar>

#include "Database.h"
#include "MeshProcessing.h"
#include "PatientWidgetItem.h"
#include "PatientWidgetToolbar.h"
#include "ScannerToolbar.h"
#include "ScannerViewer.h"
#include "ViewerToolbar.h"

MainWindow::MainWindow() :
  scanner_(nullptr),
  patient_form_(new PatientForm),
  exam_form_(new ExaminationForm)
#ifndef _DEBUG
  ,
  viewer_(new Viewer())
#endif
{
  Database db("database.db");
  db.createScheme();
  patient_widget_ = new PatientWidget(Database::selectPatient());
  connect(patient_widget_, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)), this, SLOT(onItemSelected(QTreeWidgetItem*, QTreeWidgetItem*)));
  connect(patient_widget_, SIGNAL(openScan(QString)), this, SLOT(openScan(QString)));

  connect(patient_form_, SIGNAL(savePatient(PatientData)), patient_widget_, SLOT(onSavePatient(PatientData)));
  connect(patient_form_, SIGNAL(deletePatient()), patient_widget_, SLOT(onDeletePatient()));
  connect(exam_form_, SIGNAL(saveExam(ExaminationData)), patient_widget_, SLOT(onSaveExamination(ExaminationData)));

  QGridLayout* grid = new QGridLayout;
  grid->addWidget(patient_widget_, 0, 0, 2, 1);
  patient_widget_->setMaximumWidth(300);
  stacked_layout_ = new QStackedLayout;
  stacked_layout_->addWidget(patient_form_);
  stacked_layout_->addWidget(exam_form_);
#ifndef _DEBUG
  //grid->addWidget(viewer_, 1, 1);
#endif

  scanner_ = new Scanner(this);
  stacked_layout_->setCurrentIndex(0);
  QWidget* viewport = new QWidget;
  viewport->setLayout(stacked_layout_);
  ScannerViewer* scanner_viewer = new ScannerViewer(scanner_, this);
  QTabWidget* viewport_tabs = new QTabWidget(this);
  viewport_tabs->addTab(viewport, tr("Formatki"));
  viewport_tabs->addTab(viewer_, tr("Wizualizacja"));
  viewport_tabs->addTab(scanner_viewer, tr("Podglad"));
  grid->addWidget(viewport_tabs, 0, 1, 2, 1);
  QWidget* central_widget = new QWidget;
  central_widget->setLayout(grid);
  setCentralWidget(central_widget);

  resize(1366, 768);
  showMaximized();

  QToolBar* patient_toolbar = new QToolBar(this);
  addToolBar(patient_toolbar);

  auto* patient_widget_toolbar = new PatientWidgetToolbar(patient_widget_, this);
  addToolBar(patient_widget_toolbar);

  auto scanner_toolbar = new ScannerToolbar(scanner_, this);
  addToolBar(scanner_toolbar);

  auto viewer_toolbar = new ViewerToolbar(viewer_, this);
  addToolBar(viewer_toolbar);

  // Add patient
  connect(patient_widget_toolbar, SIGNAL(addNewPatient()), this, SLOT(addPatient()));
  connect(patient_widget_toolbar, SIGNAL(addNewExamination()), this, SLOT(addExam()));
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
  QString n_ref = "data/A.ply";
  QString n_mesh = "data/B.ply";

  CMesh ref, mesh;
  openMesh(n_ref, ref);
  openMesh(n_mesh, mesh);
  CMesh out;
  computeDifference(ref, mesh, out);
  auto* pmesh = new CMesh;
  vcg::tri::Append<CMesh, CMesh>::MeshCopy(*(pmesh), mesh);
  viewer_->clearMesh();
  viewer_->addMesh("diff", pmesh);
}

void MainWindow::calculateMirror()
{
  //flipMeshXAxis(*(viewer_->cmesh_));
}

void MainWindow::onItemSelected(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
  if (current) {
    QString text = current->text(0);
    int id = PatientWidgetItem::getId(current);
    qDebug() << "Currently seleted ID: " << id;
    if (PatientWidgetItem::isPatient(current)) {
      stacked_layout_->setCurrentIndex(0);
      PatientForm* patient_form_ = dynamic_cast<PatientForm*>(stacked_layout_->currentWidget());
      PatientData patient;
      Database::selectPatient(id, patient);
      patient_form_->setData(patient);
      patient_form_->setShowState(true);
    }
    else if (PatientWidgetItem::isExamination(current)) {
      stacked_layout_->setCurrentIndex(1);
      exam_form_ = dynamic_cast<ExaminationForm*>(stacked_layout_->currentWidget());
      ExaminationData exam;
      Database::selectExamination(id, exam);
      exam_form_->setData(exam);
      exam_form_->setDisabled(true);
    }
  }
}

void MainWindow::addPatient()
{
  qDebug() << "MainWindow::addPatient()";
  stacked_layout_->setCurrentIndex(0);
  patient_form_->setDisabled(false);
  patient_form_->clear();
}

void MainWindow::addExam()
{
  qDebug() << "MainWindow::addExam()";
  stacked_layout_->setCurrentIndex(1);
  exam_form_->setDisabled(false);
  exam_form_->clear();
}
