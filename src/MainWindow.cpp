#include "MainWindow.h"

#include <QDebug>
#include <QAction>
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

  scanner_ = new Scanner(this);
  stacked_layout_->setCurrentIndex(0);
  QWidget* viewport = new QWidget;
  viewport->setLayout(stacked_layout_);
  ScannerViewer* scanner_viewer = new ScannerViewer(scanner_, this);
  viewport_tabs_ = new QTabWidget(this);
  viewport_tabs_->addTab(viewport, tr("Formatki"));
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

  auto* patient_widget_toolbar = new PatientWidgetToolbar(patient_widget_, this);
  addToolBar(patient_widget_toolbar);

  auto scanner_toolbar = new ScannerToolbar(scanner_, this);
  addToolBar(scanner_toolbar);

  ViewerToolbar* viewer_toolbar = new ViewerToolbar(viewer_, this);
  addToolBar(viewer_toolbar);
  connect(viewer_toolbar, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
  connect(patient_widget_, SIGNAL(showTabWithIndex(int)), viewport_tabs_, SLOT(setCurrentIndex(int)));
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
  auto items = patient_widget_->selectedItems();
  if (items.size() == 2) {
    int idA = PatientWidgetItem::getId(items[0]);
    int idB = PatientWidgetItem::getId(items[1]);

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
  //flipMeshXAxis(*(viewer_->ma));
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
    viewport_tabs_->setCurrentIndex(0);
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
