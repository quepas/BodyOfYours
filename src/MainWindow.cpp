#include "MainWindow.h"

#include <QDebug>
#include <QtWidgets/QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QStackedLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMenu>

#include <QtCore/QCoreApplication>
#include <QtCore/QElapsedTimer>
#include <QtCore/QTimer>
#include <QtCore/QMutex>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>

#include "RecFusionUtils.h"
#include "Database.h"
#include "MeshProcessing.h"
#include "patientform.h"
#include "examinationform.h"
#include "PatientWidgetItem.h"
#include "PatientWidgetToolbar.h"
#include "Scanner.h"
#include "ScannerToolbar.h"
#include "ScannerViewer.h"

#include <vcg/complex/algorithms/update/position.h>

using namespace RecFusion;

MainWindow::MainWindow() :
  m_calibMessageBox(0),
  m_reconstruct(false),
  m_calibrate(false),
  m_rec(0),
  num_sensor_(0),
  sensor_data_(nullptr),
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

  // Add patient
  QAction* add_patient = new QAction("Dodaj pacjenta", this);
  addAction(add_patient);
  connect(add_patient, SIGNAL(triggered()), this, SLOT(addPatient()));
  patient_toolbar->addAction(add_patient);
  // Add examintation
  QAction* add_examination = new QAction("Dodaj badanie", this);
  addAction(add_examination);
  connect(add_examination, SIGNAL(triggered()), this, SLOT(addExam()));
  patient_toolbar->addAction(add_examination);

  QToolBar* toolbar = new QToolBar(this);
  addToolBar(toolbar);

  QAction* a;
  // Compute diff
  a = new QAction("Wylicz roznice", this);
  connect(a, SIGNAL(triggered()), this, SLOT(calculateDiff()));
  addAction(a);
  toolbar->addAction(a);

  // Flip by X axis
  a = new QAction("Wylicz odbicie lustrzane", this);
  connect(a, SIGNAL(triggered()), this, SLOT(calculateMirror()));
  addAction(a);
  toolbar->addAction(a);
}

MainWindow::~MainWindow()
{
  // Close and delete sensors
  for (unsigned i = 0; i < num_sensor_; ++i) {
    m_sensor[i]->close();
    delete m_sensor[i];
  }

  // Delete all allocated data
  delete m_rec;
  delete sensor_data_;
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
