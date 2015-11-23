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

#include <vcg/complex/algorithms/update/position.h>

using namespace RecFusion;

MainWindow::MainWindow() :
  m_timer(0),
  m_calibMessageBox(0),
  m_reconstruct(false),
  m_calibrate(false),
  m_rec(0),
  num_sensor_(0),
  sensor_data_(nullptr),
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
  grid->addWidget(viewer_, 1, 1);
#endif

  stacked_layout_->setCurrentIndex(0);
  QWidget* viewport = new QWidget;
  viewport->setLayout(stacked_layout_);
  grid->addWidget(viewport, 0, 1);
  QWidget* central_widget = new QWidget;
  central_widget->setLayout(grid);
  setCentralWidget(central_widget);

  resize(1366, 768);
  showMaximized();

  // Initialize pointers to zero
  m_sensor[0] = m_sensor[1] = m_sensor[2] = nullptr;

  // Output RecFusion SDK version
  qDebug() << "Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion();

  // Load license file
  bool ok = RecFusionSDK::setLicenseFile("License.dat");
  if (!ok)
    qDebug() << "Invalid RecFusion license. Export will be disabled.";

  // Instantiate sensor objects
  m_sensor[0] = new Sensor();
  m_sensor[1] = new Sensor();
  m_sensor[2] = new Sensor();

#ifndef _DEBUG
  num_sensor_ = m_sensor[0]->deviceCount();
  qDebug() << QString("Number of sensors connected: ") + QString::number(num_sensor_);

  for (unsigned i = 0; i < num_sensor_; ++i) {
    ok = m_sensor[i]->open(i);
    if (!ok)
    {
      QMessageBox::warning(this, "Initialization", "Couldn't open sensor num. " + QString::number(i) + ". Exiting.");
      QTimer::singleShot(0, this, SLOT(close()));
    }
    else
    {
      // Get sensor properties
      sensor_data_ = new SensorData(*m_sensor[i]);
      sensors_data_.push_back(sensor_data_);
      int w = m_sensor[i]->depthWidth();
      int h = m_sensor[i]->depthHeight();
      m_imgLabel[i]->resize(w, h);
    }
  }
#endif

  // Create message box for calibration dialog
  m_calibMessageBox = new QMessageBox(this);
  m_calibMessageBox->setIcon(QMessageBox::Information);
  m_calibMessageBox->setWindowTitle("Calibration");
  m_calibMessageBox->setText("Press OK to capture calibration frame");
  m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
  connect(m_calibMessageBox,SIGNAL(accepted()),this,SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(finished(int)), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(rejected()), this, SLOT(performCalibration()));

  QToolBar* patient_toolbar = new QToolBar(this);
  addToolBar(patient_toolbar);

  // Add patient
  QAction* add_patient = new QAction("Dodaj pacjenta", this);
  addAction(add_patient);
  connect(add_patient, SIGNAL(triggered()), this, SLOT(addPatient()));
  patient_toolbar->addAction(add_patient);
  // Add examintation
  QAction* add_examination = new QAction("Dodaj badanie", this);
  addAction(add_examination);
  connect(add_examination, SIGNAL(triggered()), this, SLOT(addExam()));
  //connect(add_examination, SIGNAL(triggered()), patient_widget_, SLOT(showAddExaminationDialog()));
  patient_toolbar->addAction(add_examination);
  // Remove selected item
  QAction* remove_selected = new QAction("Usun zaznaczony element", this);
  addAction(remove_selected);
  connect(remove_selected, SIGNAL(triggered()), patient_widget_, SLOT(removeCurrentItem()));
  patient_toolbar->addAction(remove_selected);

  // DEBUG: show index
  QAction* show_index = new QAction("Pokaz indeks", this);
  addAction(show_index);
  connect(show_index, SIGNAL(triggered()), patient_widget_, SLOT(showIndex()));
  patient_toolbar->addAction(show_index);

  QToolBar* toolbar = new QToolBar(this);
  addToolBar(toolbar);

  // Create actions for calibrating and reconstructing
  QAction* a;
  a = new QAction("Calibrate",this);
  a->setShortcut(QKeySequence("F9"));
  a->setDisabled(num_sensor_ == 0);
  connect(a,SIGNAL(triggered()),this,SLOT(calibrate()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Save Calibration",this);
  a->setShortcut(QKeySequence("F10"));
  a->setDisabled(num_sensor_ == 0);
  connect(a,SIGNAL(triggered()),this,SLOT(saveCalibration()));
  addAction(a);
  toolbar->addAction(a);

  a = new QAction("Load calibration",this);
  a->setShortcut(QKeySequence("F11"));
  a->setDisabled(num_sensor_ == 0);
  connect(a,SIGNAL(triggered()),this,SLOT(loadCalibration()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Start Reconstruction",this);
  a->setShortcut(QKeySequence("F5"));
  a->setDisabled(num_sensor_ == 0);
  connect(a,SIGNAL(triggered()),this,SLOT(startReconstruction()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Stop Reconstruction", this);
  a->setShortcut(QKeySequence("F6"));
  a->setDisabled(num_sensor_ == 0);
  connect(a, SIGNAL(triggered()), this, SLOT(stopReconstruction()));
  addAction(a);
  toolbar->addAction(a);

  // Open 3D model
  /*a = new QAction("Open 3D Model", this);
  connect(a, SIGNAL(triggered()), this, SLOT(open3DModel(TODO)));
  addAction(a);
  toolbar->addAction(a);*/

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

  // Show entire scene
  a = new QAction("Pokaz cala scene", this);
  connect(a, SIGNAL(triggered()), this, SLOT(showScene()));
  addAction(a);
  toolbar->addAction(a);

  m_timer = new QTimer(this);
  connect(m_timer,SIGNAL(timeout()),this,SLOT(processFrames()));
  m_timer->start(50);
}

MainWindow::~MainWindow()
{
  // Close and delete sensors
  for (unsigned i = 0; i < num_sensor_; ++i) {
    m_sensor[i]->close();
    delete m_sensor[i];
  }

  // Delete all allocated data
  delete m_timer;
  delete m_rec;
  delete sensor_data_;
  delete patient_widget_;
}


void MainWindow::calibrate()
{
  // Show message box to let user choose correct frame before starting calibration
  m_calibrate = false;

  m_calibMessageBox->setText("Press OK to capture calibration frames.");
  m_calibMessageBox->show();
}


void MainWindow::performCalibration()
{
  unsigned num_sensor = static_cast<int>(sensors_data_.size());
  for (unsigned i = 0; i < num_sensor; ++i) {
    sensors_data_[i]->ResetT();
  }

  // Create calibration object for two sensors
  Calibration calib;
  calib.init(num_sensor);

  // Single-sided calibration
  //calib.setMarker(100, 190);
  calib.setMarker(100, 370);

  //// Two-sided calibration with a marker board of thickness 12 mm. The calibration coordinate system origin is in the middle of the board.
  //// Transformation from the first marker coordinate system to the calibration coordinate system.
  //// The z-Axis is always perpendicular to the marker surface pointing toward the viewer.
  //// When placing the marker such that the marker id printed on the lower left of the marker
  //// the x-axis extends from the center of the marker towards the bottom (where the marker id is printed),
  //// the y-axis extends to the right, the origin of the marker coordinate system is at the center of the marker.
  //
  //// Transformation from first marker coordinate system to calibration coordinate system (move 6 mm in z-direction to the center of the board). Matrix is column-major-order.
  //double T1_[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -6, 1 };
  //Mat4 T1(T1_);
  //calib.setMarker(400, 190, &T1)

  //// Transformation from second marker coordinate system to calibration coordinate system (rotate 180° around x-axis and move 6 mm in z-direction to the center of the board). Matrix is column-major-order.
  //double T2_[] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, -6, 1 };
  //Mat4 T2(T2_);
  //calib.setMarker(500, 190, &T2);

  bool ok = false;

  // Try to run calibration until it succeeds but at most 10 times
  for (int i = 0; i < 10; ++i)
  {
    // Reset valid flag for capturing calibration images
    for (unsigned i = 0; i < num_sensor; ++i) {
      auto data = sensors_data_[i];
      data->calib_image_valid = false;
    }
    // Setting m_calibrate to true, instructs the capture loop to capture calibration images
    m_calibrate = true;

    // Wait until calibration images for both sensors have been captured
    bool waiting = true;
    while (waiting) {
      for (unsigned i = 0; i < num_sensor; ++i) {
        auto data = sensors_data_[i];
        waiting = waiting && !data->IsCalibrated();
      }
      QCoreApplication::processEvents();
    }

    // Stop calibration frame capturing
    m_calibrate = false;

    // Pass captured images to calibration
    for (unsigned i = 0; i < num_sensor; ++i) {
      auto data = sensors_data_[i];
      calib.setImage(i, *(data->calib_depth_image), *(data->calib_color_image), data->K, data->K);
    }

    // Run calibration
    ok = calib.calibrate();

    if (ok)
      break;
  }

  if (ok)
  {
    // Retrieve sensor transformation if calibration succeeded
    for (unsigned i = 0; i < num_sensor_; ++i) {
      auto data = sensors_data_[i];
      calib.getTransformation(i, data->T);
      std::cout << Mat4ToString(data->T) << std::endl;
    }
    QMessageBox::information(this, "Calibration", "Calibration succeeded");
  }
  else
  {
    QMessageBox::information(this, "Calibration", "Calibration failed");
  }
}


void MainWindow::saveCalibration()
{
  QString filename = QFileDialog::getSaveFileName(this,"Save calibration");
  if (filename.isEmpty()) return;

  // Save calibrations to file as 4x4 matrices in row-major order
  std::ofstream out(filename.toStdString());
  for (int i = 0; i < 2; ++i)
  {
    for (int r = 0; r < 4; ++r)
    {
      for (int c = 0; c < 4; ++c)
        out << m_sensorT[i](r,c) << " ";
      out << std::endl;
    }
  }
  out.close();
}


void MainWindow::loadCalibration()
{
  QString filename = QFileDialog::getOpenFileName(this,"Load calibration");
  if (filename.isEmpty()) return;
  std::ifstream in(filename.toStdString());
  if (!in.is_open() || !in.good())
  {
    QMessageBox::information(this,"Load calibration","Couldn't open calibration file");
    return;
  }

  // Load calibration from file
  double tmp[2][16];
  for (int i = 0; i < 2; ++i)
  {
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        in >> tmp[i][c * 4 + r];
  }
  if (in.fail())
  {
    QMessageBox::information(this,"Load calibration","Error reading calibration file");
    return;
  }
  in.close();

  for (int i = 0; i < 2; ++i)
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        m_sensorT[i](r, c) = tmp[i][c * 4 + r];
}


void MainWindow::startReconstruction()
{
  m_reconstruct = false;

  // Delete reconstruction object if there is one
  delete m_rec;
  m_rec = 0;

  // Set reconstruction parameters for two sensors
  ReconstructionParams params(num_sensor_);

  // Set per-sensor parameters
  for (unsigned i = 0; i < num_sensor_; ++i)
  {
    auto data = sensors_data_[i];
    auto depth_image = data->depth_image;
    auto color_image = data->color_image;
#ifndef _DEBUG
    params.setImageSize(color_image->width(), color_image->height(), depth_image->width(), depth_image->height(), i);
#endif
    params.setIntrinsics(data->K, i);
  }

  //params.setVolumePosition(Vec3(0, 200, 1650));
  //params.setVolumeResolution(Vec3i(204, 512, 204));
  //params.setVolumeSize(Vec3(800, 2000, 800));

  // Set volume parameters
  params.setVolumePosition(Vec3(0, 0, 1000));
  params.setVolumeResolution(Vec3i(256, 256, 256));
  params.setVolumeSize(Vec3(1000, 1000, 1000)); // x3 100

  // Create reconstruction object
  m_rec = new Reconstruction(params);

  // Start reconstruction
  m_reconstruct = true;
}


void MainWindow::stopReconstruction()
{
  // Stop reconstruction
  m_reconstruct = false;
  if (!m_rec)
    return;

  // Get reconstructed mesh
  Mesh* mesh = new Mesh;
  bool ok = m_rec->getMesh(mesh);
  // TODO: display reconstructed mesh

  // Delete reconstruction object
  delete m_rec;
  m_rec = 0;
  if (!ok)
  {
    std::cout << "Couldn't retrieve mesh" << std::endl;
    return;
  }
  /*
  std::cout << "Reconstructed mesh (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
  // Save mesh to file
  ok = mesh.save("mesh.ply", Mesh::PLY);
  if (ok)
    std::cout << "Saved mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
    */
  // TODO: view mesh
}


void MainWindow::processFrames()
{
  for (unsigned i = 0; i < sensors_data_.size(); ++i) {
    if (!sensors_data_[i]->HasRegularImages())
      return;
  }

  // Grab images from sensor
  bool ok[3];
  for (unsigned i = 0; i < num_sensor_; ++i) {
    ok[i] = m_sensor[i]->readImage(*(sensors_data_[i]->depth_image), *(sensors_data_[i]->color_image), 40);
  }

  // Process images
  for (unsigned i = 0; i < num_sensor_; ++i)
  {
    if (!ok[i]) continue;
    sensor_data_ = sensors_data_[i];

    // Get image size
    int w = sensor_data_->color_image->width();
    int h = sensor_data_->color_image->height();

    if (m_reconstruct && m_rec)
    {
      // Add frame to reconstruction
      bool status = false;
#ifndef _DEBUG
      bool ret = m_rec->addFrame(
        i,
        *sensor_data_->depth_image,
        *sensor_data_->color_image,
        &status,
        sensor_data_->scene_image,
        0,
        &sensor_data_->T);
#else
      bool ret = false;
#endif

      if (ret && status)
      {
        // Display rendering of current reconstruction when tracking succeeded
        QImage image(sensor_data_->scene_image->data(), w, h, QImage::Format_RGB888);
        m_recLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(w, h, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
      }
    }
    else if (m_calibrate)
    {
      // Save calibration frame
      for (unsigned i = 0; i < num_sensor_; ++i) {
        auto data = sensors_data_[i];
        memcpy(data->calib_color_image->data(), data->color_image->data(), w * h * 3);
        memcpy(data->calib_depth_image->data(), data->depth_image->data(), w * h * 2);
        data->calib_image_valid = true;
      }
    }

    // Display captured images in GUI
    QImage image(sensor_data_->color_image->data(), w, h, QImage::Format_RGB888);
    m_imgLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(w/2.0, h/2.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
  }

  // Update GUI
  update();
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
