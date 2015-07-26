#include "MainWindow.h"

#include <QtWidgets/QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QToolBar>

#include <QtCore/QCoreApplication>
#include <QtCore/QElapsedTimer>
#include <QtCore/QTimer>
#include <QtCore/QMutex>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>

#include "RecFusionUtils.h"

using namespace RecFusion;

MainWindow::MainWindow() :
  m_timer(0),
  m_calibMessageBox(0),
  m_reconstruct(false),
  m_calibrate(false),
  m_rec(0),
  sensor_num_(0)
{
  // Create main window GUI
  m_imgLabel[0] = new QLabel;
  m_imgLabel[1] = new QLabel;
  m_imgLabel[2] = new QLabel;
  m_recLabel[0] = new QLabel;
  m_recLabel[1] = new QLabel;
  m_recLabel[2] = new QLabel;
  QGridLayout* l = new QGridLayout;
  l->addWidget(m_imgLabel[0], 0, 0);
  l->addWidget(m_imgLabel[1], 0, 1);
  l->addWidget(m_imgLabel[2], 0, 2);
  l->addWidget(m_recLabel[0], 1, 0);
  l->addWidget(m_recLabel[1], 1, 1);
  l->addWidget(m_recLabel[2], 1, 2);

  QWidget* wt = new QWidget;
  wt->setLayout(l);
  setCentralWidget(wt);

  resize(1366, 768);
  showMaximized();

  // Initialize pointers to zero
  m_colorImg[0] = m_colorImg[1] = m_colorImg[2] = nullptr;
  m_depthImg[0] = m_depthImg[1] = m_depthImg[2] = nullptr;
  m_sceneImg[0] = m_sceneImg[1] = m_sceneImg[2] = nullptr;
  m_calibImgColor[0] = m_calibImgColor[1] = m_calibImgColor[2] = nullptr;
  m_calibImgDepth[0] = m_calibImgDepth[1] = m_calibImgDepth[2] = nullptr;
  m_calibImgValid[0] = m_calibImgValid[1] = m_calibImgValid[2] = false;
  m_sensor[0] = m_sensor[1] = m_sensor[2] = nullptr;

  // Output RecFusion SDK version
  std::cout << "Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion() << std::endl;

  // Load license file
  bool ok = RecFusionSDK::setLicenseFile("License.dat");
  if (!ok)
    std::cout << "Invalid RecFusion license. Export will be disabled." << std::endl;

  // Instantiate sensor objects
  m_sensor[0] = new Sensor();
  m_sensor[1] = new Sensor();
  m_sensor[2] = new Sensor();

  sensor_num_ = m_sensor[0]->deviceCount();
  QMessageBox::information(this, "Sensors connected", QString("Number of sensors connected: ") + QString::number(sensor_num_));

  for (unsigned i = 0; i < sensor_num_; ++i) {
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
      int w = m_sensor[i]->width();
      int h = m_sensor[i]->height();
      m_imgLabel[i]->resize(w, h);
    }
  }

  // Create message box for calibration dialog
  m_calibMessageBox = new QMessageBox(this);
  m_calibMessageBox->setIcon(QMessageBox::Information);
  m_calibMessageBox->setWindowTitle("Calibration");
  m_calibMessageBox->setText("Press OK to capture calibration frame");
  m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
  connect(m_calibMessageBox,SIGNAL(accepted()),this,SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(finished(int)), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(rejected()), this, SLOT(performCalibration()));

  QToolBar* toolbar = new QToolBar(this);
  addToolBar(toolbar);

  // Create actions for calibrating and reconstructing
  QAction* a;
  a = new QAction("Calibrate",this);
  a->setShortcut(QKeySequence("F9"));
  connect(a,SIGNAL(triggered()),this,SLOT(calibrate()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Save Calibration",this);
  a->setShortcut(QKeySequence("F10"));
  connect(a,SIGNAL(triggered()),this,SLOT(saveCalibration()));
  addAction(a);
  toolbar->addAction(a);

  a = new QAction("Load calibration",this);
  a->setShortcut(QKeySequence("F11"));
  connect(a,SIGNAL(triggered()),this,SLOT(loadCalibration()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Start Reconstruction",this);
  a->setShortcut(QKeySequence("F5"));
  connect(a,SIGNAL(triggered()),this,SLOT(startReconstruction()));
  addAction(a);
  toolbar->addAction(a);
  a = new QAction("Stop Reconstruction", this);
  a->setShortcut(QKeySequence("F6"));
  connect(a, SIGNAL(triggered()), this, SLOT(stopReconstruction()));
  addAction(a);
  toolbar->addAction(a);

  m_timer = new QTimer(this);
  connect(m_timer,SIGNAL(timeout()),this,SLOT(processFrames()));
  m_timer->start(50);
}

MainWindow::~MainWindow()
{
  // Close and delete sensors
  for (unsigned i = 0; i < sensor_num_; ++i) {
    m_sensor[i]->close();
    delete m_sensor[i];
  }

    // Delete all allocated data
    delete m_colorImg[0];
    delete m_colorImg[1];
    delete m_depthImg[0];
    delete m_depthImg[1];
    delete m_sceneImg[0];
    delete m_sceneImg[1];
    delete m_calibImgColor[0];
    delete m_calibImgColor[1];
    delete m_calibImgDepth[0];
    delete m_calibImgDepth[1];

    delete m_timer;

    delete m_rec;

  delete sensor_data_;
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
    // Set sensor transformation to identity
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            m_sensorT[0](r,c) = (r == c) ? 1 : 0;
            //m_sensorT[1](r,c) = (r == c) ? 1 : 0;
        }
    }

    // Create calibration object for two sensors
    Calibration calib;
    calib.init(1);

    // Single-sided calibration
    calib.setMarker(100, 190);

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
    //calib.setMarker(400, 190, &T1);

    //// Transformation from second marker coordinate system to calibration coordinate system (rotate 180° around x-axis and move 6 mm in z-direction to the center of the board). Matrix is column-major-order.
    //double T2_[] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, -6, 1 };
    //Mat4 T2(T2_);
    //calib.setMarker(500, 190, &T2);

    bool ok = false;


    // Try to run calibration until it succeeds but at most 10 times
    for (int i = 0; i < 10; ++i)
    {
        // Reset valid flag for capturing calibration images
        m_calibImgValid[0] = m_calibImgValid[1] = false;

        // Setting m_calibrate to true, instructs the capture loop to capture calibration images
        m_calibrate = true;

        // Wait until calibration images for both sensors have been captured
        while (!m_calibImgValid[0] && !m_calibImgValid[1])
            QCoreApplication::processEvents();

        // Stop calibration frame capturing
        m_calibrate = false;

        // Pass captured images to calibration
        calib.setImage(0, *m_calibImgDepth[0], *m_calibImgColor[0], m_K[0], m_K[0]);
        calib.setImage(1, *m_calibImgDepth[1], *m_calibImgColor[1], m_K[1], m_K[1]);

        // Run calibration
        ok = calib.calibrate();

        if (ok)
            break;
    }

    if (ok)
    {
        // Retrieve sensor transformation if calibration succeeded
        calib.getTransformation(0, m_sensorT[0]);
        calib.getTransformation(1, m_sensorT[1]);
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
    if (filename.isEmpty())
        return;

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
    if (filename.isEmpty())
        return;
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
    ReconstructionParams params(sensor_num_);

    // Set per-sensor parameters
  for (unsigned i = 0; i < sensor_num_; ++i)
    {
    auto data = sensors_data_[i];
    auto depth_image = data->depth_image;
    params.setImageSize(depth_image->width(), depth_image->height(), i);
    params.setIntrinsics(data->K, i);
    }

    //params.setVolumePosition(Vec3(0, 200, 1650));
    //params.setVolumeResolution(Vec3i(204, 512, 204));
    //params.setVolumeSize(Vec3(800, 2000, 800));

    // Set volume parameters
    params.setVolumePosition(Vec3(230, 0, 1000));
    params.setVolumeResolution(Vec3i(360, 512, 360));
    params.setVolumeSize(Vec3(1000, 1000, 1000));

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
    Mesh mesh;
    bool ok = m_rec->getMesh(&mesh);

    // Delete reconstruction object
    delete m_rec;
    m_rec = 0;
    if (!ok)
    {
        std::cout << "Couldn't retrieve mesh" << std::endl;
        return;
    }

    std::cout << "Reconstructed mesh (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
    // Save mesh to file
    ok = mesh.save("mesh.ply", Mesh::PLY);
    if (ok)
        std::cout << "Saved mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;

#ifndef _DEBUG
    // Show mesh in viewer
    MeshViewer viewer;
    viewer.showMesh(&mesh);
#endif
}


void MainWindow::processFrames()
{
  for (unsigned i = 0; i < sensors_data_.size(); ++i) {
    if (!sensors_data_[i]->HasRegularImages())
      return;
  }

  // Grab images from sensor
  bool ok[3];
  for (unsigned i = 0; i < sensor_num_; ++i) {
    ok[i] = m_sensor[i]->readImage(*(sensors_data_[i]->depth_image), *(sensors_data_[i]->color_image), 40);
  }

  // Process images
  for (unsigned i = 0; i < sensor_num_; ++i)
  {
    if (!ok[i]) continue;
    sensor_data_ = sensors_data_[i];

    // Get image size
    int w = sensor_data_->color_image->width();
    int h = sensor_data_->color_image->height();

    if (m_reconstruct && m_rec)
    {
      // Add frame to reconstruction
      bool status;
      bool ret = m_rec->addFrame(
        i,
        *sensor_data_->depth_image,
        *sensor_data_->color_image,
        &status,
        sensor_data_->scene_image,
        0,
        &sensor_data_->T);

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
      memcpy(m_calibImgColor[i]->data(), m_colorImg[i]->data(), w * h * 3);
      memcpy(m_calibImgDepth[i]->data(), m_depthImg[i]->data(), w * h * 2);
      m_calibImgValid[i] = true;
    }

    // Display captured images in GUI
    QImage image(sensor_data_->color_image->data(), w, h, QImage::Format_RGB888);
    m_imgLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(w/2.0, h/2.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
  }

  // Update GUI
  update();
}
