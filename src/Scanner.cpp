#include "Scanner.h"
#include "RecFusionUtils.h"
#include <QCoreApplication>
#include <QDebug>
#include <QFileDialog>
#include <QImage>
#include <fstream>

using RecFusion::Calibration;
using RecFusion::Mesh;
using RecFusion::MeshViewer;
using RecFusion::RecFusionSDK;
using RecFusion::Reconstruction;
using RecFusion::ReconstructionParams;
using RecFusion::Sensor;
using RecFusion::Vec3;
using RecFusion::Vec3i;

Scanner::Scanner(QWidget* parent) 
  : reconstruction_(nullptr),
    recInProgress_(false),
    calibInProgress_(false),
    timer_(nullptr),
    parent_(parent)
{
  // Output RecFusion SDK version
  qDebug() << "[INFO] Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion();

  setLicense("License.dat"); // TODO: parametrize this
  if (!hasLicense())
    qDebug() << "[WARNING] Invalid RecFusion license. Export will be disabled.";

  for (int i = 0; i < MAX_NUM_SENSORS; ++i) {
    sensors_[i] = new Sensor();
    sensorsData_[i] = nullptr;
  }
  numSensor_ = sensors_[0]->deviceCount();
  qDebug() << "[INFO] Num. of sensor devices connected: " << numSensor_;

  for (int i = 0; i < numSensor_; ++i) {
    bool is_ok = sensors_[i]->open(i);
    if (!is_ok) {
      qDebug() << "[ERROR] Couldn't open sensor with num.: " << i;
    }
    else {
      sensorsData_[i] = new SensorData(*sensors_[i]);
      /*int w = m_sensor[i]->depthWidth();
      int h = m_sensor[i]->depthHeight();
      m_imgLabel[i]->resize(w, h);*/
    }
  }
  timer_ = new QTimer();
  connect(timer_, SIGNAL(timeout()), this, SLOT(processFrames()));
  timer_->start(50);

  m_calibMessageBox = new QMessageBox(parent_);
  m_calibMessageBox->setIcon(QMessageBox::Information);
  m_calibMessageBox->setWindowTitle(tr(":calibration"));
  m_calibMessageBox->setText(tr(":press_ok_to_capture_frame"));
  m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
  connect(m_calibMessageBox, SIGNAL(accepted()), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(finished(int)), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(rejected()), this, SLOT(performCalibration()));
}

Scanner::~Scanner()
{
  for (int i = 0; i < MAX_NUM_SENSORS; ++i) {
    delete sensors_[i];
    delete sensorsData_[i];
  }
  delete reconstruction_;
  delete timer_;
}

void Scanner::startReconstruction()
{
  if (recInProgress_) {
    qDebug() << "[WARNING] Reconstruction already take place. Returning.";
    return;
  }
  recInProgress_ = false;
  delete reconstruction_;
  ReconstructionParams parameters(numSensor_);
  for (int i = 0; i < numSensor_; ++i) {
    SensorData* data = sensorsData_[i];
    auto depth_image = data->depth_image;
    auto color_image = data->color_image;
    parameters.setImageSize(color_image->width(), color_image->height(), depth_image->width(), depth_image->height(), i);
    parameters.setIntrinsics(data->K, i);
  }
  parameters.setVolumePosition(Vec3(0, 0, 1000));
  parameters.setVolumeResolution(Vec3i(256, 256, 256));
  parameters.setVolumeSize(Vec3(500, 500, 500));

  reconstruction_ = new Reconstruction(parameters);
  recInProgress_ = true;
}

void Scanner::stopReconstruction()
{
  recInProgress_ = false;
  if (!reconstruction_) {
    qDebug() << "[ERROR] Reconstruction class failed.";
    return;
  }
  Mesh* mesh = new Mesh;
  bool is_mesh_ok = reconstruction_->getMesh(mesh);
  delete reconstruction_;
  reconstruction_ = nullptr;
  if (!is_mesh_ok) {
    qDebug() << "[ERROR] Retrieving mesh failed.";
    return;
  }

  // If no valid license - display reconstructed mesh
  if (!hasLicense()) {
    MeshViewer viewer;
    viewer.showMesh(mesh);
  }
  // If valid license - save mesh
  else {
    std::string meshFile = "mesh.ply"; // TODO: parametrize this!
    if (mesh->save(meshFile.c_str(), Mesh::PLY)) {
      qDebug() << "[INFO@Scanner]: Saved mesh as PLY (" << meshFile.c_str() << ")";
    }
  }
}

void Scanner::processFrames()
{
  for (int i = 0; i < numSensor_; ++i) {
    if (!sensorsData_[i]->HasImages())
      return;
  }
  // Grab images from sensor
  bool isDataOk[3];
  for (int i = 0; i < numSensor_; ++i) {
    isDataOk[i] = sensors_[i]->readImage(*(sensorsData_[i]->depth_image), *(sensorsData_[i]->color_image), 40);
  }
  QList<ImageData> imgCamera;
  QList<ImageData> imgScene;
  // Process images
  for (int i = 0; i < numSensor_; ++i)
  {
    if (!isDataOk[i]) continue;
    auto data = sensorsData_[i];

    // Get image size
    int width = data->color_image->width();
    int height = data->color_image->height();

    if (recInProgress_ && reconstruction_)
    {
      // Add frame to reconstruction
      bool isRecOk = false;
      bool isFrameOk = reconstruction_->addFrame(
        i,
        *data->depth_image, *data->color_image,
        &isRecOk,
        data->scene_image,
        0,
        &data->T);

      if (isFrameOk && isRecOk)
      {
        // Prepare rendered image of reconstructed scene
       QImage image(data->scene_image->data(), width, height, QImage::Format_RGB888);
       imgScene.append({image, width, height});
      }
    }
    else if (calibInProgress_)
    {
      // Save calibration frame
      for (int i = 0; i < numSensor_; ++i) {
        data->calib_image_valid = true;
      }
    }

    // Display captured images in GUI
    QImage image(data->color_image->data(), width, height, QImage::Format_RGB888);
    imgCamera.append({image, width, height});
  }
  emit sendImages(imgCamera, imgScene);
}

void Scanner::performCalibration()
{
  for (int i = 0; i < numSensor_; ++i) {
    sensorsData_[i]->ResetT();
  }
  // Create calibration object for two sensors
  Calibration calib;
  calib.init(numSensor_);
  // Single-sided calibration
  calib.setMarker(100, 190);
  bool ok = false;
  // Try to run calibration until it succeeds but at most 10 times
  for (int i = 0; i < 10; ++i)
  {
    // Reset valid flag for capturing calibration images
    for (int j = 0; j < numSensor_; ++j) {
      sensorsData_[j]->calib_image_valid = false;
    }
    // Setting m_calibrate to true, instructs the capture loop to capture calibration images
    calibInProgress_ = true;
    // Wait until calibration images for both sensors have been captured
    bool waiting = true;
    while (waiting) {
      for (int i = 0; i < numSensor_; ++i) {
        waiting = waiting && !(sensorsData_[i]->IsCalibrated());
      }
      QCoreApplication::processEvents();
    }

    // Stop calibration frame capturing
    calibInProgress_ = false;

    // Pass captured images to calibration
    for (int i = 0; i < numSensor_; ++i) {
      auto data = sensorsData_[i];
      calib.setImage(i, *(data->depth_image), *(data->color_image), data->K, data->K);
    }
    // Run calibration
    ok = calib.calibrate();
    if (ok) break;
  }

  if (ok)
  {
    // Retrieve sensor transformation if calibration succeeded
    for (int i = 0; i < numSensor_; ++i) {
      auto data = sensorsData_[i];
      calib.getTransformation(i, data->T);
      qDebug() << mat4ToString(data->T);
    }
    QMessageBox::information(parent_, tr(":calibration"), tr(":calibration_succeeded"));
  }
  else
  {
    QMessageBox::information(parent_, tr(":calibration"), tr(":calibration_failed"));
  }
}

void Scanner::calibrate()
{
  // Show message box to let user choose correct frame before starting calibration
  calibInProgress_ = false;
  m_calibMessageBox->setText("Press OK to capture calibration frames.");
  m_calibMessageBox->show();
}

void Scanner::saveCalibration()
{
  QString filename = QFileDialog::getSaveFileName(parent_, tr(":save_calibration"));
  if (filename.isEmpty()) return;
  // Save calibrations to file as 4x4 matrices in row-major order
  std::ofstream out(filename.toStdString());
  for (int i = 0; i < numSensor_; ++i)
  {
    for (int row = 0; row < 4; ++row)
    {
      for (int col = 0; col < 4; ++col)
        out << sensorsData_[i]->T(row, col) << " ";
      out << std::endl;
    }
  }
  out.close();
}

void Scanner::loadCalibration()
{
  QString filename = QFileDialog::getOpenFileName(parent_, tr(":load_calibration"));
  if (filename.isEmpty()) return;
  std::ifstream in(filename.toStdString());
  if (!in.is_open() || !in.good())
  {
    QMessageBox::information(parent_, tr(":load_calibration"), tr(":couldnt_open_calibration_file"));
    return;
  }
  // Load calibration from file
  double temp[2][16];
  for (int i = 0; i < numSensor_; ++i)
  {
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        in >> temp[i][col * 4 + row];
  }
  if (in.fail())
  {
    QMessageBox::information(parent_, tr("load_calibration"), tr(":error_reading_calibration_file"));
    return;
  }
  in.close();

  for (int i = 0; i < numSensor_; ++i)
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        sensorsData_[i]->T(row, col) = temp[i][col * 4 + row];
}

int Scanner::numSensor() {
  return numSensor_;
}

bool Scanner::hasLicense() {
  return hasLicense_;
}

bool Scanner::setLicense(QString file)
{
  const char* name = file.toStdString().c_str();
  hasLicense_ = RecFusionSDK::setLicenseFile(name);
  return hasLicense_;
}
