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
    rec_in_progress_(false),
    calib_in_progress_(false),
    timer_(nullptr),
    parent_(parent)
{
  // Output RecFusion SDK version
  qDebug() << "[INFO] Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion();

  // Load license file
  bool ok = RecFusionSDK::setLicenseFile("License.dat");
  if (!ok)
    qDebug() << "[WARNING] Invalid RecFusion license. Export will be disabled.";

  for (int i = 0; i < MAX_NUM_SENSORS; ++i) {
    sensors_[i] = new Sensor();
    sensors_data_[i] = nullptr;
  }
  numSensor_ = sensors_[0]->deviceCount();
  qDebug() << "[INFO] Num. of sensor devices connected: " << numSensor_;

  for (int i = 0; i < numSensor_; ++i) {
    bool is_ok = sensors_[i]->open(i);
    if (!is_ok) {
      qDebug() << "[ERROR] Couldn't open sensor with num.: " << i;
    }
    else {
      sensors_data_[i] = new SensorData(*sensors_[i]);
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
  m_calibMessageBox->setWindowTitle("Calibration");
  m_calibMessageBox->setText("Press OK to capture calibration frame");
  m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
  connect(m_calibMessageBox, SIGNAL(accepted()), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(finished(int)), this, SLOT(performCalibration()));
  connect(m_calibMessageBox, SIGNAL(rejected()), this, SLOT(performCalibration()));
}

Scanner::~Scanner()
{
  for (int i = 0; i < MAX_NUM_SENSORS; ++i) {
    delete sensors_[i];
    delete sensors_data_[i];
  }
  delete reconstruction_;
  delete timer_;
}

void Scanner::startReconstruction()
{
  if (rec_in_progress_) {
    qDebug() << "[WARNING] Reconstruction already take place. Returning.";
    return;
  }
  rec_in_progress_ = false;
  delete reconstruction_;
  ReconstructionParams parameters(numSensor_);
  for (int i = 0; i < numSensor_; ++i) {
    SensorData* data = sensors_data_[i];
    auto depth_image = data->depth_image;
    auto color_image = data->color_image;
    parameters.setImageSize(color_image->width(), color_image->height(), depth_image->width(), depth_image->height(), i);
    parameters.setIntrinsics(data->K, i);
  }
  parameters.setVolumePosition(Vec3(0, 0, 1000));
  parameters.setVolumeResolution(Vec3i(256, 256, 256));
  parameters.setVolumeSize(Vec3(500, 500, 500));

  reconstruction_ = new Reconstruction(parameters);
  rec_in_progress_ = true;
}

void Scanner::stopReconstruction()
{
  rec_in_progress_ = false;
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
  MeshViewer viewer;
  viewer.showMesh(mesh);
  /*
  std::cout << "Reconstructed mesh (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
  // Save mesh to file
  ok = mesh.save("mesh.ply", Mesh::PLY);
  if (ok)
  std::cout << "Saved mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;
  */
}

void Scanner::processFrames()
{
  for (int i = 0; i < numSensor_; ++i) {
    if (!sensors_data_[i]->HasRegularImages())
      return;
  }
  // Grab images from sensor
  bool is_data_ok[3];
  for (int i = 0; i < numSensor_; ++i) {
    is_data_ok[i] = sensors_[i]->readImage(*(sensors_data_[i]->depth_image), *(sensors_data_[i]->color_image), 40);
  }
  QList<ImageData> img_camera;
  QList<ImageData> img_scene;
  // Process images
  for (int i = 0; i < numSensor_; ++i)
  {
    if (!is_data_ok[i]) continue;
    auto data = sensors_data_[i];

    // Get image size
    int w = data->color_image->width();
    int h = data->color_image->height();

    if (rec_in_progress_ && reconstruction_)
    {
      // Add frame to reconstruction
      bool is_recon_ok = false;
      bool is_frame_ok = reconstruction_->addFrame(
        i,
        *data->depth_image,
        *data->color_image,
        &is_recon_ok,
        data->scene_image,
        0,
        &data->T);

      if (is_frame_ok && is_recon_ok)
      {
        // Display rendering of current reconstruction when tracking succeeded
       QImage image(data->scene_image->data(), w, h, QImage::Format_RGB888);
       img_scene.append({image, w, h});
      }
    }
    /*else if (m_calibrate)
    {
      // Save calibration frame
      for (unsigned i = 0; i < num_sensor_; ++i) {
        auto data = sensors_data_[i];
        memcpy(data->calib_color_image->data(), data->color_image->data(), w * h * 3);
        memcpy(data->calib_depth_image->data(), data->depth_image->data(), w * h * 2);
        data->calib_image_valid = true;
      }
    }*/

    // Display captured images in GUI
    QImage image(data->color_image->data(), w, h, QImage::Format_RGB888);
    img_camera.append({image, w, h});
  }
  emit sendImages(img_camera, img_scene);
}

void Scanner::performCalibration()
{
  for (int i = 0; i < numSensor_; ++i) {
    sensors_data_[i]->ResetT();
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
      sensors_data_[j]->calib_image_valid = false;
    }
    // Setting m_calibrate to true, instructs the capture loop to capture calibration images
    calib_in_progress_ = true;
    // Wait until calibration images for both sensors have been captured
    bool waiting = true;
    while (waiting) {
      for (int i = 0; i < numSensor_; ++i) {
        waiting = waiting && !(sensors_data_[i]->IsCalibrated());
      }
      QCoreApplication::processEvents();
    }

    // Stop calibration frame capturing
    calib_in_progress_ = false;

    // Pass captured images to calibration
    for (int i = 0; i < numSensor_; ++i) {
      auto data = sensors_data_[i];
      calib.setImage(i, *(data->calib_depth_image), *(data->calib_color_image), data->K, data->K);
    }
    // Run calibration
    ok = calib.calibrate();
    if (ok) break;
  }

  if (ok)
  {
    // Retrieve sensor transformation if calibration succeeded
    for (int i = 0; i < numSensor_; ++i) {
      auto data = sensors_data_[i];
      calib.getTransformation(i, data->T);
      qDebug() << mat4ToString(data->T);
    }
    QMessageBox::information(parent_, "Calibration", "Calibration succeeded");
  }
  else
  {
    QMessageBox::information(parent_, "Calibration", "Calibration failed");
  }
}

void Scanner::calibrate()
{
  // Show message box to let user choose correct frame before starting calibration
  calib_in_progress_ = false;
  /*m_calibMessageBox->setText("Press OK to capture calibration frames.");
  m_calibMessageBox->show();*/
}

void Scanner::saveCalibration()
{
  QString filename = QFileDialog::getSaveFileName(parent_, "Save calibration");
  if (filename.isEmpty()) return;
  // Save calibrations to file as 4x4 matrices in row-major order
  std::ofstream out(filename.toStdString());
  for (int i = 0; i < numSensor_; ++i)
  {
    for (int row = 0; row < 4; ++row)
    {
      for (int col = 0; col < 4; ++col)
        out << sensors_data_[i]->T(row, col) << " ";
      out << std::endl;
    }
  }
  out.close();
}

void Scanner::loadCalibration()
{
  QString filename = QFileDialog::getOpenFileName(parent_, "Load calibration");
  if (filename.isEmpty()) return;
  std::ifstream in(filename.toStdString());
  if (!in.is_open() || !in.good())
  {
    QMessageBox::information(parent_, "Load calibration", "Couldn't open calibration file");
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
    QMessageBox::information(parent_, "Load calibration", "Error reading calibration file");
    return;
  }
  in.close();

  for (int i = 0; i < numSensor_; ++i)
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        sensors_data_[i]->T(row, col) = temp[i][col * 4 + row];
}
