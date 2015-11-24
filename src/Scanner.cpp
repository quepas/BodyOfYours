#include "Scanner.h"
#include <QDebug>

using RecFusion::Mesh;
using RecFusion::MeshViewer;
using RecFusion::RecFusionSDK;
using RecFusion::Reconstruction;
using RecFusion::ReconstructionParams;
using RecFusion::Sensor;
using RecFusion::Vec3;
using RecFusion::Vec3i;

Scanner::Scanner()
  : reconstruction_(nullptr),
    rec_in_progress_(false),
    timer_(nullptr)
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
  num_sensors_ = sensors_[0]->deviceCount();
  qDebug() << "[INFO] Num. of sensor devices connected: " << num_sensors_;

  for (int i = 0; i < num_sensors_; ++i) {
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

  // emit foundSensor signal
  timer_ = new QTimer();
  connect(timer_, SIGNAL(timeout()), this, SLOT(processFrames()));
  timer_->start(50);
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
  ReconstructionParams parameters(num_sensors_);
  for (int i = 0; i < num_sensors_; ++i) {
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
  for (unsigned i = 0; i < num_sensors_; ++i) {
    if (!sensors_data_[i]->HasRegularImages())
      return;
  }

  // Grab images from sensor
  bool is_data_ok[3];
  for (unsigned i = 0; i < num_sensors_; ++i) {
    is_data_ok[i] = sensors_[i]->readImage(*(sensors_data_[i]->depth_image), *(sensors_data_[i]->color_image), 40);
  }

  // Process images
  for (unsigned i = 0; i < num_sensors_; ++i)
  {
    if (!is_data_ok[i]) continue;
    auto data = sensors_data_[i];

    // Get image size
    int w = data->color_image->width();
    int h = data->color_image->height();

    if (rec_in_progress_ && reconstruction_)
    {
      // Add frame to reconstruction
      bool status = false;
      bool ret = reconstruction_->addFrame(
        i,
        *data->depth_image,
        *data->color_image,
        &status,
        data->scene_image,
        0,
        &data->T);

      if (ret && status)
      {
        // Display rendering of current reconstruction when tracking succeeded
       /* QImage image(data->scene_image->data(), w, h, QImage::Format_RGB888);
        m_recLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(w, h, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));*/
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
   /* QImage image(data->color_image->data(), w, h, QImage::Format_RGB888);
    m_imgLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(w / 2.0, h / 2.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));*/
  }

  // Update GUI
  //update();
}
