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
    rec_in_progress_(false)
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

}

Scanner::~Scanner()
{
  for (int i = 0; i < MAX_NUM_SENSORS; ++i) {
    delete sensors_[i];
    delete sensors_data_[i];
  }
  delete reconstruction_;
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
  parameters.setVolumeSize(Vec3(1000, 1000, 1000));

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
