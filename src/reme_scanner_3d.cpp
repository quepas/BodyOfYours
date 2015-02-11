#include "reme_scanner_3d.h"

#include <QDebug>

RemeScanner3D::RemeScanner3D()
{
  reme_context_create(&context_);
  reme_sensor_create(context_, "openni;kinect", true, &sensor_);
  reme_sensor_open(context_, sensor_);
  reme_image_create(context_, &image_);
}

RemeScanner3D::~RemeScanner3D()
{
  reme_context_destroy(&context_);
}

QStringList RemeScanner3D::GetComputingDevices()
{
  QStringList result("choose device");

  reme_options_t options, new_options;
  reme_options_create(context_, &options);
  reme_options_create(context_, &new_options);

  if (reme_context_bind_opencl_info(context_, options) != REME_ERROR_SUCCESS) {
    qDebug() << "Error while getting OpenCL compatible devices.";
  }
  int num_devices = 0;
  reme_options_get_repeated_count(context_, options, "devices", &num_devices);
  char name[256], vendor[256], type[256];

  for (int idx = 0; idx < num_devices; ++idx) {
    reme_options_bind_repeated_message(context_, options, "devices", idx, new_options);
    reme_options_get(context_, new_options, "name", name, 256);
    reme_options_get(context_, new_options, "vendor", vendor, 256);
    reme_options_get(context_, new_options, "type", type, 256);
    result.append(QString(name).trimmed() + " [" + type + "]");
  }
  return result;
}

bool RemeScanner3D::InitComputingDevice(int device_id)
{
  reme_options_t options;
  reme_options_create(context_, &options);
  reme_context_bind_reconstruction_options(context_, options);
  reme_options_set_int(context_, options, "device_id", device_id);
  if (reme_context_compile(context_) == REME_ERROR_UNSPECIFIED) {
    reme_context_print_errors(context_);
    return false;
  }
  qDebug() << "[RemeScanner3d]: Device (id: " + QString::number(device_id) + ") initialized.";
  return true;
}

bool RemeScanner3D::GrabCameraFrame(FrameData* out_frame)
{
  return GrabFrame(out_frame, REME_IMAGE_AUX);
}

bool RemeScanner3D::GrabDepthFrame(FrameData* out_frame)
{
  return GrabFrame(out_frame, REME_IMAGE_DEPTH);
}

bool RemeScanner3D::GrabFrame(FrameData* out_frame, reme_sensor_image_t frame_type)
{
  bool is_grab_ok = REME_SUCCESS(reme_sensor_grab(context_, sensor_));
  if (is_grab_ok) {
    reme_sensor_prepare_image(context_, sensor_, frame_type);
    reme_sensor_get_image(context_, sensor_, frame_type, image_);
    reme_image_get_bytes(context_, image_, &(out_frame->data), &(out_frame->length));
    reme_image_get_info(context_, image_, &(out_frame->width), &(out_frame->height));
  }
  return is_grab_ok;
}

bool RemeScanner3D::GrabVolumeFrame(FrameData* out_frame)
{
  return GrabFrame(out_frame, REME_IMAGE_VOLUME);
}
