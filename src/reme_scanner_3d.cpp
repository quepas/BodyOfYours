#include "reme_scanner_3d.h"

#include <QDebug>

RemeScanner3D::RemeScanner3D()
{
  reme_context_create(&context_);
}

RemeScanner3D::~RemeScanner3D()
{

}

QStringList RemeScanner3D::GetCompatibleDevices()
{
  QStringList result;

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
    result.append(name);
    reme_options_get(context_, new_options, "vendor", vendor, 256);
    reme_options_get(context_, new_options, "type", type, 256);
  }
  return result;
}
