#pragma once

#include <QString>
#include <QDir>

namespace Resources {
  static const QString ICON_OK = "gui/icons/OK.ico";
  static const QString ICON_ERROR = "gui/icons/error.ico";
  static const QString ICON_SYNC = "gui/icons/sync.ico";

  static const QString SCANS_DATA_PATH = QDir::currentPath() + "/data";
}