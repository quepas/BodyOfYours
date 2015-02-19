#pragma once

#include <QString>
#include <QDir>

namespace Resources {
  static const QString ICON_OK = "gui/icons/OK.ico";
  static const QString ICON_ERROR = "gui/icons/error.ico";
  static const QString ICON_SYNC = "gui/icons/sync.ico";
  static const QString ICON_WARNING = "gui/icons/warning.ico";
  static const QString ICON_FEMALE = "gui/icons/female.ico";
  static const QString ICON_MALE = "gui/icons/male.ico";
  static const QString ICON_ADD = "gui/icons/add.ico";
  static const QString ICON_REMOVE = "gui/icons/remove.ico";
  static const QString ICON_SCAN = "gui/icons/scan.ico";
  static const QString ICON_MODIFY = "gui/icons/modify.ico";
  static const QString ICON_CREATE = "gui/icons/create.ico";

  static const QString SCANS_DATA_PATH = QDir::currentPath() + "/data";
}