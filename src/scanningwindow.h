#ifndef SCANNINGWINDOW_H
#define SCANNINGWINDOW_H

#include "sensor_viewer.h"

#include <QDialog>

namespace Ui {
  class ScanningWindow;
}

class ScanningWindow : public QDialog
{
  Q_OBJECT

public:
  explicit ScanningWindow(QWidget *parent = 0);
  ~ScanningWindow();

private slots:
  void on_startScanButton_clicked();
  void on_stopScanButton_clicked();

private:
  Ui::ScanningWindow *ui;
  SensorViewer *camera_viewer_, *depth_viewer_, *volume_viewer_;
};

#endif // SCANNINGWINDOW_H
