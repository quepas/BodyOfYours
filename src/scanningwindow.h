#ifndef SCANNINGWINDOW_H
#define SCANNINGWINDOW_H

#include "sensor_viewer.h"
#include "scanning_3d.h"

#include <QDialog>

namespace Ui {
  class ScanningWindow;
}

class ScanningWindow : public QDialog
{
  Q_OBJECT

public:
  ScanningWindow(Scanner3D* scanner, QWidget *parent = 0);
  ~ScanningWindow();

  void StartGrabbingData();

private slots:
  void on_startScanButton_clicked();
  void on_stopScanButton_clicked();
  void on_restartScanButton_clicked();
  void on_saveScanButton_clicked();
  void on_cancelScanButton_clicked();

  void captureCameraFrame(FrameData* frame);
  void captureDepthFrame(FrameData* frame);
  void captureVolumeFrame(FrameData* frame);

private:
  Ui::ScanningWindow *ui;
  SensorViewer *camera_viewer_, *depth_viewer_, *volume_viewer_;
  Scanning3D *scanning_;
};

#endif // SCANNINGWINDOW_H
