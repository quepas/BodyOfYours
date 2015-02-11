#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "scans_viewer.h"
#include "sensor_viewer.h"
#include "scans_tree.h"
#include "scanner_3d.h"
#include "reme_scanner_3d.h"

#include <QMainWindow>
#include <QModelIndex>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_scansTree_doubleClicked(const QModelIndex& index);
  void on_computingDevicesComboBox_currentIndexChanged(int index);
  void on_scanButton_clicked();

  void captureCameraFrame(FrameData* frame);
  void captureDepthFrame(FrameData* frame);
  void captureVolumeFrame(FrameData* frame);

private:
  Ui::MainWindow *ui;
  ScansViewer* scans_viewer_;
  SensorViewer* sensor_viewer_, *sensor_depth_viewer_, *sensor_volume_viewer_;
  ScansTree* scans_tree_;
  Scanner3D* scanner3d_;

  bool is_scanning_;
};

#endif // MAINWINDOW_H
