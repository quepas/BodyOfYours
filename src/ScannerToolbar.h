#pragma once

#include <QAction>
#include <QToolBar>

class ScannerToolbar : public QToolBar
{
  Q_OBJECT
public:
  ScannerToolbar(QWidget* parent = nullptr);
  ~ScannerToolbar();

signals:
  void startReconstruction();
  void stopReconstruction(QString meshFilePath);

public slots:
  void setEnabled(bool enabled);

private:
  QAction* start_recon_;
  QAction* stop_recon_;
  QAction* addExternalMesh_;
};
