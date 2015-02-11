#ifndef SCANNINGWINDOW_H
#define SCANNINGWINDOW_H

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

private:
  Ui::ScanningWindow *ui;
};

#endif // SCANNINGWINDOW_H
