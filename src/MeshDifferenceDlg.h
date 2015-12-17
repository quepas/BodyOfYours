#pragma once

#include <QDialog>
#include <QComboBox>
#include <QPushButton>
#include <QStringList>

class MeshDifferenceDlg : public QDialog
{
  Q_OBJECT
public:
  MeshDifferenceDlg(QWidget* parent, QMap<int, QString> data, int refScanID);
  ~MeshDifferenceDlg();

signals:
  void calculateDiff(int refScanID, int compScanID);

private:
  QComboBox* refScanComboBox_;
  QComboBox* compScanComboBox_;
  QPushButton* closeButton_;
  QPushButton* calculateButton_;
};