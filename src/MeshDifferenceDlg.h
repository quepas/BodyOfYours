#pragma once

#include <QDialog>
#include <QComboBox>
#include <QPushButton>
#include <QStringList>

class MeshDifferenceDlg : public QDialog
{
  Q_OBJECT
public:
  MeshDifferenceDlg(QWidget* parent, QStringList data);
  ~MeshDifferenceDlg();

private:
  QComboBox* additional_mesh_;
  QPushButton* close_button_;
  QPushButton* calculate_button_;
};