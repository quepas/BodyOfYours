#pragma once

#include <QTreeWidget>

class PatientsWidget : public QTreeWidget
{
public:
  PatientsWidget(QWidget* parent = 0);
  ~PatientsWidget();
};
