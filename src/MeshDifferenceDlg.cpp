#include "MeshDifferenceDlg.h"

#include <QDebug>
#include <QVBoxLayout>

MeshDifferenceDlg::MeshDifferenceDlg(QWidget* parent, QStringList data) : QDialog(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  additional_mesh_ = new QComboBox(this);
  for (int i = 0; i < data.size(); ++i) {
    additional_mesh_->insertItem(i, data[i]);
  }
  layout->addWidget(additional_mesh_);
  calculate_button_ = new QPushButton(tr("Wylicz"), this);
  layout->addWidget(calculate_button_);
  close_button_ = new QPushButton(tr("Anuluj"), this);
  layout->addWidget(close_button_);

  connect(close_button_, &QPushButton::clicked, [=]{ close(); });
}

MeshDifferenceDlg::~MeshDifferenceDlg()
{
  delete additional_mesh_;
  delete close_button_;
  delete calculate_button_;
}
