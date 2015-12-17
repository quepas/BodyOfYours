#include "MeshDifferenceDlg.h"

#include <QDebug>
#include <QFormLayout>
#include <QVBoxLayout>

MeshDifferenceDlg::MeshDifferenceDlg(QWidget* parent, QMap<int, QString> data, int refScanID) : QDialog(parent)
{
  setWindowTitle("Wylicz roznice skanow");
  setMinimumWidth(400);
  QVBoxLayout* layout = new QVBoxLayout(this);
  refScanComboBox_ = new QComboBox(this);
  refScanComboBox_->setEnabled(false);
  compScanComboBox_ = new QComboBox(this);
  int index = 0;
  for (int key : data.keys()) {
    if (key != refScanID)
      compScanComboBox_->insertItem(index++, data[key], key);
    else
      refScanComboBox_->insertItem(index++, data[key], key);
  }
  QFormLayout* form = new QFormLayout();
  form->addRow(tr("Skan referencyjny"), refScanComboBox_);
  form->addRow(tr("Porownaj z"), compScanComboBox_);
  layout->addLayout(form);
  QHBoxLayout* buttons = new QHBoxLayout();
  calculateButton_ = new QPushButton(QIcon("icon/two25.png"), tr("Wylicz"), this);
  calculateButton_->setEnabled(data.size() > 1);
  buttons->addStretch();
  buttons->addWidget(calculateButton_);
  closeButton_ = new QPushButton(QIcon("icon/delete85.png"), tr("Anuluj"), this);
  buttons->addWidget(closeButton_);
  layout->addLayout(buttons);

  connect(closeButton_, &QPushButton::clicked, [=]{ close(); });
  connect(calculateButton_, &QPushButton::clicked, [=]{
    int refScanID = refScanComboBox_->itemData(refScanComboBox_->currentIndex()).toInt();
    int compScanID = compScanComboBox_->itemData(compScanComboBox_->currentIndex()).toInt();
    emit calculateDiff(refScanID, compScanID);
  });
}

MeshDifferenceDlg::~MeshDifferenceDlg()
{
  delete refScanComboBox_;
  delete compScanComboBox_;
  delete closeButton_;
  delete calculateButton_;
}
