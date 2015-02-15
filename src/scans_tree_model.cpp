#include "scans_tree_model.h"

#include <QString>
#include <QVariant>

ScansTreeModel::ScansTreeModel(QObject* parent)
  : QStandardItemModel()
{
  setHorizontalHeaderItem(0, new QStandardItem(QString("Catalogue")));
  setHorizontalHeaderItem(1, new QStandardItem(QString("Patient")));
  setHorizontalHeaderItem(2, new QStandardItem(QString("Scan")));
}
