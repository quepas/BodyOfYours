#pragma once

#include <QStandardItemModel>
#include <QFileSystemModel>

class ScansTreeModel : public QStandardItemModel
{
public:
  ScansTreeModel(QObject* parent);

private:
  QFileSystemModel* help_model_;

  void PrepareTree();
};
