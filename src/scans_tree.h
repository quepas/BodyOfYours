#pragma once

#include <QFileSystemModel>
#include <QString>
#include <QTreeView>

class ScansTree
{
public:
  ScansTree(QTreeView* tree_view, QString tree_root);
  ~ScansTree();

  QFileSystemModel* model() { return model_; }

private:
  QString root_;
  QFileSystemModel* model_;
  QTreeView* tree_view_;
};
