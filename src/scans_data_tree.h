#pragma once

#include "scans_tree_model.h"

#include <QTreeView>

class ScansDataTree
{
public:
  ScansDataTree(QTreeView* tree_view);
  ~ScansDataTree();

  ScansTreeModel* model() { return model_; }

private:
  QTreeView* view_;
  ScansTreeModel* model_;
};
