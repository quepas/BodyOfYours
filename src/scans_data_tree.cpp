#include "scans_data_tree.h"


ScansDataTree::ScansDataTree(QTreeView* tree_view)
  : view_(tree_view),
    model_(new ScansTreeModel(nullptr))
{

}

ScansDataTree::~ScansDataTree()
{
  delete model_;
}
