#include "FormWidget.h"

//FormWidget::FormWidget(QString table, QWidget* parent /*= nullptr*/)
//  : QWidget(parent)
//{
//  initLayouts();
//  initButtons();
//  initModel(table);
//  initMapper();
//  lockButton_->setVisible(false);
//  unlockButton_->setVisible(false);
//}

FormWidget::FormWidget(QString table, QWidget* parent /*= nullptr*/)
  : QWidget(parent), currentRowIndex_(-1)
{
  initLayouts();
  initButtons();
  initModel(table);
  initMapper();
  lockButton_->setVisible(false);
  formWidget_->setEnabled(false);
}

FormWidget::~FormWidget()
{
  delete mapper_;
  delete model_;
  delete unlockButton_;
  delete lockButton_;
  delete formWidget_;
  delete formLayout_;
  delete buttonLayout_;
  delete mainLayout_;
}

void FormWidget::initLayouts()
{
  mainLayout_ = new QVBoxLayout(this);

  formWidget_ = new QWidget;
  formLayout_ = new QFormLayout(formWidget_);
  buttonLayout_ = new QHBoxLayout();
  buttonLayout_->addStretch();

  mainLayout_->addWidget(formWidget_);
  mainLayout_->addLayout(buttonLayout_);
  mainLayout_->addStretch();
}

void FormWidget::initButtons()
{
  lockButton_ = new QPushButton(QIcon(tr("icon/locked59.png")), tr("Zablokuj"), this);
  unlockButton_ = new QPushButton(QIcon(tr("icon/tool683.png")), tr("Odblokuj"), this);

  buttonLayout_->addWidget(lockButton_);
  buttonLayout_->addWidget(unlockButton_);

  connect(lockButton_, &QPushButton::clicked, [=]{
    formWidget_->setEnabled(false);
    lockButton_->setVisible(false);
    unlockButton_->setVisible(true);
    emit locked(true);
  });
  connect(unlockButton_, &QPushButton::clicked, [=]{
    formWidget_->setEnabled(true);
    lockButton_->setVisible(true);
    unlockButton_->setVisible(false);
    emit locked(false);
  });
}

void FormWidget::initModel(QString table)
{
  model_ = new QSqlTableModel(this);
  model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  model_->setTable(table);
  //model_->setFilter("id = 3");
  model_->select();
}

void FormWidget::initMapper()
{
  mapper_ = new QDataWidgetMapper(this);
  mapper_->setModel(model_);
}

void FormWidget::setCurrentRowIndex(int rowIndex)
{
  currentRowIndex_ = rowIndex;
  mapper_->setCurrentIndex(currentRowIndex_);
}

void FormWidget::setCurrentRowWithId(int rowId)
{
  model_->setFilter("id = " + rowId);
  model_->select();
}
