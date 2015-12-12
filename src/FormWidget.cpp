#include "FormWidget.h"

#include <QDebug>
#include <QSqlError>
#include <QSqlRecord>

FormWidget::FormWidget(QString table, QWidget* parent /*= nullptr*/)
  : QWidget(parent)
{
  initLayouts();
  initButtons();
  initModel(table);
  initMapper();
  lock(true);
  saveButton_->setVisible(false);
}

FormWidget::FormWidget(QSqlTableModel* model, QWidget* parent /*= nullptr*/)
  : QWidget(parent), model_(model)
{
  initLayouts();
  initButtons();
  model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  initMapper();
  lock(true);
  saveButton_->setVisible(false);
}

FormWidget::~FormWidget()
{
  delete mapper_;
  delete model_;
  delete unlockButton_;
  delete lockButton_;
  delete saveButton_;
  delete formLayout_;
  delete formWidget_;
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
  saveButton_ = new QPushButton(QIcon(tr("icon/save29.png")), tr("Zapisz"), this);

  buttonLayout_->addWidget(saveButton_);
  buttonLayout_->addWidget(lockButton_);
  buttonLayout_->addWidget(unlockButton_);

  connect(lockButton_, &QPushButton::clicked, [=]{
    lock(true);
  });
  connect(unlockButton_, &QPushButton::clicked, [=]{
    lock(false);
  });
  connect(saveButton_, &QPushButton::clicked, [=]{
    mapper_->submit();
  });
}

void FormWidget::initModel(QString table)
{
  model_ = new QSqlTableModel(this);
  model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  model_->setTable(table);
  model_->select();
}

void FormWidget::initMapper()
{
  mapper_ = new QDataWidgetMapper(this);
  mapper_->setModel(model_);
}

void FormWidget::setCurrentRowIndex(int rowIndex)
{
  mapper_->setCurrentIndex(rowIndex);
  lock(true);
  saveButton_->setVisible(false);
}

void FormWidget::setCurrentRowWithId(int rowId)
{
  for (int i = 0; i < model_->rowCount(); ++i) {
    QSqlRecord record = model_->record(i);
    if (record.value("id") == rowId) {
      mapper_->setCurrentIndex(i);
      lock(true);
      saveButton_->setVisible(false);
      return;
    }
  }
}

void FormWidget::addRow()
{
  int rowCount = model_->rowCount();
  qDebug() << "RowCOunt: " << rowCount;
  if (!model_->insertRow(rowCount)) {
    qDebug() << "[ERROR] Couldn't insert new row: " << model_->lastError().text();
    --rowCount;
  }
  mapper_->setCurrentIndex(rowCount);
  lock(false);
  lockButton_->setVisible(false);
  saveButton_->setVisible(true);
}

void FormWidget::lock(bool lock)
{
  formWidget_->setEnabled(!lock);
  lockButton_->setVisible(!lock);
  unlockButton_->setVisible(lock);
  emit locked(lock);
}
