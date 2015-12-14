#pragma once

#include <QWidget>
#include <QDataWidgetMapper>
#include <QFormLayout>
#include <QPushButton>
#include <QSqlTableModel>
#include <QSqlRecord>
#include <QHBoxLayout>
#include <QVBoxLayout>

class FormWidget : public QWidget
{
  Q_OBJECT
public:
  explicit FormWidget(QWidget* parent = nullptr);
  FormWidget(QSqlTableModel* model, QWidget* parent = nullptr);
  FormWidget(QString table, QWidget* parent = nullptr);
  virtual ~FormWidget();

public slots:
  void setCurrentRowIndex(int rowIndex);
  void setCurrentRowWithId(int rowId);
  void addRow();
  void resetModel();

signals:
  void locked(bool locked);
  void canceled();
  void saved();

protected:
  QWidget* formWidget_;
  QFormLayout* formLayout_;
  QHBoxLayout* buttonLayout_;
  QSqlTableModel* model_;
  QDataWidgetMapper* mapper_;

private:
  QVBoxLayout* mainLayout_;

  QPushButton* lockButton_;
  QPushButton* unlockButton_;
  QPushButton* saveButton_;
  QPushButton* cancelButton_;

  void initLayouts();
  void initButtons();
  void initModel(QString table);
  void initMapper();
  void lock(bool lock);
};