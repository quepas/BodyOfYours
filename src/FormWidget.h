#pragma once

#include <QWidget>
#include <QDataWidgetMapper>
#include <QFormLayout>
#include <QPushButton>
#include <QSqlTableModel>
#include <QHBoxLayout>
#include <QVBoxLayout>

class FormWidget : public QWidget
{
  Q_OBJECT
public:
  FormWidget(QString table, QWidget* parent = nullptr);
  //FormWidget(QString table, int rowIndex, QWidget* parent = nullptr);
  virtual ~FormWidget();

public slots:
  void setCurrentRowIndex(int rowIndex);
  void setCurrentRowWithId(int rowId);

signals:
  void locked(bool locked);

protected:
  QWidget* formWidget_;
  QFormLayout* formLayout_;
  QHBoxLayout* buttonLayout_;
  QSqlTableModel* model_;
  QDataWidgetMapper* mapper_;

private:
  int currentRowIndex_;
  QVBoxLayout* mainLayout_;

  QPushButton* lockButton_;
  QPushButton* unlockButton_;

  void initLayouts();
  void initButtons();
  void initModel(QString table);
  void initMapper();
};