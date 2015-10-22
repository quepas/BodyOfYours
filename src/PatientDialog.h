#pragma once

#include <QDialog>
#include <QTextEdit>

class PatientDialog : public QDialog
{
  Q_OBJECT

public:
  PatientDialog(QWidget* parent = 0);
  ~PatientDialog();

public slots:
  void onClose();
  void onSave();

signals:
  void savePatient(QString name);

private:
  QTextEdit* name_text;
};
