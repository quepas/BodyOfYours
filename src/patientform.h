#ifndef PATIENTFORM_H
#define PATIENTFORM_H

#include "PatientData.h"

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>

namespace Ui {
  class PatientForm;
}

class PatientForm : public QWidget
{
  Q_OBJECT

public:
  explicit PatientForm(QWidget *parent = 0);
  ~PatientForm();

  void setData(const PatientData& data);
  void clear();

  void setShowState(bool show_state);

public slots:
  void onSave();
  void onClear();
  void onDelete();

signals:
  void savePatient(PatientData name);
  void deletePatient();

private:
  Ui::PatientForm *ui;

  QPushButton* save_button_;
  QPushButton* clear_button_;
  QPushButton* delete_button_;
  QHBoxLayout* buttons_layout_;
};

#endif // PATIENTFORM_H
