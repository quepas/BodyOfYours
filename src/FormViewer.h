#pragma once

#include "PatientData.h"
#include "ExaminationData.h"
#include <QStackedWidget>

#include "PatientForm.h"
#include "ExaminationForm.h"

class FormViewer : public QStackedWidget
{
public:
  FormViewer(QWidget* parent = nullptr);
  ~FormViewer();

public slots:
  void showPatient(PatientData data);
  void showExamination(ExaminationData data);
  void newPatient();
  void newExamination();

private:
  PatientForm* patient_form_;
  ExaminationForm* exam_form_;

};