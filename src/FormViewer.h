#pragma once

#include "PatientForm.h"
#include "examinationform.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include <QStackedWidget>

class FormViewer : public QStackedWidget
{
public:
  FormViewer(QWidget* parent = nullptr);
  ~FormViewer();

  PatientForm* patient_form() { return patient_form_; }
  ExaminationForm* examination_form() { return exam_form_; }

public slots:
  void showPatient(PatientData data);
  void ShowExamination(ExaminationData data);
  void newPatient();
  void newExamination();

private:
  PatientForm* patient_form_;
  ExaminationForm* exam_form_;

};