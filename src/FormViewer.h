#pragma once

#include "patientform.h"
#include "examinationform.h"
#include "PatientData.h"
#include "ExaminationData.h"
#include <QStackedWidget>

class FormViewer : public QStackedWidget
{
public:
  FormViewer(QWidget* parent = nullptr);
  ~FormViewer();

public slots:
  void showPatient(PatientData data);
  void ShowExamination(ExaminationData data);
  void newPatient();
  void newExamination();

private:
  PatientForm* patient_form_;
  ExaminationForm* exam_form_;

};