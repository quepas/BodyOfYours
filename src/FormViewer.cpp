#include "FormViewer.h"
#include "PatientData.h"

FormViewer::FormViewer(QWidget* parent /*= nullptr*/)
  : QStackedWidget(parent)
{
}

FormViewer::~FormViewer()
{
  delete patient_form_;
  delete exam_form_;
}

void FormViewer::showPatient(PatientData data)
{
  setCurrentIndex(0);
}

void FormViewer::showExamination(ExaminationData data)
{
  setCurrentIndex(1);
}

void FormViewer::newPatient()
{
  setCurrentIndex(0);
}

void FormViewer::newExamination()
{
  setCurrentIndex(1);
}
