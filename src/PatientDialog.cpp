#include "PatientDialog.h"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

PatientDialog::PatientDialog(QWidget* parent /*= 0*/)
  : QDialog(parent)
{
  QPushButton *close_button = new QPushButton(tr("Zamknij"));
  connect(close_button, SIGNAL(clicked()), this, SLOT(onClose()));

  QPushButton *save_button = new QPushButton(tr("Zapisz"));
  connect(save_button, SIGNAL(clicked()), this, SLOT(onSave()));

  QLabel *name_label = new QLabel("Imie i nazwisko");
  name_text = new QTextEdit;

  QGridLayout *content_layout = new QGridLayout;
  content_layout->addWidget(name_label, 0, 0);
  content_layout->addWidget(name_text, 0, 1);

  QHBoxLayout *buttons_layout = new QHBoxLayout;
  buttons_layout->addStretch(1);
  buttons_layout->addWidget(close_button);
  buttons_layout->addWidget(save_button);

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addLayout(content_layout);
  main_layout->addLayout(buttons_layout);
  setLayout(main_layout);

  setWindowTitle("Dodaj pacjenta");
}

PatientDialog::~PatientDialog()
{

}

void PatientDialog::onClose()
{
  close();
}

void PatientDialog::onSave()
{
  emit savePatient(name_text->toPlainText());
}
