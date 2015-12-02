#pragma once

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>

class FormButtons : public QButtonGroup
{
  Q_OBJECT
public:
  FormButtons(QWidget* parent = nullptr);
  ~FormButtons();

  void setNewFormState();
  void setShowFormState();

  enum BUTTON
  {
    SAVE = 0,
    MODIFY,
    CLEAR,
    CANCEL,
    REMOVE,
    LOCK,
    UNLOCK
  };

  QWidget* toQWidget() { return buttons_; }

private:
  QWidget* buttons_;
  QHBoxLayout* buttons_layout_;
  QPushButton* save_button_;
  QPushButton* modify_button_;
  QPushButton* clear_button_;
  QPushButton* cancel_button_;
  QPushButton* remove_button_;
  QPushButton* lock_button_;
  QPushButton* unlock_button_;
};