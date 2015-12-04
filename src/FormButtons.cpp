#include "FormButtons.h"

FormButtons::FormButtons(QWidget* parent /*= nullptr*/)
{
  buttons_ = new QWidget(parent);
  buttons_layout_ = new QHBoxLayout(buttons_);
  // SAVE
  save_button_ = new QPushButton(QIcon(tr("icon/save29.png")), tr("Zapisz"), buttons_);
  addButton(save_button_, SAVE);
  buttons_layout_->addWidget(save_button_);
  // MODIFY
  modify_button_ = new QPushButton(QIcon(tr("icon/checked21.png")), tr("Modyfikuj"), buttons_);
  addButton(modify_button_, MODIFY);
  buttons_layout_->addWidget(modify_button_);
  // CLEAR
  clear_button_ = new QPushButton(QIcon(tr("icon/wiping16.png")), tr("Wyczysc"), buttons_);
  addButton(clear_button_, CLEAR);
  buttons_layout_->addWidget(clear_button_);
  // CANCEL
  cancel_button_ = new QPushButton(QIcon(tr("icon/delete85.png")), tr("Anuluj"), buttons_);
  addButton(cancel_button_, CANCEL);
  buttons_layout_->addWidget(cancel_button_);
  cancel_button_->setVisible(false);
  // DELETE
  remove_button_ = new QPushButton(QIcon(tr("icon/delete81.png")), tr("Usun"), buttons_);
  addButton(remove_button_, REMOVE);
  buttons_layout_->addWidget(remove_button_);
  // LOCK
  lock_button_ = new QPushButton(QIcon(tr("icon/tool683.png")), tr("Zablokuj"), buttons_);
  lock_button_->setVisible(false);
  addButton(lock_button_, LOCK);
  buttons_layout_->addWidget(lock_button_);
  // UNLOCK
  unlock_button_ = new QPushButton(QIcon(tr("icon/locked59.png")), tr("Odblokuj"), buttons_);
  addButton(unlock_button_, UNLOCK);
  buttons_layout_->addWidget(unlock_button_);

  // Current buttons state
  setNewFormState();
  setLocked(false);
  lock_button_->setVisible(false);
  // MODIFY
  connect(modify_button_, &QPushButton::clicked, [=]{
    setLocked(true);
  });
  // CANCEL
  connect(cancel_button_, &QPushButton::clicked, [=]{
    setLocked(true);
  });
  // CLEAR
  connect(clear_button_, &QPushButton::clicked, [=]{
    setLocked(true);
  });
  // REMOVE
  connect(remove_button_, &QPushButton::clicked, [=]{
    setLocked(true);
  });

  // SAVE -> MODIFY
  connect(save_button_, &QPushButton::clicked, [=]{
    setShowFormState();
  });
  // LOCK <-> UNLOCK
  connect(lock_button_, &QPushButton::clicked, [=]{
    setLocked(true);
  });
  connect(unlock_button_, &QPushButton::clicked, [=]{
    setLocked(false);
  });
}

FormButtons::~FormButtons()
{
  delete save_button_;
  delete clear_button_;
  delete cancel_button_;
  delete lock_button_;
  delete unlock_button_;
  delete remove_button_;
  delete modify_button_;
  delete buttons_;
}

void FormButtons::setNewFormState()
{
  save_button_->setVisible(true);
  modify_button_->setVisible(false);
  remove_button_->setVisible(false);
  clear_button_->setVisible(true);
  setLocked(false);
  unlock_button_->setVisible(false);
  lock_button_->setVisible(false);
}

void FormButtons::setShowFormState()
{
  save_button_->setVisible(false);
  modify_button_->setVisible(true);
  clear_button_->setVisible(false);
  remove_button_->setVisible(true);
  setLocked(true);
}

void FormButtons::setLocked(bool locked)
{
  save_button_->setDisabled(locked);
  modify_button_->setDisabled(locked);
  clear_button_->setDisabled(locked);
  cancel_button_->setDisabled(locked);
  remove_button_->setDisabled(locked);
  unlock_button_->setVisible(locked);
  lock_button_->setVisible(!locked);
}
