#include "FormButtons.h"

FormButtons::FormButtons(QWidget* parent /*= nullptr*/)
{
  buttons_ = new QWidget(parent);
  buttons_layout_ = new QHBoxLayout(buttons_);
  save_button_ = new QPushButton(QIcon(tr("icon/save29.png")), tr("Zapisz"), buttons_);
  addButton(save_button_, SAVE);
  buttons_layout_->addWidget(save_button_);
  clear_button_ = new QPushButton(QIcon(tr("icon/wiping16.png")), tr("Wyczysc"), buttons_);
  addButton(clear_button_, CLEAR);
  buttons_layout_->addWidget(clear_button_);
  cancel_button_ = new QPushButton(QIcon(tr("icon/delete85.png")), tr("Anuluj"), buttons_);
  addButton(cancel_button_, CANCEL);
  buttons_layout_->addWidget(cancel_button_);
  lock_button_ = new QPushButton(QIcon(tr("icon/unlocked46.png")), tr("Zablokuj"), buttons_);
  addButton(lock_button_, LOCK);
  buttons_layout_->addWidget(lock_button_);
  unlock_button_ = new QPushButton(QIcon(tr("icon/locked61.png")), tr("Odblokuj"), buttons_);
  addButton(unlock_button_, UNLOCK);
  buttons_layout_->addWidget(unlock_button_);
  //buttons_layout_->addStretch(1);
  //setLayout(buttons_layout_);
}

FormButtons::~FormButtons()
{
  delete save_button_;
  delete clear_button_;
  delete cancel_button_;
  delete lock_button_;
  delete unlock_button_;
  // 
  delete buttons_;

}
