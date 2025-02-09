#include <stdio.h>

class ButtonHAL {
 public:
  virtual bool readButton() = 0;
};

class Button : public ButtonHAL {
 public:
  bool readButton() {
    return true;
  }
};

class Board {
 public:
  ButtonHAL* button() {
    return &m_button;
  }

 private:
  Button m_button;
};

//////////////////////////////////////////////////////////////////////////
int main() {
  Board board;
  ButtonHAL* button = board.button();
  printf("button: %d\n", button->readButton());
  return 0;
}
