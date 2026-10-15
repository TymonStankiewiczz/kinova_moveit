#pragma once
// Odczyt pojedynczych znaków z terminala bez czekania na Enter i bez echo
// ("tryb cbreak") - odpowiednik Pythonowego tty.setcbreak() + select().
// RAII: ustawienia terminala są przywracane w destruktorze, nawet jeśli
// program zakończy się przez wyjątek.

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <stdexcept>

namespace kinova_moveit
{

class TerminalInput
{
public:
  TerminalInput()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("ten węzeł wymaga prawdziwego terminala (stdin nie jest TTY)");
    }
    tcgetattr(STDIN_FILENO, &original_settings_);
    termios raw = original_settings_;
    raw.c_lflag &= ~(static_cast<tcflag_t>(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~TerminalInput() { tcsetattr(STDIN_FILENO, TCSADRAIN, &original_settings_); }

  TerminalInput(const TerminalInput &) = delete;
  TerminalInput & operator=(const TerminalInput &) = delete;

  // Zwraca jeden znak jeśli pojawił się w ciągu timeout_sec, inaczej '\0'.
  char get_key(double timeout_sec) const
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_sec);
    tv.tv_usec = static_cast<suseconds_t>((timeout_sec - static_cast<double>(tv.tv_sec)) * 1e6);

    if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) <= 0) {
      return '\0';
    }
    char c = '\0';
    if (read(STDIN_FILENO, &c, 1) != 1) {
      return '\0';
    }
    return c;
  }

private:
  termios original_settings_{};
};

}  // namespace kinova_moveit
