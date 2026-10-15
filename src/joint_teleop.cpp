// joint_teleop
// ============
// Interaktywny "pilot" w terminalu: wpisujesz docelowe pozycje 7 jointów
// Kinova Gen3 (radiany) albo preset, węzeł wysyła je jako trajektorię do
// arm_controller (akcja FollowJointTrajectory).
//
// UWAGA: to sterowanie omija planowanie i sprawdzanie kolizji w MoveIt
// (idzie prosto do kontrolera) - w tej symulacji (bez przeszkód) to nie
// problem.
//
// Użycie:
//   ros2 run kinova_moveit joint_teleop
//
// Komendy w konsoli (po uruchomieniu wpisz "help"):
//   j p1 p2 p3 p4 p5 p6 p7  - wszystkie 7 jointów naraz (radiany)
//   home                     - preset "home" (bezpieczna, wyśrodkowana poza)
//   vertical                 - preset "vertical" (wszystkie jointy = 0)
//   t sekundy                - czas trwania kolejnych ruchów (domyślnie 2s)
//   help                     - pokaż tę pomoc
//   q / quit                 - wyjście

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "kinova_moveit/arm_trajectory_client.hpp"
#include "kinova_moveit/robot_model.hpp"

namespace kinova_moveit
{

constexpr char kHelpText[] = R"(Komendy:
  j p1 p2 p3 p4 p5 p6 p7  - wszystkie 7 jointów naraz (radiany)
  home                     - preset "home" (bezpieczna, wyśrodkowana poza)
  vertical                 - preset "vertical" (wszystkie jointy = 0)
  t sekundy                - czas trwania kolejnych ruchów (domyślnie 2s)
  help                     - pokaż tę pomoc
  q / quit                 - wyjście
)";

void run_repl(const std::shared_ptr<ArmTrajectoryClient> & node)
{
  double default_duration = 2.0;
  std::cout << kHelpText;
  std::string line;
  while (std::cout << "teleop> " && std::getline(std::cin, line)) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) {
      continue;
    }

    if (cmd == "q" || cmd == "quit" || cmd == "exit") {
      break;
    } else if (cmd == "h" || cmd == "help") {
      std::cout << kHelpText;
    } else if (cmd == "t") {
      double seconds;
      if (iss >> seconds) {
        default_duration = seconds;
        std::cout << "  domyślny czas ruchu = " << seconds << "s\n";
      } else {
        std::cout << "  użycie: t <sekundy>\n";
      }
    } else if (cmd == "j") {
      std::vector<double> values;
      double v;
      while (iss >> v) {
        values.push_back(v);
      }
      if (values.size() != kArmJoints.size()) {
        std::cout << "  potrzeba dokładnie 7 wartości: j p1 p2 p3 p4 p5 p6 p7\n";
        continue;
      }
      std::cout << (node->move_arm(values, default_duration) ? "  OK\n" : "  ruch zakończony z błędem\n");
    } else if (cmd == "home") {
      std::cout << (node->move_arm(kPresetHome, default_duration) ? "  OK\n" : "  ruch zakończony z błędem\n");
    } else if (cmd == "vertical") {
      std::cout << (node->move_arm(kPresetVertical, default_duration) ? "  OK\n" : "  ruch zakończony z błędem\n");
    } else {
      std::cout << "  nieznana komenda: '" << cmd << "' (wpisz 'help')\n";
    }
  }
}

}  // namespace kinova_moveit

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kinova_moveit::ArmTrajectoryClient>("joint_teleop");

  std::cout << "Łączę się z arm_controller...\n";
  if (!node->wait_for_controller(10.0)) {
    std::cout << "  [błąd] Nie widzę akcji .../follow_joint_trajectory. "
                 "Czy demo.launch.py na pewno działa?\n";
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "Połączono.\n\n";

  kinova_moveit::run_repl(node);

  rclcpp::shutdown();
  return 0;
}
