#pragma once
// Stałe opisujące ramię Kinova Gen3 (7-DOF, bez chwytaka) - wspólne dla
// wszystkich trzech węzłów teleop w tym pakiecie.

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace kinova_moveit
{

// 7 jointów ramienia, w kolejności oczekiwanej przez arm_controller.
inline const std::vector<std::string> kArmJoints = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

struct JointLimit
{
  double lower;
  double upper;
};

// Jointy 1, 3, 5, 7 są "continuous" w prawdziwym Gen3 (bez ograniczenia
// pozycji) - reprezentujemy to jako brak limitu (+/-inf). Używane tylko do
// ostrzegania/przycinania wartości wpisanych z klawiatury, nie do
// sprawdzania kolizji.
inline const std::array<JointLimit, 7> kJointLimits = {{
  {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
  {-2.24, 2.24},
  {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
  {-2.57, 2.57},
  {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
  {-2.09, 2.09},
  {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
}};

// Dobrze wyśrodkowana, bezpieczna poza startowa (ta sama co
// config/initial_positions.yaml).
inline const std::vector<double> kPresetHome = {0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57};
inline const std::vector<double> kPresetVertical = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Przycina wartość do limitu jointu o danym indeksie (w kArmJoints), z
// ostrzeżeniem na stdout jeśli coś przycięto.
inline double clamp_joint(std::size_t index, double value)
{
  const auto & limit = kJointLimits.at(index);
  const double clamped = std::max(limit.lower, std::min(limit.upper, value));
  if (clamped != value) {
    std::cout << "  [uwaga] " << kArmJoints.at(index) << "=" << value
               << " poza limitem [" << limit.lower << ", " << limit.upper
               << "] -> przycięto do " << clamped << "\n";
  }
  return clamped;
}

}  // namespace kinova_moveit
