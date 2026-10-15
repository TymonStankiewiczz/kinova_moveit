# kinova_moveit

Pakiet ROS 2 (Humble): URDF + konfiguracja MoveIt2 dla ramienia **Kinova
Gen3** (7-DOF) sterowanie z klawiatury.

- 7 jointów: `joint_1`..`joint_7` (1/3/5/7 są `continuous`, bez limitu pozycji)
- grupa MoveIt: `arm`
- end-effector: `end_effector_link`
- kontroler: `arm_controller`

Mesh-e i geometria pochodzą z
[Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex)
(BSD-3-Clause, patrz `meshes/ATTRIBUTION.md`).

## Zależności

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager
```

## Budowanie

```bash
mkdir -p ~/ros2_ws/src
cp -r kinova_moveit ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kinova_moveit
source install/setup.bash
```

## Uruchomienie

**Terminal 1**: symulacja (mock hardware) + MoveIt2 + RViz:

```bash
ros2 launch kinova_moveit demo.launch.py
```

**Terminal 2**: jeden z węzłów sterujących:

```bash
ros2 run kinova_moveit joint_teleop             # wpisywanie pozycji jointów
ros2 run kinova_moveit keyboard_joy_teleop      # dżojowanie pojedynczych jointów
ros2 run kinova_moveit keyboard_cartesian_plan  # sterowanie kiścią w przestrzeni kartezjańskiej
```

### joint_teleop

```
j p1 p2 p3 p4 p5 p6 p7  - wszystkie 7 jointów naraz (radiany)
home                     - preset "home"
vertical                 - preset "vertical" (wszystkie jointy = 0)
t sekundy                - czas trwania kolejnych ruchów (domyślnie 2s)
help, q
```

### keyboard_joy_teleop

```
1..7   - wybierz joint      w/s - zwiększ/zmniejsz o krok
[ / ]  - zmniejsz/zwiększ krok     h - home    v - vertical
```

### keyboard_cartesian_plan

Każdy ruch jest planowany przez `/compute_cartesian_path`. Cel
nieosiągalny jest odrzucany.

```
w/s - przód/tył   a/d - lewo/prawo   q/e - góra/dół
m       - przełącz tryb: translate <-> rotate
[ / ]   - zmniejsz / zwiększ krok (domyślnie 3mm / 0.03rad)
```
