# kinova_moveit

Samodzielny pakiet ROS 2 (Humble): URDF/MoveIt2 dla ramienia **Kinova
Gen3** (7-DOF, bez chwytaka) + trzy węzły **C++** do sterowania z
klawiatury. Jeden folder = jeden pakiet, gotowy do sklonowania do `src/`
dowolnego workspace'u na innym komputerze.

## Co tu jest, a czego nie ma

- **Geometria** (URDF/xacro + mesh-e) - przepisana ręcznie z oficjalnego
  [Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex)
  (pakiet `kortex_description`, BSD-3-Clause, patrz `meshes/ATTRIBUTION.md`).
- **MoveIt2** (SRDF, kinematyka, kontrolery, `demo.launch.py`) - napisane
  od zera, nie z gotowego configu producenta.
- **Trzy węzły teleop w C++** - przepisane z działającej wersji Python
  (`kinova_ws`): `joint_teleop`, `keyboard_joy_teleop`,
  `keyboard_cartesian_plan`.
- Czego **nie ma**: chwytaka, `drive_bridge`, demo leader-follower - to
  celowo pominięte, nieaktualne/niepotrzebne części poprzednich wersji.

7 jointów: `joint_1`..`joint_7` (1/3/5/7 są `continuous` - bez ograniczenia
pozycji, tak jak w prawdziwym Gen3). Grupa MoveIt `arm`, end-effector
`end_effector_link`, kontroler `arm_controller`.

## Wymagane zależności

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

**Terminal 1** - symulacja (mock hardware) + MoveIt2 + RViz:

```bash
ros2 launch kinova_moveit demo.launch.py
```

**Terminal 2** - jeden z trzech sposobów sterowania (wybierz, który Cię
interesuje):

```bash
ros2 run kinova_moveit joint_teleop             # wpisywanie pozycji jointów
ros2 run kinova_moveit keyboard_joy_teleop      # dżojowanie pojedynczych jointów
ros2 run kinova_moveit keyboard_cartesian_plan  # sterowanie kiścią w przestrzeni kartezjańskiej
```

### joint_teleop

```
j p1 p2 p3 p4 p5 p6 p7  - wszystkie 7 jointów naraz (radiany)
home                     - preset "home" (bezpieczna, wyśrodkowana poza)
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

Każdy ruch jest **planowany** (`/compute_cartesian_path`), nie sterowany
strumieniem prędkości - jeśli cel jest nieosiągalny, węzeł go odrzuca
zamiast robić coś nieprzewidywalnego.

```
w/s - przód/tył   a/d - lewo/prawo   q/e - góra/dół
m       - przełącz tryb: translate (przesuwanie) <-> rotate (obrót)
[ / ]   - zmniejsz / zwiększ krok
```

Domyślny krok jest celowo mały (3mm / 0.03rad) - narzut jednego cyklu
(serwis + akcja + monitorowanie wykonania) jest z grubsza stały niezależnie
od prędkości jointów, więc żeby trzymanie klawisza wyglądało płynnie, same
kroki muszą być drobne. `[`/`]` pozwalają to dostroić.

Gen3 ma 7 DOF (nadmiarowe względem 6 wymaganych do pełnej kontroli
pozycji+orientacji), więc w przeciwieństwie do mniej licznych ramion ruch
translacyjny przy stałej orientacji powinien niemal zawsze się udawać
wprost, bez potrzeby dopuszczania dryfu orientacji.

## Status weryfikacji

To, co faktycznie sprawdziłem w tej sesji (nie tylko "składnia się zgadza"):

- `xacro` realnie przetworzył `urdf/kinova_gen3.urdf.xacro` do poprawnego,
  płaskiego URDF (zero nierozwiniętych `${}`/`$()`).
- `check_urdf` potwierdził poprawny łańcuch kinematyczny: `world -> base_link
  -> shoulder_link -> half_arm_1_link -> half_arm_2_link -> forearm_link ->
  spherical_wrist_1_link -> spherical_wrist_2_link -> bracelet_link ->
  end_effector_link`.
- Wszystkie trzy węzły C++ **skompilowały się bez ani jednego ostrzeżenia**
  (`-Wall -Wextra -Wpedantic`).
- `demo.launch.py` wystartował `move_group` poprawnie aż do komunikatu
  "You can start planning now!" (czyli SRDF/kinematyka/plugin OMPL się
  zgadzają).

Czego **nie** udało się zweryfikować: samego ruchu ramienia na żywo w
RViz/z poziomu węzłów teleop - w tej sesji `ros2_control`/`controller_manager`
w ogóle nie ładował żadnych kontrolerów (`joint_state_broadcaster`,
`arm_controller`), i to **dla każdego pakietu**, łącznie z `so_arm100_moveit_config`,
który wcześniej u Ciebie realnie działał - czyli to problem tego
konkretnego środowiska sandboxowego (brakująca/niekompatybilna biblioteka
pluginów), a nie błąd w tym configu. Na normalnej maszynie z działającym
`ros2_control` (tak jak Twoja) powinno to wstać bez przeszkód, ale pierwsze
uruchomienie i tak warto zrobić uważnie patrząc na logi.
