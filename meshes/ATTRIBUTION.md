# Pochodzenie mesh-y

Pliki .dae w tym katalogu pochodzą bez zmian z oficjalnego repozytorium:

https://github.com/Kinovarobotics/ros2_kortex (pakiet `kortex_description`,
katalog `arms/gen3/7dof/meshes/`)

Licencja: BSD-3-Clause (plik LICENSE w repozytorium).
Copyright (c) 2018, Kinova inc.

Geometria (originy jointów, limity, inercje) w `kinova_gen3.urdf.xacro`
została przepisana ręcznie na podstawie tego samego źródła
(`arms/gen3/7dof/urdf/gen3_macro.xacro`), ale SRDF, kinematyka, kontrolery
i launch w tym workspace są napisane od zera - nie korzystamy z gotowego
pakietu `kinova_gen3_7dof_robotiq_2f_85_moveit_config`.
