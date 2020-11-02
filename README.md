# ShelfMatch test

## Подготовка
Репозиторий содержит `catkin workspace` с пакетами для запуска симуляции `turtlebot3` и процедуры обхода одной из полок по О-образному пути вокруг неё, сохраняя направление одной из сторон - "лица" робота" - в сторону полок на всём пути.
Для установки требуется `Ubuntu 20.04` с ROS Noetic Desktop Full и пакетами `catkin-tools` и `rosdep`.
Вы можете запустить симуляцию в виртуальной машине.
Для этого в репозиторий включён `Vagrantfile`, с помощью которого можно подготовить всё необходимое автоматически.
Чтобы запустить виртуальную машину, на хост-машине должен быть установлен `Vagrant` с провайдером `VirtualBox`.
Для установки `Vagrant` следуйте официальной инструкции [по ссылке](https://www.vagrantup.com/docs/installation).
Перед запуском виртуальной машины убедитесь, что хост-машина имеет 4Гб свободной оперативной памяти, 3 ядра центрального процессора и от 20Гб свободного дискового пространства.
Для запуска виртуальной машины выполните следующие шаги на хост-машине.

1. Копируем проект в домашнюю директорию
```console
cd ~ && git clone --recursive https://github.com/khssnv/shelfmatch_test.git && cd shelfmatch_test
```

2. Запускаем настройку и перезагрузку виртуальной машины
```console
vagrant up && vagrant halt
```

3. Запускаем настроенную виртуальную машину
```console
vagrant up
```

4. Логинимся в окне виртуальной машины
```
Пользователь: vagrant
Пароль: vagrant
```

Следующие шаги выполняйте в виртуальной машине.

## Установка

1. Копируем проект в домашнюю директорию
```console
cd ~ && git clone --recursive https://github.com/khssnv/shelfmatch_test.git && cd shelfmatch_test && catkin init
```

2. Устанавливаем зависимости
```console
rosdep install --from-paths src --ignore-src -y
```

3. Собираем проект
```console
catkin build
```

## Запуск

1. Запускаем симулятор и визуализацию
```console
source devel/setup.bash
TURTLEBOT3_MODEL=waffle_pi roslaunch shelfmatch_task bookstore_turtlebot_navigation.launch
```

2. Запускаем сканер стеллажа
```console
source devel/setup.bash
rosrun shelfmatch_task shelf_follower
```

3. Запускаем процедуру сканирования
```console
rosservice call /shelf_scanner/start_scan "{}"
```
