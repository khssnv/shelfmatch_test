# ShelfMatch test

## Установка
Репозиторий содержит `catkin workspace` с пакетами для запуска симуляции `turtlebot3` и процедуры обхода одной из полок по О-образному пути вокруг неё, сохраняя направление одной из сторон - "лица" робота" - в сторону полок на всём пути.
Для установки требуется `Ubuntu 20.04` с ROS Noetic Full Desktop и пакетами `catkin-tools` и `rosdep`.

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
