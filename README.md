## Установка пакета на ubuntu 20.04:
Установка оптимизатора nlopt:
```
cd ~/catkin_ws/src
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```
Установка и сборка пакета:
```
cd ~/catkin_ws/src
sudo apt update
sudo apt-get install -y libarmadillo-dev
sudo apt install -y --fix-missing ros-noetic-pcl-conversions ros-noetic-pcl-ros
git clone https://github.com/Iliaaer/Fast-Planner
cd ../ 
catkin_make -DCMAKE_CXX_STANDARD=14
bash ~./bashrc
```
## Запуск пакетов:
Запуск визуализации:
```bash
roslaunch plan_manage rviz.launch
```
Запуск полета до точки:
```bash
roslaunch plan_manage kino_replan.launch
```
Запуск полета по прямой:
```bash
roslaunch plan_manage kino_replan.launch
```
