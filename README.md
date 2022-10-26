## Установка пакета на ubuntu 20.04:
Установка оптимизатора nlopt:
```
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
sudo apt-get install libarmadillo-dev
cd ${YOUR_WORKSPACE_PATH}/src
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
