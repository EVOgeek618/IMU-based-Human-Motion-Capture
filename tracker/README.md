Для отслеживания точек тела с помощью kinect была использована библиотека для ROS  под названием openni_tracker.

К сожалению, большинство библиотек не были обновлены для ROS Melodic, предназначенной для Ubuntu 18.04, поэтому при начале работы с этой библиотекой возникнут определенные трудности. Я попытаюсь детально описать шаги, которые необходимо сделать для начала работы, а также трудности, с которыми я встретился, и как я с ними справился.


**Эти команды устанавливают необходимые пакеты и драйверы:**

sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-8-jdk

sudo apt-get install doxygen graphviz mono-complete


**Создаем директорию, в которой мы будем собирать и устанавливать остальные пакеты:**

cd ~/

mkdir kinect


**Установка OpenNI:**

cd ~/kinect

git clone https://github.com/OpenNI/OpenNI.git

cd OpenNI

git checkout Unstable-1.5.4.0

cd Platform/Linux/CreateRedist

chmod +x RedistMaker


_Следующий шаг у меня вызвал проблемы, поэтому сначала путь их решения_

Пакет старый, поэтому на современных компиляторах gcc и g++ 7.0 выдает ошибку при сборке. Решить эту проблему поможет установка и временный переход на gcc и g++ 4.8:

sudo apt-get install gcc-4.8

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 30

sudo update-alternatives --config gcc

После последней команды необходимо выбрать gcc версии 4.8, следуя инструкции на экране (Нажать клавишу с цифрой, соответсвующей нашему выбору).

После повторяем те же шаги для g++:

sudo apt-get install g++-4.8

sudo update-alternatives --config g++

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 40

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 30

sudo update-alternatives --config g++

И опять выбираем компилятор версии 4.8, нажав необходимую клавишу.


**Обойдя проблемы с компилятором, можно продолжить сборку**

./RedistMaker

cd ../Redist/OpenNI-Bin-Dev-Linux-[xxx] 

(где [xxx] это ваша архитекутра и номер релиза OpenNI)
sudo ./install.sh


_После установки OpenNI не забудьте вновь выбрать последние версии gcc и g++, иначе у вас возникнут проблемы со сборкой всего:_

sudo update-alternatives --config gcc

sudo update-alternatives --config g++


**Установка SensorKinect:**

cd ~/kinect

git clone https://github.com/avin2/SensorKinect

cd SensorKinect

cd Platform/Linux/CreateRedist

chmod +x RedistMaker

./RedistMaker

cd ../Redist/Sensor-Bin-Linux-[xxx] 

(где [xxx] опять же ваша архитектура и номер релиза бибилотеки)


chmod +x install.sh

sudo ./install.sh

_ОСТОРОЖНО: Во избежание проблем не стоит устанавливать это больше одного раза. Если нужно что-то изменить, то перейдите в директорию ./install и запустите sudo ./install.sh -u_


**Установка NITE:**

cd ~/kinect

git clone https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23

cd NITE-Bin-Dev-Linux-v1.5.2.23/[xxx]

(где [xxx] это ваша архитектура)


sudo ./install.sh


**Установка библиотек для ROS:**

sudo apt-get install ros-melodic-openni-launch ros-melodic-openni-launch


**Сборка и установка openni_tracker:**

cd ~/catkin_ws/src

git clone https://github.com/ros-drivers/openni_tracker.git

cd ~/catkin_ws

catkin_make

catkin_make install


**Запуск осуществляется командами:**

roscore

roslaunch openni_launch openni.launch

rosrun openni_tracker openni_tracker

_**Альтернатива запуска**_

Используйте скрипт write.sh:

./openni_launch_script.sh

