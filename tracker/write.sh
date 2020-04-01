#!/usr/bin/env bash
<<comment
Снятие данных с кинекта возможно нексолькими способами:
1)   (по-моему, который использовал я изначально)                 rostopic echo /tf >> Kinect_TF_msg.txt
2.a) (получше, потому что записывает в проигрываемый bag файл)    rosbag record /tf
2.b) (альтернатива предыдущему, и можно задать путь сохранения)   rostopic echo -b ~/Project/bags/Kinect_TF_msg.bag /tf
     (больше на       http://wiki.ros.org/rosbag/Commandline)
comment

#gnome-terminal --tab -e "command1" --tab -e "command2

gnome-terminal
roscore

gnome-terminal --tab
roslaunch openni_launch openni.launch

gnome-terminal --tab
rosrun openni_tracker openni_tracker

gnome-terminal --tab
read -p "Ожидание заверешния калибровки" -n 1 -s  
#ожидание нажатия одной клавиши в "тихом" режиме (возможно можно сделать лучше)
#необходимо в связи с тем, что мы не знаем, когда openni_tracker выполнит калибровку и начнет слать данные

rostopic echo -b ~/Project/bags/Kinect_TF_msg.bag /tf   #запись данных с кинекта в bag file для возможности проигрывания

gnome-terminal --tab
python3 ~/Project/getData.py #скрипт по получению данных с датчика по сериал порту
