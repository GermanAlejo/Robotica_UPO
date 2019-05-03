# Robotica_UPO
Repositorio con todas las epds correspondientes a la asignatura de robotica de la upo

Usefull commads:

source ./devel/setup.bash
catkin_create_pkg nombrepkg roscpp -> crea paquetes
export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/empty.world -> cambia a mundo vacio
roslaunch turtlebot_gazebo turtlebot_world.launch -> lanza mundo
roslaunch turtlebot_teleop keyboard_teleop.launch -> lanza teleop para controlar manualmente el robot
rqt 
rosrun rviz rviz -> lanza visualizador
rosrun pkg ejercicio -> lanza codigo
rosrun tf tf_echo /world /frame_1 -> antes de ejecutar rosrun rviz rviz pkg
