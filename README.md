# vsss-simulator
![Imagem](vsss_simulator.png "VSSS Simulator")

Simulador para a categoria IEEE Very Small Size da LARC/CBR utilizando Gazebo + ROS.

Este projeto é um trabalho de conclusão de curso em Sistemas de Informação da UNESP/Bauru.

## Instalar ROS
http://wiki.ros.org/melodic/Installation/Ubuntu

## Instalar simulador [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
```
sudo apt-get install gazebo9
```

## Instalar dependências para desenvolvimento no Gazebo
```
sudo apt-get install libgazebo9-dev
```

## Criar pasta do projeto
```
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/evertonkelvin/vsss-simulator.git
```

## Compilar aplicação
```
cd ~/catkin_ws
catkin_make
```

## Rodar aplicação

### Rodar simulador gazebo
```
cd ~/catkin_ws
roslaunch vsss_simulator game_ready.launch
```

### Rodar software de controle
* Abrir outro terminal e executar os comandos:
```
cd ~/catkin_ws
roslaunch vsss_control vsss_control.launch
```