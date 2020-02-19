# Prevenção de colisões por controle preditivo

Este repositório contém os algoritmos utilizados para realizar estratégias de controle local em robôs autônomos navegando em cenários multiagentes, de forma que cada robô recebe uma trajetória de referência e precisa realizar comandos de velocidade / aceleração que evitem situações de colisão, sem a necessidade de replanejamento, nem comunicação direta entre os agentes.

## Sobre o código

Os algoritmos foram desenvolvidos para rodar no ecossistema [ROS](https://www.ros.org/), especificamente na distribuição *Melodic* rodando no Ubuntu 18.04, mas deve funcionar em outras distribuições igualmente.

## Dependências

* [ROS Melodic](https://www.ros.org/)
* [Python 2.7](https://www.python.org/)
* [Gazebo](http://gazebosim.org/) *
* [OSQP (Operator Splitting Quadratic Program)](https://osqp.org/)

\* Geralmente é adquirido a partir de uma instalação completa do ROS

## Instalação das bibliotecas

### ROS

Um guia de instalação completo do ROS Melodic pode ser encontrado na [página oficial](http://wiki.ros.org/melodic/Installation). Neste trabalho, foi obtida a versão do repositório oficial, com a instalação completa no Ubuntu 18.04. Sumarizando, foram executados os seguintes comandos:

`$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

`$ sudo apt update`

`$ sudo apt install ros-melodic-desktop-full`


### OSQP

O OSQP possui implementações compatíveis com diversas [linguagens](https://osqp.org/docs/get_started/). Nesse trabalho, o *solver* foi utilizado em uma aplicação ROS desenvolvida em Python. A partir do gerenciador de pacotes [pip](https://pypi.org/project/pip/), pode-se facilmente instalar a biblioteca com o comando:

`$ pip install osqp`

## Simulações

Considerando que os pacotes foram devidamente instalados no *workspace* do ROS (via [catkin](http://wiki.ros.org/pt_BR/ROS/Tutorials/InstallingandConfiguringROSEnvironment)), primeiramente, é interessante iniciar o ROS MASTER:

`$ roscore`

Foi desenvolvida uma especificação *URDF* para um robô diferencial genérico, dois mundos foram criados para abordar simulações com apenas um robô e outra com múltiplos robôs. Para rodar uma simulação, é só executar:

`$ roslaunch gazebo_worlds world0.launch`

ou 

`$ roslaunch gazebo_worlds world1.launch`

Para rodar o controlador MPC em um robô apenas, é necessário configurar seu ponto inicial e final no arquivo *controller.py* dentro do pacote *mpc* e rodar

`$ rosrun mpc controller.py`

Já o MPC-ORCA, por questões de escalabilidade, utiliza um arquivo de *launch* com as posições iniciais e desejadas dos robôs.

`$ roslaunch mpc_orca start_controllers.launch`

## Trajetória

A função de trajetória padrão se trata de uma função sigmóide com parâmetros definidos em *controller.py*, mas podem ser alteradas para funções genéricas na seção *Global Path Planning* do referido arquivo.