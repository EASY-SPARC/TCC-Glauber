## Simulações

Foi desenvolvida uma especificação *URDF* para um robô diferencial genérico, dois mundos foram criados para abordar simulações com apenas um robô e outra com múltiplos robôs. Para rodar uma simulação, é só executar:

`$ ros2 launch gazebo_worlds world0.launch`

Para rodar o controlador MPC em um robô apenas, pode-se configurar uma posição objetivo (pos_x, pos_y), já que o script gerará uma trajetória usando função logística:

`$ ros2 run mpc mpc <pos_x> <pos_y>`

