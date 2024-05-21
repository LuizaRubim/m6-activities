# Teleoperação do robô

Este projeto permite a teleoperação de um robô utilizando o ROS2 e comandos do teclado. Siga as instruções abaixo para configurar e executar o projeto.

## Pré-requisitos

- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) ou versão posterior.
- Python 3.8 ou superior.

Instalar o webots:
 siga o tutorial para o passo a passo: `https://rmnicola.github.io/m8-ec-encontros/sprint2/encontro4/nav2`

 Obs: para esse projeto, precisaremos somente dos passos e 1 5 do tutorial


## Passo a Passo

### 1. Configuração do Ambiente ROS2

Instale o ROS2 conforme a [documentação oficial](https://docs.ros.org/en/humble/Installation.html).

### 2. Clonagem do Repositório

```bash
cd ~/workspace/src
git clone https://github.com/LuizaRubim/m6-activities

```

### 3. Compilação do Projeto

```bash
cd ~/workspace
colcon build
source install/setup.bash
```

### 4. Abrir o webots após instalado


```bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```


### 4. Execução do Projeto

Para executar o projeto, abra um terminal e execute o comando abaixo:
```bash
ros2 run ponderada_sem3 move
```
Para movimentar, use as teclas `W`, `A`, `X` e `D` para frente, esquerda, trás e direita, respectivamente. Para parar o robô, aperte `S`.


## Vídeo de Demonstração

`https://youtu.be/z58X15WlP6w`