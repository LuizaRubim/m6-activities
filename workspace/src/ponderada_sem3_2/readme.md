### Ponderada semana 3 pt 2

#### Como rodar o projeto:

1. Rodar o launch do simulador webots em um terminal

```bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

2. rodar o servidor websocket do rosbridge:

```bash
ros2 run rosbridge_server rosbridge_websocket
```

3. Em outro terminal, vá até a pasta 'ponderada_sem3_2' e rode o comando:

```bash
python3 camera.py
```

4. EM outro terminal, vá até a pasta 'ponderada_sem3_2' e rode o comando:

```bash
python3 movimentos.py
```

5. Acesse o arquivo 'index.html' em um navegador e controle o seu robô lindamente :)