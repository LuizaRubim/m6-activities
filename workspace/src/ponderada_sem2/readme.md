# Atividade ponderada semana 2 - Turtlesim

Essa atividade compreende os fundamentos básicos de ROS e movimentação do robô por meio da simulação Turtlesim

Para executar o programa, primeiro precisamos criar um workspace, navegar até a pasta `src` e clonar o repositório abaixo:


```bash
git clone https://github.com/LuizaRubim/m6-activities/tree/main/workspace/src/ponderada_sem2 
```

<div class="callout">
obs: caso ainda não tenha criado um workspace, siga o tutorial https://rmnicola.github.io/m6-ec-encontros/workspaces
</div>

Agora para instalar as dependências que o seu pacote necessita, utilizaremos o rosdep. Portanto, caso não tneha instalado, siga os três comandos abaixo:

```bash
sudo apt install python3-rosdep
```
Após instalar, precisamos configurar o rosdep. Assim, execute esses dois comandos no terminal:

```bash
sudo rosdep init
```

```bash
rosdep update
```

volte para a raíz do seu workspace e execute o comando abaixo para instalar as dependências:







