# PSA-P2-G3

Projeto criado no âmbito da UC Projeto em Sistemas de Automação, Departamento de Engenharia Mecânica, Universidade de Aveiro.

**Objetivo:** Desenvolver um Software para condução autónoma em ambiente simulado CARLA,
utilizando o eco sistema ROS para interface com o simulador.

---
## Instalações necessárias:
* ubuntu
* ros
* ros-bridge
* pycharm
* carla simulator
---
## Comandos para inicializar (terminal):

**CARLA:** 
```cd /opt/carla-simulator/```
```./CarlaUE4.sh -prefernvidia -quality-level=Low```

**Código para gerar um veículo e definir uma câmera frontal:**
```python3 camera.py```

**Código para Deteção de Estrada:**
```python3 readvideoroad.py```

**Lançar Ros-bridge:**
```roslaunch carla_ros_bridge carla_ros_bridge.launch```

**Mudar de Mapa durante a utilização do carla:**
```cd /opt/carla-simulator/PythonAPI/util```
```python3 config.py -m TownXX```
**ou**
```python3 mudar_de_cidade.py```

![WhatsApp Video 2022-05-16 at 10 55 49 (1)](https://user-images.githubusercontent.com/101104928/168574549-b0bbb146-e772-4d52-a4b5-adaa12701e73.gif)
---


# Explicação dos scripts e pastas
 
## CARLA

Contem todos os scripts e informação relacionada à CARLA 
**wwv**
