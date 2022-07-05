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


https://user-images.githubusercontent.com/92921814/177432412-884904ab-f8b8-4444-ac9c-fb112c3752e5.mp4



# Explicação dos scripts e pastas
 
## CARLA:
Contem todos os scripts e informação relacionada à CARLA 


**camera.py :**

Script para abrir uma câmera no CARLA com determinadas configurações, em uma determinada cidade.

**conducaoautonoma.py :**

Início do script para condução autónoma através da deteção das linhas.

**lidar3d.py :**

Código para abrir o sensor Lidar3D no carro do CARLA.

**mudar_cidade_input :**

Código para mudar a cidade durante a utilização do CARLA.

**readvideoroad.py :**

Código para detetar as linhas da estrada da cidade.

## Deteção de linhas_Imagens:


**readimageroad :**

Código para ler qualquer imagem retirada do Google.

**readvideoexemplo.py :**

Código para ler o vídeo retirado do Google.

## FNR:

--> Github do professor Miguel Riem para download do mundo e robô : https://github.com/miguelriemoliveira/PSA-P2-21-22

### Scripts :

**driver.py :**

Código criado originalmente pelo professor mas com adição da deteção das linhas da esquerda e direita.

**driver_fnr :**

Código para deteção de sinais, passadeira, condução autónoma através da linha guia (direita)-

**driver_fnr_obstacles :**

Código de deteção de obstáculos e de travagem perante os mesmos.



