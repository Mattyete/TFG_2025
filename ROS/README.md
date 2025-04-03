## Instalación

1. Clona el repositorio (theConstruct):
   ```bash
   git clone https://github.com/Mattyete/TFG_2025.git
   cd TFG_2025
   
2. Hacer el script ejecutable i instalar las dependencias (theConstruct):
   ```bash
   chmod +x ~/catkin_ws/src/limo_vision/scripts/keras_detector.py
   sudo pip install keras
   sudo pip install tensorflow
   
3. Compila el workspace (theConstruct):
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

## Uso

1. Lanzar la cámara ORBBEC (LIMO):
   ```bash
   roslaunch astra_camera dabai_u3.launch

2. Verificar el topic (theConstruct):
   ```bash
   rostopic list
   
3. Lanzar el nodo de clasificación (theConstruct):
   ```bash
   roslaunch limo_vision detect_signs.launch

4. Parar o seguir haciendo fotos (theConstruct) (true/false):
   ```bash
   rostopic pub /capture_toggle std_msgs/Bool "data: true"
