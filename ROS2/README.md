## Instalación

1. Clona el repositorio (theConstruct):
   ```bash
   git clone https://github.com/Mattyete/TFG_2025.git
   cd TFG_2025
   
2. Hacer el script ejecutable i instalar las dependencias (theConstruct):
   ```bash
   chmod +x ~/TFG_2025/ROS2/ros2_ws/src/limo_vision/scripts/keras_detector_v2.py
   sudo pip install keras
   sudo pip install tensorflow
   
3. Compila el workspace (theConstruct):
   ```bash
   cd ~/TFG_2025/ROS2/ros2_ws
   colcon build
   source install/setup.bash
   
## Uso

1. Lanzar la cámara ORBBEC (LIMO):
   ```bash
   ros2 launch astra_camera dabai.launch.py

2. Verificar el topic (theConstruct):
   ```bash
   ros2 topic list
   
3. Lanzar el nodo de clasificación (theConstruct):
   ```bash
   ros2 launch limo_vision keras_detector.launch.py

4. Parar o seguir haciendo fotos (theConstruct) (true/false):
   ```bash
   ros2 topic pub /capture_toggle std_msgs/Bool "data: false"
