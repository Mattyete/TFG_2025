## Instalación

1. Clona el repositorio:
   ```bash
   git clone https://github.com/Mattyete/TFG_2025.git
   cd TFG_2025
   
2. Instala las dependencias (puedes usar un script o entorno virtual):
   ```bash
   sudo apt update
   rosdep install --from-paths src --ignore-src -r -y
   
3. Compila el workspace:
   ```bash
   colcon build
   source install/setup.bash

## Uso

1. Lanzar la simulación:
   ```bash
   ros2 launch launch/robot_simulation.launch.py
   
2. Ejecutar detección de señales:
   ```bash
   ros2 run signal_detection signal_detector_node

3. Activar la conducción autónoma:
   ```bash
   ros2 launch launch/autonomous_drive.launch.py
