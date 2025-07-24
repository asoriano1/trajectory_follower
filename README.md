# trajectory_follower

Paquete ROS2 Python para ejecutar trayectorias basadas en waypoints (YAML) y evaluar algoritmos de localización como AMCL en simulación.  
Incluye nodo configurable, soporte para Gazebo y ejemplo de launch.

## Instalación

```bash
cd ~/ros2_py_ws/
colcon build --symlink-install
source install/setup.bash
```
## Uso
```bash
ros2 launch trajectory_follower trajectory_follower.launch.py
```



