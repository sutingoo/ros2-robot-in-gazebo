# Robot Autónomo con ROS 2 Humble

Este repositorio contiene el desarrollo de un robot móvil diferencial simulado en Gazebo, capaz de realizar navegación autónoma básica mediante visión artificial.

[Gazebo](imagenes/gazebo_autonomous_one.gif)
[RQT-IMAGE-VIEW](imagenes/rqt-prueba-one.gif)
## Progreso del Proyecto

### 1: Fundamentos y Simulación
- Configuración del entorno de desarrollo (Ubuntu 22.04 + ROS 2 Humble).
- Creación de paquetes y estructura de workspace.
- **Simulación:** Lanzamiento de mundos básicos en Gazebo Classic.

### 2: Percepción y Control (Estado Actual)
- **Diseño URDF:** Robot con física realista, chasis y ruedas (Skid-Steer/Diferencial).
- **Sensores:** Integración de LIDAR (Rays) y Cámara RGB.
- **Visión Artificial:** Nodo de procesamiento de imágenes con OpenCV (Detección de carril amarillo).
- **Control:** Implementación de un controlador **PID** para seguimiento de línea autónomo.

[Foto del Robot en Rviz - Vectores xyz](media/Rviz_one.png)
## Cómo ejecutar la simulación

1. **Lanzar el entorno y el robot:**
   ```bash
   ros2 launch my_bot_description launch_sim.launch.py

[Media en Gazebo](media/Gazebo_laser_two.png)
