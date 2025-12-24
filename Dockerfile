#Imagen base de ROS2 Humble
FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

#Copiar el codigo al contenedor
COPY src /ros2_ws/src

#Compilar el codigo en el contenedor
RUN . /opt/ros/humble/setup.sh && \
    colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch turtle_challenge turtle_mission.launch.py"]
