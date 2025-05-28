FROM elainasuki/ros:ros2-jazzy-full
# 定义用户和用户组
ARG USERNAME=Elaina
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#安装必要软件
ARG ROS_DISTRO=jazzy
#添加orbbec深度相机驱动
WORKDIR /home/$USERNAME/packages
#基础依赖包和功能包
RUN sudo apt-get update && sudo apt-get install -y \
    libgflags-dev \
    nlohmann-json3-dev \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joy \
    libdw-dev
RUN mkdir -p ./orbbec_ws/src/ && cd ./orbbec_ws/src/ && \
    git config --global http.sslVerify false && git config --global http.postBuffer 524288000 && \
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# 构建 orbbec_ws 工作区
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH} && \ 
    cd orbbec_ws &&\
    # 构建整个 packages 工作区
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    echo "source ~/packages/orbbec_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
#添加realsense驱动
RUN sudo apt-get update \
    && sudo apt-get install -y ros-${ROS_DISTRO}-realsense2-camera 

#添加ms200驱动
COPY  --chown=${USERNAME}:${USER_GID} ./packages/ms200_ros/oradar_ros /home/${USERNAME}/packages/ros_ws/src/oradar_ros 
COPY --chown=${USERNAME}:${USER_GID} ./packages/nagisa_imu /home/${USERNAME}/packages/ros_ws/src/nagisa_imu 
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd ./ros_ws && colcon build \
    && echo 'source ~/packages/ros_ws/install/setup.bash' >> /home/$USERNAME/.bashrc
#添加轮趣imu驱动
COPY --chown=${USERNAME}:${USER_GID} ./packages/wheel_imu /home/${USERNAME}/packages/ros_ws/src/wheel_imu
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd ./ros_ws && colcon build --packages-select  serial \
    && . ./install/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
#添加bashrc配置
USER root
#增加docker 嵌套支持与usb_cam支持
RUN apt-get update && apt-get install docker.io docker-compose ros-${ROS_DISTRO}-usb-cam ros-${ROS_DISTRO}-rosbridge-suite   ros-${ROS_DISTRO}-imu-tools  -y
USER $USERNAME

RUN pip3 install pyzmq crcmod pyserial --break-system-packages
USER root

RUN  echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc \
    #&& echo 'source ~/docker/ros2/packages/ros-jazzy-ros1-bridge/install/setup.bash'>> /home/${USERNAME}/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc \ 
    && echo 'source ~/ros2_driver/install/setup.bash' >> /home/$USERNAME/.bashrc \  
    && echo 'bass source ~/ros2_driver/install/setup.bash' >> /home/$USERNAME/.config/fish/config.fish \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/$USERNAME/.config/fish/config.fish 
CMD ["/bin/bash"]
