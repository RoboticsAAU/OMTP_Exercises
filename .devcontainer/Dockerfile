FROM althack/ros2:humble-cuda-gazebo-nvidia

ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute,display

# Install nvidia container toolkit
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
        && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
        && \
        sudo apt-get update


# ** [Optional] Uncomment this section to install additional packages. **
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update \
        && apt install -y --no-install-recommends \
        ~nros-humble-rqt* \
        ros-humble-moveit* \
        ros-humble-cv-bridge* \
        nvidia-container-toolkit \
        lshw \
        ros-humble-flexbe-behavior-engine \
        # Clean up
        && sudo apt autoremove -y \
        && sudo apt clean -y \
        && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Install python
RUN apt-get update && \
        apt-get install -y software-properties-common && \
        add-apt-repository -y ppa:deadsnakes/ppa && \
        apt-get update && \
        apt install -y python3.8 python3.8-venv\
        && rm -rf /var/lib/apt/lists/*

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Set up auto-source of gazebo for ros user
RUN echo "if [ -f /usr/share/gazebo/setup.sh ]; then source /usr/share/gazebo/setup.sh; fi" >> /home/ros/.bashrc