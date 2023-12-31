###########################################
#   BASE IMAGE
###########################################
FROM ubuntu:20.04 AS base
ENV DEBIAN_FRONTEND=noninteractive



# ###########################################
# #   e:fs PROXY SETTINGS
# ###########################################
# # Setup Proxy for e:fs environment
# # Check whether docker is being used on developer laptop or vehicle platform 
# ARG PROXY
# ARG PROXY_UPPERCASE
# # Set PROXY variable to PROXY_UPPERCASE if PROXY is empty
# # This should handle cases where the system defines a `HTTP_PROXY` variable but no `http_proxy` variable
# ARG PROXY=${PROXY:-${PROXY_UPPERCASE}}
# ARG SET_PROXY
# ARG PROXY_ARG=${SET_PROXY:+"\"${PROXY}\";"}
# ARG t1=${SET_PROXY:+"Acquire::http::Proxy "}
# ARG t2=${SET_PROXY:+"Acquire::https::Proxy "}
# # Setup proxy for apt to be used on developer laptops
# # Empty lines will be written to the file in case of the environment being the vehicle platform
# RUN echo "${t1}${PROXY_ARG}" >> /etc/apt/apt.conf.d/proxy.conf
# RUN echo "${t2}${PROXY_ARG}" >> /etc/apt/apt.conf.d/proxy.conf

# # Set proxys as environment variables for shells and commands in container to have internet access on developer laptops
# # The environment variables will be set to be empty if the environment is the vehicle plattform
# ARG ENV_PROXY=${SET_PROXY:+${PROXY}}
# ENV HTTP_PROXY=${ENV_PROXY} 
# ENV HTTPS_PROXY=${ENV_PROXY}
# ENV http_proxy=${ENV_PROXY}
# ENV https_proxy=${ENV_PROXY}



# NOTE: Next three sections are based on
# https://github.com/athackst/dockerfiles/blob/main/ros2/foxy.Dockerfile

###########################################
#   BASE IMAGE WITH foxy ROS
###########################################
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnome-terminal \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 base
RUN sudo add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-foxy-ros-base \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.10/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=
ENV EDITOR=vim

###########################################
#   DEVELOP IMAGE
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    ros-dev-tools \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
    && rm -rf /var/lib/apt/lists/* 

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#   FULL IMAGE
###########################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=


# ###########################################
# #   Post install stuff
# ###########################################
FROM full as efs

USER $USERNAME

RUN pip3 install --upgrade pip \
    && python3 -m pip install \
        carla==0.9.14 \
        casadi \
        networkx \
        pygame \
        simple-pid \
        transforms3d

SHELL [ "/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash

RUN echo "source \$WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
