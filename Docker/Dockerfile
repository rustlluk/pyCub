ARG BASE_IMAGE=nvidia/cuda:11.0.3-devel-ubuntu20.04
FROM $BASE_IMAGE

LABEL maintainer="Lukas Rustler <lukas.rustler@fel.cvut.cz>"

#Non-interactive mode
ENV DEBIAN_FRONTEND='noninteractive'

#Change for other python; 3.11 may be too much right now
ARG PYTHON_VER=3.11

#Install all neccesary thingies
RUN apt update -y && apt install software-properties-common -y && add-apt-repository ppa:deadsnakes/ppa -y  \
    && apt update && apt install python$PYTHON_VER wget sudo gedit unzip apt-utils curl \
    python$PYTHON_VER-venv nano mesa-utils curl htop net-tools sshfs python$PYTHON_VER-distutils  \
    screen git python$PYTHON_VER-dev libpng-dev libqhull-dev libfreetype6-dev libfreetype6  \
    pkg-config python3.8-venv python3.8-dev -y

#Install pips
# 3.8 is default at Ubuntu 20.04 and is fine for thing that do not work on newer versions
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python$PYTHON_VER &&  \
    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.8

#Default args, that can be changed during build
ARG UID=1000
ARG GID=1000

# SSH
RUN apt install openssh-server -y && sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config &&  \
    echo Port 2222 >> /etc/ssh/sshd_config && service ssh start

#Add docker user with correct UID and GID; and add him to sudoers
RUN groupadd -g $GID docker_users && useradd -l -u $UID -G $GID -md /home/docker -s /bin/bash docker && \
    echo 'docker:docker' | chpasswd && usermod -aG sudo docker && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers && \
    echo "docker  ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers

# PyCharm
ARG PYCHARM_VER=2023.2.3
RUN cd /opt && wget https://download.jetbrains.com/python/pycharm-community-$PYCHARM_VER.tar.gz &&  \
    tar -xvf pycharm-community-$PYCHARM_VER.tar.gz && rm pycharm-community-$PYCHARM_VER.tar.gz &&  \
    mv pycharm-community-$PYCHARM_VER pycharm && ln -s /opt/pycharm/bin/pycharm.sh /usr/bin/pycharm && chmod +x /usr/bin/pycharm

RUN ln -sf /usr/bin/python$PYTHON_VER /usr/bin/python && ln -sf /usr/bin/python$PYTHON_VER /usr/bin/python3 &&  \
    ln -sf /usr/local/lib/python$PYTHON_VER/dist-packages/pip /usr/bin/pip3 && ln -sf /usr/bin/pip3 /usr/bin/pip

# pybullet, robot_description, numpy, etc.
RUN MAKEFLAGS="-j$((`nproc`-2))" python3 -m pip install pybullet numpy scipy cython matplotlib --verbose --no-cache-dir
# open3D for 3.11 now only through development packages
RUN python3 -m pip install -U --trusted-host www.open3d.org -f http://www.open3d.org/docs/latest/getting_started.html open3d --ignore-installed --no-cache-dir
RUN python3 -m pip install sphinx sphinx_rtd_theme m2r2 && echo "export PATH=$PATH:/home/docker/.local/bin" >> /home/docker/.bashrc
RUN apt update && apt install latexmk texlive-latex-extra -y

# Delete thingies from aptitude
RUN rm -Rf /var/lib/apt/lists/*

WORKDIR /home/docker
ENTRYPOINT sudo service ssh start >> /dev/null && /bin/bash