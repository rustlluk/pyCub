ARG BASE_IMAGE=ubuntu:24.04
FROM $BASE_IMAGE

LABEL maintainer "Lukas Rustler <lukas.rustler@fel.cvut.cz>"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ARG TARGETPLATFORM

#Install all neccesary thingies
RUN apt update -y && apt install software-properties-common -y && add-apt-repository ppa:deadsnakes/ppa -y  \
    && apt update && apt install wget sudo gedit unzip apt-utils curl \
    nano mesa-utils curl htop net-tools sshfs \
    screen git libpng-dev libqhull-dev libfreetype6-dev libfreetype6  \
    pkg-config python3-venv python3-dev apt-transport-https psmisc tmux gdb gitk autoconf locales gdebi  \
    terminator meld dos2unix -y

# Set the locale
RUN locale-gen en_US.UTF-8

#Install pip
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.12 - --break-system-packages

# VSCode
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ] ; then \
       wget -O code.deb "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64"; \
    else \
       wget -O code.deb https://go.microsoft.com/fwlink/?LinkID=760868; \
    fi

RUN gdebi -n code.deb && rm code.deb

# PyCharm
ARG PYCHARM_VER=2025.2
RUN cd /opt && wget https://download.jetbrains.com/python/pycharm-community-$PYCHARM_VER.tar.gz &&  \
    tar -xvf pycharm-community-$PYCHARM_VER.tar.gz && rm pycharm-community-$PYCHARM_VER.tar.gz &&  \
    mv pycharm-community-$PYCHARM_VER pycharm && ln -s /opt/pycharm/bin/pycharm.sh /usr/bin/pycharm && chmod +x /usr/bin/pycharm

RUN apt remove python3-psutil -y && python3 -m pip install --upgrade pip --break-system-packages

#Default args, that can be changed during build
ARG UID=1000
ARG GID=1000

# SSH
RUN apt install openssh-server -y && sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config &&  \
    echo Port 2222 >> /etc/ssh/sshd_config && service ssh start

# Remove the default 'ubuntu' user/group that comes with Ubuntu 24.04
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && \
    userdel -f ubuntu && groupdel ubuntu || true

#Add docker user with correct UID and GID; and add him to sudoers
RUN groupadd -g $GID docker_users && useradd -l -u $UID -G $GID -md /home/docker -s /bin/bash docker && \
    echo 'docker:docker' | chpasswd && usermod -aG sudo docker && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers && \
    echo "docker  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN echo "export PATH=/usr/local/bin:$PATH" >> /home/docker/.bashrc

USER docker
RUN cd /home/docker && git clone https://github.com/rustlluk/pyCub.git && cd pyCub/icub_pybullet && python3 -m pip install --break-system-packages --no-cache-dir -e .

# Delete thingies from aptitude
RUN sudo rm -Rf /var/lib/apt/lists/*

WORKDIR /home/docker
ENTRYPOINT sudo service ssh start >> /dev/null && /bin/bash