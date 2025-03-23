# pyCub Documentation
pyCub is iCub humanoid robot simulator written in Python. It uses PyBullet for simulation and Open3D for visualization.

## Installation  
- Requires python3.8 to python3.11
  - newer python versions are now not supported due to incompatible with some dependencies 
- We recommend using virtual environment when installing from PyPi or from source
  - ```
    python3 -m venv pycub_venc
    source pycub_venc/bin/activate
    OTHER_COMMANDS
    ```
1. **(Recommended)** Install from PyPi (not yet available)  
    - ```python3 -m pip install icub_pybullet```
2. Install from source  
    - Pull this repository  
    - ```
      cd PATH_TO_THE_REPOSITORY
      python3 -m pip install --upgrade pip
      python3 -m pip install .
      ```
3. Native Docker (GNU/Linux only)
   - see [Docker Native Version](#native-version) section
4. VNC Docker 
    - see [Docker VNC Version](#vnc-version) section
5. Gitpod
    - open [https://gitpod.io/#github.com/rustlluk/pycub](https://gitpod.io/#github.com/rustlluk/pycub) 
      and log in with GitHub account

## Examples
- [push_the_ball_pure_joints.py](https://github.com/rustlluk/pycub/blob/master/icub_pybullet/examples/push_the_ball_pure_joints.py) contains an example that
  shows how to control the robot in joint space
- [push_the_ball_cartesian.py](https://github.com/rustlluk/pycub/blob/master/icub_pybullet/examples/push_the_ball_cartesian.py) contains an example that
  shows how to control the robot in Cartesian space
- [skin_test.py](https://github.com/rustlluk/pycub/blob/master/icub_pybullet/examples/skin_test.py) contains an example with balls falling the robot and skin 
  should turn green on the places where contact occurs. You may want to slow the simulation a little bit to see that :)

## Information
- documentation can be found at [https://lukasrustler.cz/pycub_documentation](https://lukasrustler.cz/pycub_documentation) or in [pycub.pdf](https://lukasrustler.cz/pycub_documentation/pycub.pdf)
- presentation with description of functionality can be found at [pycub presentation](https://lukasrustler.cz/pycub_documentation/pycub_presentation.pdf)
- simulator code is in [pycub.py](https://github.com/rustlluk/pycub/blob/master/icub_pybullet/pycub.py)
  - it uses PyBullet for simulation and provides high-level interface
- visualization code in [visualizer.py](https://github.com/rustlluk/pycub/blob/master/icub_pybullet/visualizer.py)
  - it uses Open3D for visualization as it is much more customizable than PyBullet default GUI

## FAQ

1. You get some kind of error with the visualization, e.g., segmentation fault. 

   1. Try to check graphics mesa/opengl/nvidia drivers as those are the most common source of problems for the openGL visualization
   2. Try Native Docker
   3. In given config your load set in GUI standard=False; web=True to enable web visualization
   4. Try VNC Docker
   5. Try Gitpod Docker

2. You get import errors, e.g. cannot load pycub from icub_pybullet
   1. Install from pip with `python3 -m pip install icub_pybullet`
   2. Install the package correctly with `python3 -m pip install .` from root directory of this repository
   3. put icub_pybullet directory to your PYTHONPATH

## Docker
### Native Version
  - version with native GUI; useful when you have problems with OpenGL (e.g., usually some driver issues)
  - only for GNU/Linux systems (Ubuntu, Mint, Arch, etc.)
1. install [docker-engine](https://docs.docker.com/engine/install/ubuntu/)
    (**DO NOT INSTALL DOCKER DESKTOP**)
    - **perform** [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
2. Build/Pull the Docker Image
    - clone this repository
        ```
        cd SOME_PATH
        git clone https://github.com/rustlluk/pyCub.git
       ```
    - pull the docker image (see [Parameters](#deploy-parameters) for more parameters)  
        ```
        cd SPATH_TO_THE_REPOSITORY/Docker
        ./deploy.py -p PATH_TO_THE_REPOSITORY -c pycub -pu
        ```
    - or, build the docker (see [Parameters](#deploy-parameters) for more parameters)  
        ```
        cd SOME_PATH/pycub_ws/Docker
        ./deploy.py -p PATH_TO_THE_REPOSITORY -c pycub -b 
        ```
    - after you pull or build the container, you can run it next time as 
        ```
        ./deploy.py -c pycub -e
        ```
    - if you want to open new terminal in existing container, run
        ```
        ./deploy.py -c pycub -t
        ```

### VNC Version
  - this version works also on Windows and MacOS because it uses VNC server to show the GUI, i.e., the output will be 
    shown on [http://localhost:6080](http://localhost:6080)
1. Install [docker-engine](https://docs.docker.com/engine/install/ubuntu/) (GNU/Linux only) or 
   [docker-desktop](https://docs.docker.com/desktop/) (all systems)
   - **perform** [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
2. The same as for [Native Version](#native-version), but use  `-vnc` option, e.g., to pull and run the image
     ```
         cd PATH_TO_THE_REPOSITORY/Docker
        ./deploy.py -p PATH_TO_THE_REPOSITORY -c pycub -pu -vnc
     ```
3. run `start-vnc-sessions.sh` script in the container
4. Open [http://localhost:6080](http://localhost:6080)

### Docker + PyCharm
#### Native Version:
You have two option:
1. Either run pycharm from docker
2. Open your pycharm on your host machine:
   - add ssh interpreter
     - user docker
     - ip can be localhost or ip where you run the docker
     - port 2222
   - uncheck automatic upload to remote folder
   - change remote path to /home/docker/pycub_ws  
#### VNC/Gitpod version
1. open pycharm from inside the container

### Deploy Parameters
  - `cd` to folder with Dockerfile
  - `./deploy.py`
    - `-b` or `--build` when building
      - default: False
    - `-e` if you just want to run existing docker without building
      - default: False
    - `-p` or `--path` with path to current folder
      - default: ""
    - `-pu` or `--pull` to pull the image from dockerhub
      - default: False
    - `-c` or `--container` with desired name of the new, created container
      - default: my_new_docker
    - `-t` or `--terminal` to run new terminal in running docker session
      - default: False
    - `-pv` or `--python-version` to specify addition python version to install
      - default: 3.11
    - `-pcv` or `--pycharm-version` to specify version of pycharm to use
      - default: 2023.2.3
    - `-bi` or `--base-image` to specify base image that will be used
      - default: ubuntu:20.04
      - other can be found at hub.docker.com
    
  Do this on computer where you will run the code. If you have a server
  you have to run it on the server over SSH to make things work
  properly.

### Docker FAQ
  - **you get error of not being in sudo group when running image**
    - check output of `id -u` command. If the output is not 1000 you have to build the image
      by yourself and can not pull it
      - this happens when your account is not the first one created on your computer
  - **`sudo apt install something` does not work**
    - you need to run `sudo apt update` first after you run the container for the first time
      - apt things are removed in Dockerfile, so it does not take unnecessary space in the image

## Known bugs
- visualization with skin dies after ~65k steps
  - e.g., [https://github.com/isl-org/Open3D/issues/4992](https://github.com/isl-org/Open3D/issues/4992) 

## License

[![CC BY 4.0][cc-by-shield]][cc-by]

This work is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png
[cc-by-shield]: https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg