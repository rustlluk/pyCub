# README
- this repo contains code for iCub simulation in PyBullet
- it provides also dockerfile for easy setup of the environment

## Known bugs
- visualization with skin dies after ~65k steps
  - e.g., [https://github.com/isl-org/Open3D/issues/4992](https://github.com/isl-org/Open3D/issues/4992) 

## Installation  

1. Install dependencies
   - use Docker  
      - see [Docker](#docker) section
   - or install python3 and `pip3 install pybullet numpy scipy open3d`
2. Pull this repository
3. try run some example, e.g., `python3 PATH_TO_THE_REPOSITORY/icub_pybullet/examples/push_the_ball_cartesian.py`

## Examples
- [push_the_ball_pure_joints.py](icub_pybullet/examples/push_the_ball_pure_joints.py) contains an example that
  shows how to control the robot in joint space
- [push_the_ball_cartesian.py](icub_pybullet/examples/push_the_ball_cartesian.py) contains an example that
  shows how to control the robot in Cartesian space
- [skin_test.py](icub_pybullet/examples/skin_test.py) contains an example with balls falling the robot and skin 
  should turn green on the places where contact occurs. You may want to slow the simulation a little bit to see that :)

## Information
- documentation can be found at [lukasrustler.cz/pycub](https://lukasrustler.cz/pycub) or in [pyCub.pdf](documentation/pyCub.pdf)
- simulator code is in [pycub.py](icub_pybullet/pycub.py)
  - it uses PyBullet for simulation and provides high-level interface
- visualization code in [visualizer.py](icub_pybullet/visualizer.py)
  - it uses Open3D for visualization as it is much more customizable than PyBullet default GUI
- movement is done using position control. You can either use position control directly
  (pycub.move_position()) or use cartesian control (pycub.move_cartesian())
  - **Neither of these check for collision before movement!**
  - Function pycub.motion_done() check whether all joints reached the target or whether collision
    ocurred. If collision, the variable pycub.collision_during_movement is set. You can also run
    pycub.motion_done() with check_collision=False to ignore collision checks, e.g., to get out
    of collision state
- until the repository is installed as a packages, examples contains line `sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))`
  so that python can find the necessary modules. To avoid that, you can run all codes from `icub_pybullet` folder or use PyCharm (or other IDE)

## Docker
[https://github.com/rustlluk/easy-docker](https://github.com/rustlluk/easy-docker) is utilized to use Docker
### Installation  
  - install [docker-engine](https://docs.docker.com/engine/install/ubuntu/)
    (**DO NOT INSTALL DOCKER DESKTOP**), do [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
    and (optional) install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
    for GPU support
  - For ubuntu (and Mint, but you have) users:  

    - if you are a mint user, change VERSION_CODENAME to UBUNTU_CODENAME  
    
    ```
    sudo apt-get update
    sudo apt-get install ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    echo \
      "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```  
    
    - and post-installation to use docker without sudo:
    
    ```
    sudo groupadd docker
    sudo usermod -aG docker $USER
    ```
    - and restart your computer

  - clone this repository
        
        cd SOME_PATH
        git clone https://github.com/rustlluk/pyCub.git
  - (optionally) rename it to be called the same as in docker

        mv SOME_PATH/pycub SOME_PATH/pycub_ws

  - run the docker (see [Parameters](#deploy-parameters) for more parameters)  
   
        cd SOME_PATH/pycub_ws/Docker

    - to build it on your computer:  
    
       ```
       ./deploy.py -b -p SOME_PATH/pycub_ws -c pycub
       ```

### Docker + PyCharm
1. Either run pycharm from docker
2. In your local docker:
   - add ssh interpreter
     - user docker
     - ip can be localhost or ip where you run the docker
     - port 2222
   - uncheck automatic upload to remote folder
   - change remote path to /home/docker/pycub_ws  

Common steps:
   - mark all folder icub_pybullet as Source Root
   - for X11 forwarding:  
     - Click on configurations drop menu -> Edit Configurations -> Edit configuration templates -> 
       Python -> Edit environment variables -> add DISPLAY with the same value as in docker and uncheck 
       'Include system environment variables'. Every new configuration will have that settings from now
       - if you already have configuration created before doing the above -> delete it and create again, or change it manually
    
### Deploy Parameters
  - `cd` to folder with Dockerfile
  - `./deploy.py`
    - `-b` or `--build` when building
      - default: False
    - `-nv` or `--nvidia` when you want to use your Nvidia card
      - you have to use it when creating a new container
      - default: False
    - `-e` if you just want to run existing docker without building
      - default: False
    - `-p` or `--path` with path to current folder
      - default: ""
    - `c` or `--container` with desired name of the new, created container
      - default: my_new_docker
    - `t` or `--terminal` to run new terminal in running docker session
      - default: False
    - `-pv` or `--python-version` to specify addition python version to install
      - default: 3.11
    - `-pcv` or `--pycharm-version` to specify version of pycharm to use
      - default: 2023.2.3
    - `-bi` or `--base-image` to specify base image that will be used
      - default: nvidia/cuda:11.0.3-devel-ubuntu20.04
      - other can be found at hub.docker.com
    
  Do this on computer where you will run the code. If you have a server
  you have to run it on the server over SSH to make things work
  properly.

### FAQ
  - **applications do not run on your screen** (or you have some strange screen
    related errors)
    - in another terminal run `xhost local:docker`
      - if it does not work, try `xhost +`
        - if this does not work, nothing can be done
  - **you get error of not being in sudo group when running image**
    - check output of `id -u` command. If the output is not 1000 you have to build the image
      by yourself and can not pull it
      - this happens when your account is not the first one created on your computer
  - **`sudo apt install something` does not work**
    - you need to run `sudo apt update` first after you run the container for the first time
      - apt things are removed in Dockerfile, so it does not take unnecessary space in the image
