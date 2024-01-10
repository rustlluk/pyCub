#!/usr/bin/env python3

"""
Script to deploy Docker container with easier setup than normal or with docker-compose
"""

import argparse
from subprocess import call, PIPE, run
import create_xauth


def parse():
    """
    The main function of the file, which parses the input arguments and do things accordingly
    """
    parser = argparse.ArgumentParser(description='Script to deploy Docker')

    parser.add_argument(
        "--build",
        "-b",
        dest="build",
        action="store_true",
        required=False,
        default=False,
        help="whether to build the image; default False"
    )

    parser.add_argument(
        "--nvidia",
        "-nv",
        dest="nvidia",
        action="store_true",
        required=False,
        default=False,
        help="whether to use nvidia support; default False"
    )

    parser.add_argument(
        "--existing",
        "-e",
        dest="existing",
        action="store_true",
        required=False,
        default=False,
        help="whether to use existing (already created) image; default False"
    )

    parser.add_argument(
        "--terminal",
        "-t",
        dest="terminal",
        action="store_true",
        required=False,
        default=False,
        help="whether to open new terminal; default False"
    )

    parser.add_argument(
        "--path",
        "-p",
        dest="path",
        required=False,
        default="",
        help="which path to link from host computer; default ''"
    )

    parser.add_argument(
        "--base-image",
        "-bi",
        dest="base_image",
        required=False,
        default="nvidia/cuda:11.0.3-devel-ubuntu20.04",
        help="which image use as base; default 'nvidia/cuda:11.0.3-devel-ubuntu20.04'"
    )

    parser.add_argument(
        "--container",
        "-c",
        dest="container",
        required=False,
        default="my_new_docker",
        help="name of the container; default 'my_new_docker'"
    )

    parser.add_argument(
        "--python-ver",
        "-pv",
        dest="python_ver",
        required=False,
        default="3.11",
        help="second python version to be used; default 3.11"
    )

    parser.add_argument(
        "--pycharm-ver",
        "-pcv",
        dest="pycharm_ver",
        required=False,
        default="2023.2.3",
        help="pycharm version to be used; default 2023.2.3"
    )

    args = parser.parse_args()
    return args.build, args.nvidia, args.existing, args.path, args.container, args.python_ver, args.pycharm_ver, \
           args.terminal, args.base_image


def main():
    # get parameters
    build, nvidia, existing, path, container, python_ver, pycharm_ver, terminal, base_image = parse()
    # set image name with "_image" suffix for easier
    image = container+"_image"

    # if we want to only open new terminal in opened container -> use 'docker run'
    if terminal:
        cmd = "docker exec -it "+container+" /bin/bash"
        call(cmd, shell=True)
        return 0

    # If we want to build the image
    if build:
        # Stop and remove existing container -> this is just to get rid of Docker errors when trying to build ond open
        # a new one
        cmd = "docker stop " + container + " && docker rm "+container
        call(cmd, shell=True)

        # build with correct arguments and plain prorgess
        print("Building")
        cmd = "docker build -t "+image+" --build-arg UID=$(id -u) --build-arg GID=$(id -g)" \
              " --build-arg PYTHON_VER="+python_ver+" --build-arg PYCHARM_VER="+pycharm_ver + \
              " --build-arg BASE_IMAGE="+base_image+" --progress=plain ."
        call(cmd, shell=True)

        image_info = run("docker image ls | grep "+image, shell=True, stdout=PIPE).stdout.decode("utf-8")
        if "second" not in image_info:
            inp = input(f"{image} was either not built properly or already built before. "
                        f"Do you still want to try to run it? ")
            if inp.lower() not in ["y", "yes"]:
                return 0

    # create correct xauth file for usage with graphics and over SSH
    print("Creating xauth")
    create_xauth.main()

    if existing:
        # just run existing docker and return
        print("Starting previous container")
        cmd = "docker start "+container+" && docker attach "+container
        call(cmd, shell=True)
        return 0

    # remove the old container
    # this is needed because we want to use the same name for the new container
    print("Removing old container")
    cmd = "docker stop " + container + " && docker rm " + container
    call(cmd, shell=True, stderr=PIPE, stdout=PIPE)

    # command to run a new container with all necessary arguments
    cmd = '''docker run -it -u $(id -u):$(id -g) -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -e "XAUTHORITY=/tmp/.docker.xauth" \
             -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /dev:/dev \
             -v /etc/hosts:/etc/hosts --network host --privileged --name '''+container+''' \
             -v '''+path+':/home/docker/pycub_ws '+image

    # add nvidia runtime if needed
    if nvidia:
        cmd = cmd.replace(image, '--runtime=nvidia '+image)

    # start it
    print("Starting the container")
    call(cmd, shell=True)

    return 0


if __name__ == "__main__":
    main()
