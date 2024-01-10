# based on https://github.com/luiscarlosgph/dockerx/tree/main
import os
import tempfile
import subprocess
import shlex
import pathlib


def run_shell(cmd):
    cmd_list = shlex.split(cmd, posix=False)
    proc = subprocess.Popen(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out, err = proc.communicate()
    return out


def main():
    # Create an empty Xauthority file if it does not exist
    xauth_path = os.path.join(tempfile.gettempdir(), '.docker.xauth')

    if os.path.isdir(xauth_path):
        pathlib.Path(xauth_path).rmdir()

    if not os.path.isfile(xauth_path):
        pathlib.Path(xauth_path).touch()

    # Hack X11 cookie
    out = run_shell('xauth nlist ' + os.environ['DISPLAY'])
    cookie = 'ffff' + out.decode('ascii')[4:]

    # Save X11 cookie in a temporary file
    cookie_path = os.path.join(tempfile.gettempdir(), '.myx11cookie')
    with open(cookie_path, 'w') as f:
        f.write(cookie)

    # Merge cookie in Xauthority file
    run_shell('xauth -f ' + xauth_path + ' nmerge ' + cookie_path)


if __name__ == "__main__":
    main()