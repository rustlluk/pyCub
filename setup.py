import os.path

from setuptools import setup
from pathlib import Path

app_name = "icub_pybullet"
folders_to_copy = ["iCub", "other_meshes", "icub_pybullet"]
files_to_copy = []
sub_path = {}

for folder in folders_to_copy:
    for path in Path(folder).rglob('*'):
        if os.path.isfile(path) and ".py" not in os.path.basename(path):
            if os.path.dirname(path) not in sub_path:
                sub_path[os.path.dirname(path)] = [os.path.normpath(path)]
            else:
                sub_path[os.path.dirname(path)].append(os.path.normpath(path))

for sp in sub_path:
    files_to_copy.append((sp, sub_path[sp]))

setup(
    name=app_name,
    version="0.0.1",
    description="Your application description",
    package_dir={"": "."},
    data_files=files_to_copy
)