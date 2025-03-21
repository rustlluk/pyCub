from setuptools import setup
from pathlib import Path

app_name = "icub_pybullet"
folders_to_copy = ["iCub", "other_meshes", "icub_pybullet"]
files_to_copy = []
sub_path = {}

# Collect non-Python files
for folder in folders_to_copy:
    for path in Path(folder).rglob('*'):
        if path.is_file() and not path.suffix in {".py", ".pyc"}:
            folder_path = str(path.parent)  # Convert Path to string
            file_path = str(path)
            sub_path.setdefault(folder_path, []).append(file_path)

files_to_copy.extend(sub_path.items())

with open("requirements.txt") as f:
    install_requires = f.read().splitlines()

setup(
    name=app_name,
    version="1.0.0",
    description="pyCub - iCub in PyBullet",
    package_dir={"": "."},
    data_files=files_to_copy,
    install_requires=install_requires,
    include_package_data=True,
)