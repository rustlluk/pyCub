from setuptools import setup
from pathlib import Path
import os
import sysconfig

app_name = "icub_pybullet"
folders_to_copy = ["iCub", "other_meshes", "icub_pybullet"]
files_to_copy = []
sub_path = {}

# Get the correct site-packages path dynamically
site_packages_path = sysconfig.get_path("purelib").split("/lib/")[1]

# Collect non-Python files
for folder in folders_to_copy:
    for path in Path(folder).rglob('*'):
        if path.is_file() and path.suffix != ".pyc":
            folder_path = os.path.join("lib", site_packages_path, os.path.normpath(path.parent))
            file_path = os.path.normpath(path)
            sub_path.setdefault(folder_path, []).append(file_path)

files_to_copy.extend(sub_path.items())

with open("requirements.txt") as f:
    install_requires = f.read().splitlines()

with open("README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name=app_name,
    version="1.0.2",
    description="pyCub - iCub in PyBullet",
    package_dir={"": "."},
    data_files=files_to_copy,
    install_requires=install_requires,
    author="Lukas Rustler",
    author_email="lukas.rustler@fel.cvut.cz",
    url="https://www.lukasrustler.cz/pycub",
    license="Creative Commons Attribution 4.0 International (CC BY 4.0)",
    license_files="LICENSE",  # Include the LICENSE file in the distribution
    classifiers=[
        # While there is no official classifier for CC BY 4.0 in the PyPI list,
        # you can include a custom classifier or use a more generic one.
        "License :: Creative Commons Attribution 4.0 International (CC BY 4.0)",
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    long_description=long_description,  # Full description from README.md
    long_description_content_type="text/markdown",  # Specify Markdown format
    platforms=["any"],
    python_requires=">=3.8, <=3.11"
)