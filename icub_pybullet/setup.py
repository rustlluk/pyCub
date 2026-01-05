from setuptools import setup
import urllib.request

app_name = "icub_pybullet"

with open("requirements.txt") as f:
    install_requires = f.read().splitlines()

# URL of the README.md file (for example, from GitHub or any other location)
url = "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/README.md"

# Fetch the README content from the URL
try:
    with urllib.request.urlopen(url) as response:
        long_description = response.read().decode('utf-8')
except:
    long_description = ""

setup(
    name=app_name,
    version="1.2.5",
    description="pyCub - iCub in PyBullet",
    package_dir={"icub_pybullet": "."},
    package_data={app_name: ["iCub/**/**", "other_meshes/**/**", "configs/**/**", "logs/**/**"]},
    include_package_data=True,
    install_requires=install_requires,
    author="Lukas Rustler",
    author_email="lukas.rustler@fel.cvut.cz",
    url="https://www.lukasrustler.cz/pycub",
    license="Creative Commons Attribution 4.0 International (CC BY 4.0)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    platforms=["any"],
    python_requires=">=3.10, <3.13"
)