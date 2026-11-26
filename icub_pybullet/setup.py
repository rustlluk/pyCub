"""Setuptools packaging script for the ``icub_pybullet`` Python distribution."""
from setuptools import setup
import os
import urllib.request

app_name = "icub_pybullet"
HERE = os.path.dirname(os.path.abspath(__file__))


def _read_version(path: str, env_var: str, fallback: str = "1.4.0") -> str:
    env_val = os.environ.get(env_var, "").strip()
    if env_val:
        return env_val
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            text = f.read().strip()
            if text:
                return text
    return fallback


icub_version = _read_version(os.path.join(HERE, "VERSION"), "ICUB_VERSION")
studio_min_version = _read_version(os.path.join(HERE, "..", "studio", "VERSION"), "STUDIO_VERSION")

with open(os.path.join(HERE, "requirements.txt"), encoding="utf-8") as f:
    install_requires = [line.strip() for line in f if line.strip() and not line.startswith("#")]

readme_local = os.path.join(HERE, "..", "README.md")
if os.path.exists(readme_local):
    with open(readme_local, encoding="utf-8") as f:
        long_description = f.read()
else:
    url = "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/README.md"
    try:
        with urllib.request.urlopen(url, timeout=5) as response:
            long_description = response.read().decode("utf-8")
    except Exception:
        long_description = ""

setup(
    name=app_name,
    version=icub_version,
    description="pyCub - iCub in PyBullet",
    package_dir={"icub_pybullet": "."},
    package_data={
        app_name: [
            "VERSION",
            "iCub/**/**",
            "other_meshes/**/**",
            "configs/**/**",
            "logs/**/**",
            "examples/**/**",
        ]
    },
    include_package_data=True,
    install_requires=install_requires,
    extras_require={
        "studio": [
            f"pycub-studio>={studio_min_version}",
        ]
    },
    author="Lukas Rustler",
    author_email="lukas.rustler@fel.cvut.cz",
    url="https://www.lukasrustler.cz/pycub",
    license="Creative Commons Attribution 4.0 International (CC BY 4.0)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    platforms=["any"],
    python_requires=">=3.10, <3.13",
)
