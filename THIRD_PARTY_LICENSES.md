# Licensing & Third-Party Notices

## Primary Project License (CC BY 4.0)

All original source code, documentation, course exercises, scripts, and project-created meshes (including `icub_pybullet/other_meshes/`, `icub_pybullet/*.py`, `studio/*.py`, `studio/assets/blockly/pycub_blocks.js`, `studio/assets/blockly/index.html`, `exercises/`, and `docs/source*/`) created by the pyCub authors are licensed under the **Creative Commons Attribution 4.0 International License (CC BY 4.0)**. See [`LICENSE`](LICENSE).

Third-party components, models, and bundled runtime libraries remain under their own respective licenses as detailed below.

---

## 1. Vendored Assets & Code in This Repository

### iCub Robot Models & Skin Data (`icub_pybullet/iCub/`)
- **Upstream:** [`robotology/icub-models`](https://github.com/robotology/icub-models) (version `1.25.0`, see `icub_pybullet/iCub/package.xml`)
- **Scope:** `icub_pybullet/iCub/full.urdf`, `icub_pybullet/iCub/simple.urdf`, `icub_pybullet/iCub/meshes/visual/`, derived collision meshes in `icub_pybullet/iCub/meshes/vhacd/`, and skin taxel calibration/point-cloud files in `icub_pybullet/iCub/skin/`
- **License:** **Creative Commons Attribution-ShareAlike 4.0 International (`CC-BY-SA-4.0`)**
- **License Text:** [`icub_pybullet/iCub/LICENSE`](icub_pybullet/iCub/LICENSE) and [`licenses/CC-BY-SA-4.0.txt`](licenses/CC-BY-SA-4.0.txt)

### Blockly Visual Programming Library (`studio/assets/blockly/`)
- **Upstream:** Google / Raspberry Pi Foundation Blockly (version `10.4.3`, [`RaspberryPiFoundation/blockly`](https://github.com/RaspberryPiFoundation/blockly))
- **Scope:** `blockly_compressed.js`, `blocks_compressed.js`, `python_compressed.js`, `msg/en.js`, `msg/cs.js`, and `media/`
- **License:** **Apache License, Version 2.0 (`Apache-2.0`)**
- **License Text:** [`studio/assets/blockly/LICENSE`](studio/assets/blockly/LICENSE) and [`licenses/Apache-2.0.txt`](licenses/Apache-2.0.txt)
- *(Note: `studio/assets/blockly/pycub_blocks.js` and `studio/assets/blockly/index.html` are original pyCub files under CC BY 4.0.)*

### Bundled UI & Emoji Fonts (`studio/assets/fonts/`)
- **Noto Color Emoji (`NotoColorEmoji.ttf`):** Copyright 2013–2017 Google Inc., Arjen Nienhuis — **SIL Open Font License 1.1 (`OFL-1.1`)** ([`licenses/OFL-1.1.txt`](licenses/OFL-1.1.txt))
- **DejaVu Sans & DejaVu Sans Mono (`DejaVuSans.ttf`, `DejaVuSans-Bold.ttf`, `DejaVuSansMono.ttf`):** Copyright (c) 2003 Bitstream, Inc., with public-domain DejaVu additions — **Bitstream Vera / DejaVu Fonts License** ([`licenses/DejaVu-Fonts.txt`](licenses/DejaVu-Fonts.txt))

### Documentation & Website Assets (`docs/` and `studio/website/`)
- **Bulma, `bulma-carousel`, `bulma-slider` (`docs/static/`):** **MIT License**
- **Font Awesome Free (`docs/static/`):** Icons **CC BY 4.0**, Fonts **SIL OFL 1.1**, Code **MIT License**
- **Sphinx Read the Docs Theme & jQuery (`docs/handbook/`):** **MIT License**
- **Academic Project Page Template (`docs/index.html`):** Derived from [Academic Project Page Template](https://github.com/eliahuhorwitz/Academic-project-page-template) / [Nerfies](https://nerfies.github.io) (**CC-BY-SA-4.0**)
- **RicoFast Astro Template (`studio/website/`):** **MIT License** ([`ricocc/ricoui-saas-template`](https://github.com/ricocc/ricoui-saas-template))

---

## 2. Bundled Runtime Dependencies (Standalone `pyCub Studio` Installers)

Standalone desktop packages (`.exe`, `.dmg`, `.AppImage`, `.deb`) built with [`packaging/pycub.spec`](packaging/pycub.spec) distribute compiled binaries of the following third-party libraries under their respective licenses:

### PySide6 / Shiboken6 / Qt 6 / Qt WebEngine
- **License:** **GNU Lesser General Public License v3.0 (`LGPL-3.0-only`)** (dual-licensed by The Qt Company under LGPLv3 / GPLv2 / GPLv3 / Commercial; pyCub Studio uses only LGPLv3-available Qt 6 modules: `QtCore`, `QtGui`, `QtWidgets`, `QtOpenGL`, `QtOpenGLWidgets`, `QtQuick`, `QtQuickWidgets`, `QtWebChannel`, `QtWebEngineCore`, `QtWebEngineWidgets`).
- **License Texts:** [`licenses/LGPL-3.0.txt`](licenses/LGPL-3.0.txt) and [`licenses/GPL-3.0.txt`](licenses/GPL-3.0.txt)
- **Embedded Chromium (`QtWebEngineCore`):** Includes Chromium and its third-party components under permissive licenses (`BSD`, `MIT`, `Apache-2.0`, `MPL-2.0`) and **GNU LGPL v2.1** ([`licenses/LGPL-2.1.txt`](licenses/LGPL-2.1.txt)).
- **Corresponding Source Code & Relinking:**
  - Qt 6 source archives: <https://download.qt.io/official_releases/qt/>
  - PySide6 / Shiboken6 source: <https://code.qt.io/cgit/pyside/pyside-setup.git/>
  - Standalone bundles are packaged in `onedir` layout so Qt/PySide6 shared libraries remain dynamically linked and replaceable, and reverse engineering for debugging modifications to LGPL-licensed libraries is not restricted.

### OpenCV (`opencv-python`)
- **License:** **Apache-2.0** (OpenCV) and **MIT** (`opencv-python` packaging scripts). Prebuilt PyPI wheels bundle third-party binaries including **FFmpeg** (**LGPLv2.1**, [`licenses/LGPL-2.1.txt`](licenses/LGPL-2.1.txt)) and, on non-headless Linux wheels, **Qt 5** (**LGPLv3**, [`licenses/LGPL-3.0.txt`](licenses/LGPL-3.0.txt)).

### PyInstaller Bootloader
- **License:** **GPL-2.0-or-later WITH Bootloader-exception** (used solely to package the standalone executable; the Bootloader Exception permits distributing the resulting bundle under pyCub's license terms).

### Permissive Python Libraries (`pip` & Standalone Bundles)
- **`pybullet`, `pybullet-arm64`:** zlib/libpng License (`Zlib`)
- **`open3d`:** MIT License
- **`roboticstoolbox-python`, `spatialmath-python`, `spatialgeometry`, `xacrodoc`, `rtb-data`, `ansitable`, `pgraph-python`, `colored`:** MIT License (`progress`: ISC; `robot_descriptions` wrapper: Apache-2.0, excluded from standalone PyInstaller bundles)
- **`numpy`, `scipy`, `psutil`, `flask`, `werkzeug`, `PyOpenGL`, `qrcode`:** BSD-3-Clause License
- **`transforms3d`:** BSD-2-Clause License
- **`jedi`, `parso`, `pyyaml`, `platformdirs`, `configargparse`:** MIT License
- **`matplotlib`:** PSF-based Matplotlib License
- **`Pillow`:** HPND (Historical Permission Notice and Disclaimer) License
