#!/usr/bin/env bash
# Build Linux standalone artifacts for pyCub Studio:
#   1. PyInstaller onedir bundle
#   2. Portable pyCub-<version>-Linux-<arch>.AppImage
#   3. Debian/Ubuntu package pycub-studio_<version>_<deb_arch>.deb
#
# By default, builds inside an isolated virtual environment at
# packaging/work/build_venv (without inheriting system-venv or host packages).
# Pass PYTHON_BIN=/path/to/python to use a specific pre-configured interpreter,
# or REBUILD_VENV=1 to recreate packaging/work/build_venv from scratch.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUT_DIR="${REPO_ROOT}/packaging/out"
DEFAULT_VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/studio/VERSION" 2>/dev/null || echo 1.4.0)"
VERSION="${PYCUB_VERSION:-${DEFAULT_VERSION}}"
PYTHON_BIN="${PYTHON_BIN:-}"

HOST_ARCH="$(uname -m)"
case "${HOST_ARCH}" in
  x86_64|amd64)
    DEB_ARCH="amd64"
    APPIMAGE_ARCH="x86_64"
    OUT_ARCH="x86_64"
    ;;
  aarch64|arm64)
    DEB_ARCH="arm64"
    APPIMAGE_ARCH="aarch64"
    OUT_ARCH="arm64"
    ;;
  *)
    echo "ERROR: Unsupported Linux architecture: ${HOST_ARCH}" >&2
    exit 1
    ;;
esac

pick_base_python() {
  local cand
  for cand in /usr/bin/python3 python3.12 python3.11 python3.10 python3; do
    if command -v "${cand}" >/dev/null 2>&1; then
      if "${cand}" -c 'import sys; raise SystemExit(0 if (3, 10) <= sys.version_info[:2] < (3, 13) else 1)' 2>/dev/null; then
        command -v "${cand}"
        return 0
      fi
    fi
  done
  return 1
}

if [[ -z "${PYTHON_BIN}" ]]; then
  unset PYTHONPATH PYTHONHOME || true
  BUILD_VENV="${REPO_ROOT}/packaging/work/build_venv"
  REQ_STAMP="${BUILD_VENV}/.deps_stamp"
  REQ_HASH="$(cat "${REPO_ROOT}/icub_pybullet/requirements.txt" "${REPO_ROOT}/studio/requirements.txt" | sha256sum | awk '{print $1}')"

  if [[ "${REBUILD_VENV:-0}" == "1" ]]; then
    rm -rf "${BUILD_VENV}"
  fi

  if [[ ! -x "${BUILD_VENV}/bin/python" ]]; then
    BASE_PYTHON="$(pick_base_python)" || {
      echo "ERROR: Could not find Python 3.10-3.12 to create isolated build venv." >&2
      exit 1
    }
    echo "==> [0/3] Creating isolated build venv with ${BASE_PYTHON} at ${BUILD_VENV}..."
    mkdir -p "$(dirname "${BUILD_VENV}")"
    "${BASE_PYTHON}" -m venv "${BUILD_VENV}"
  fi

  if [[ ! -f "${REQ_STAMP}" ]] || [[ "$(cat "${REQ_STAMP}")" != "${REQ_HASH}" ]]; then
    echo "==> [0/3] Installing isolated build dependencies..."
    "${BUILD_VENV}/bin/python" -m pip install --upgrade pip setuptools wheel
    "${BUILD_VENV}/bin/python" -m pip install "${REPO_ROOT}/icub_pybullet"
    "${BUILD_VENV}/bin/python" -m pip install -r "${REPO_ROOT}/studio/requirements.txt"
    "${BUILD_VENV}/bin/python" -m pip install "pyinstaller>=6.10"
    printf '%s\n' "${REQ_HASH}" > "${REQ_STAMP}"
  fi

  PYTHON_BIN="${BUILD_VENV}/bin/python"
fi

WORK_BASE="${RUNNER_TEMP:-${REPO_ROOT}/packaging/work}/pycub_build_$$"
DIST_DIR="${WORK_BASE}/dist"
BUILD_DIR="${WORK_BASE}/build"

cleanup() {
  rm -rf "${WORK_BASE}"
}
trap cleanup EXIT

mkdir -p "${OUT_DIR}" "${DIST_DIR}" "${BUILD_DIR}"

echo "==> [1/3] Running PyInstaller (${PYTHON_BIN}, ${HOST_ARCH})..."
"${PYTHON_BIN}" -m PyInstaller \
  --noconfirm \
  --clean \
  --distpath "${DIST_DIR}" \
  --workpath "${BUILD_DIR}" \
  "${REPO_ROOT}/packaging/pycub.spec"

BUNDLE_DIR="${DIST_DIR}/pyCub"

echo "==> [2/3] Building Debian/Ubuntu .deb package (${DEB_ARCH})..."
DEB_ROOT="${WORK_BASE}/deb_pkg/pycub-studio_${VERSION}_${DEB_ARCH}"
mkdir -p "${DEB_ROOT}/DEBIAN" \
         "${DEB_ROOT}/opt/pycub" \
         "${DEB_ROOT}/usr/bin" \
         "${DEB_ROOT}/usr/share/applications" \
         "${DEB_ROOT}/usr/share/icons/hicolor/512x512/apps"

cp -a "${BUNDLE_DIR}/." "${DEB_ROOT}/opt/pycub/"
ln -sf /opt/pycub/pyCub "${DEB_ROOT}/usr/bin/pycub"
cp "${REPO_ROOT}/studio/assets/icons/pycub.png" \
   "${DEB_ROOT}/usr/share/icons/hicolor/512x512/apps/pycub.png"

cat > "${DEB_ROOT}/usr/share/applications/pycub.desktop" <<'DESKTOP'
[Desktop Entry]
Type=Application
Name=pyCub Studio
Comment=iCub Humanoid Robot Simulator (Kid, Scratch, Python & Classroom Modes)
Exec=/opt/pycub/pyCub
Icon=pycub
Terminal=false
Categories=Education;Science;Robotics;Development;
DESKTOP

cat > "${DEB_ROOT}/DEBIAN/control" <<CONTROL
Package: pycub-studio
Version: ${VERSION}
Section: education
Priority: optional
Architecture: ${DEB_ARCH}
Maintainer: Lukas Rustler <lukas.rustler@fel.cvut.cz>
Description: Standalone iCub Humanoid Simulator with Kid, Scratch, Python IDE, and Classroom Web Modes
CONTROL

if command -v dpkg-deb >/dev/null 2>&1; then
  dpkg-deb --build --root-owner-group "${DEB_ROOT}" "${OUT_DIR}/pycub-studio_${VERSION}_${DEB_ARCH}.deb"
  echo "    Built: ${OUT_DIR}/pycub-studio_${VERSION}_${DEB_ARCH}.deb"
fi

echo "==> [3/3] Building portable .AppImage (${OUT_ARCH})..."
APPDIR="${WORK_BASE}/pyCub.AppDir"
mkdir -p "${APPDIR}/usr/bin"
cp -a "${BUNDLE_DIR}/." "${APPDIR}/usr/bin/"
cp "${REPO_ROOT}/studio/assets/icons/pycub.png" "${APPDIR}/pycub.png"
ln -sf pycub.png "${APPDIR}/.DirIcon"

cat > "${APPDIR}/pycub.desktop" <<'APPDESK'
[Desktop Entry]
Type=Application
Name=pyCub Studio
Exec=pyCub
Icon=pycub
Categories=Education;Science;
Terminal=false
APPDESK

cat > "${APPDIR}/AppRun" <<'APPRUN'
#!/usr/bin/env bash
HERE="$(dirname "$(readlink -f "${0}")")"
export QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}"
exec "${HERE}/usr/bin/pyCub" "$@"
APPRUN
chmod +x "${APPDIR}/AppRun"

APPIMAGETOOL="${APPIMAGETOOL:-${WORK_BASE}/appimagetool}"
if ! command -v "${APPIMAGETOOL}" >/dev/null 2>&1 && [[ ! -x "${APPIMAGETOOL}" ]]; then
  curl -fsSL "https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-${APPIMAGE_ARCH}.AppImage" \
    -o "${APPIMAGETOOL}"
  chmod +x "${APPIMAGETOOL}"
fi

ARCH="${APPIMAGE_ARCH}" APPIMAGE_EXTRACT_AND_RUN=1 "${APPIMAGETOOL}" "${APPDIR}" "${OUT_DIR}/pyCub-${VERSION}-Linux-${OUT_ARCH}.AppImage"
chmod +x "${OUT_DIR}/pyCub-${VERSION}-Linux-${OUT_ARCH}.AppImage"
echo "==> Done! Artifacts written to ${OUT_DIR}:"
ls -lh "${OUT_DIR}"
