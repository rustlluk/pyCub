#!/usr/bin/env bash
# Build macOS .app bundle + .dmg disk image (supports both arm64 Apple Silicon and x86_64 Intel Macs)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/studio/VERSION" 2>/dev/null || echo 1.4.0)"
VERSION="${PYCUB_VERSION:-${DEFAULT_VERSION}}"
ARCH_NAME="$(uname -m)"
WORK_DIR="${REPO_ROOT}/packaging/work"
DIST_DIR="${WORK_DIR}/dist"
BUILD_DIR="${WORK_DIR}/build"
OUT_DIR="${REPO_ROOT}/packaging/out"
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "${DIST_DIR}" "${BUILD_DIR}" "${OUT_DIR}"

echo "==> [1/3] Running PyInstaller for macOS (${ARCH_NAME})..."
"${PYTHON_BIN}" -m PyInstaller \
  --noconfirm \
  --clean \
  --distpath "${DIST_DIR}" \
  --workpath "${BUILD_DIR}" \
  "${REPO_ROOT}/packaging/pycub.spec"

APP_BUNDLE="${DIST_DIR}/pyCub.app"
ENTITLEMENTS_PLIST="${SCRIPT_DIR}/entitlements.plist"

echo "==> [2/3] Ad-hoc signing ${APP_BUNDLE} (inside-out for QtWebEngine)..."
# Sign nested QtWebEngineProcess.app with JIT / executable-memory entitlements BEFORE sealing
# QtWebEngineCore.framework and pyCub.app (codesign --deep strips nested helper entitlements,
# causing QtWebEngineProcess to fail and render QWebEngineView black).
while IFS= read -r -d '' helper_app; do
  helper_exe="${helper_app}/Contents/MacOS/QtWebEngineProcess"
  if [[ -f "${helper_exe}" ]]; then
    codesign --force --sign - --entitlements "${ENTITLEMENTS_PLIST}" "${helper_exe}"
  fi
  codesign --force --sign - --entitlements "${ENTITLEMENTS_PLIST}" "${helper_app}"
done < <(find "${APP_BUNDLE}/Contents/Frameworks" -type d -name "QtWebEngineProcess.app" -print0 2>/dev/null || true)

while IFS= read -r -d '' we_fw; do
  codesign --force --sign - "${we_fw}"
done < <(find "${APP_BUNDLE}/Contents/Frameworks" -type d -name "QtWebEngineCore.framework" -print0 2>/dev/null || true)

if [[ -f "${APP_BUNDLE}/Contents/MacOS/pyCub" ]]; then
  codesign --force --sign - --entitlements "${ENTITLEMENTS_PLIST}" "${APP_BUNDLE}/Contents/MacOS/pyCub"
fi
codesign --force --sign - --entitlements "${ENTITLEMENTS_PLIST}" "${APP_BUNDLE}"
codesign --verify --deep --strict --verbose=2 "${APP_BUNDLE}"

echo "==> [3/3] Creating DMG installer..."
DMG_STAGING="${WORK_DIR}/dmg_staging"
rm -rf "${DMG_STAGING}"
mkdir -p "${DMG_STAGING}"
ditto "${APP_BUNDLE}" "${DMG_STAGING}/pyCub.app"
ln -s /Applications "${DMG_STAGING}/Applications"

DMG_OUT="${OUT_DIR}/pyCub-${VERSION}-macOS-${ARCH_NAME}.dmg"
DMG_TMP="${WORK_DIR}/pyCub-${VERSION}-macOS-${ARCH_NAME}-$$.dmg"
rm -f "${DMG_OUT}" "${DMG_TMP}"
trap 'rm -f "${DMG_TMP}"' EXIT
create_dmg() {
  local use_sudo="$1"
  shift
  if [[ "${use_sudo}" == "yes" ]]; then
    sudo hdiutil create "$@"
  else
    hdiutil create "$@"
  fi
}

for attempt in 1 2 3; do
  echo "    hdiutil attempt ${attempt}/3..."
  USE_SUDO="no"
  if [[ "${attempt}" -gt 1 ]]; then
    USE_SUDO="yes"
  fi
  if create_dmg "${USE_SUDO}" \
    -volname "pyCub Studio" \
    -srcfolder "${DMG_STAGING}" \
    -ov \
    -format UDZO \
    "${DMG_TMP}"; then
    if [[ "${USE_SUDO}" == "yes" ]]; then
      sudo chown "$(id -u):$(id -g)" "${DMG_TMP}"
    fi
    mv "${DMG_TMP}" "${DMG_OUT}"
    break
  fi

  rm -f "${DMG_TMP}"
  if [[ "${attempt}" -eq 1 ]]; then
    echo "    Retrying hdiutil with elevated privileges..."
    continue
  fi
  sleep 5
done

if [[ ! -s "${DMG_OUT}" ]]; then
  echo "ERROR: DMG creation failed after 3 attempts." >&2
  hdiutil info || true
  exit 1
fi

echo "==> Built ${DMG_OUT}:"
ls -lh "${OUT_DIR}"
