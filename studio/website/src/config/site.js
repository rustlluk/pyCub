const SITE_URL = import.meta.env.PUBLIC_SITE_URL || "https://pycub.lukasrustler.cz";

export const releaseAssets = {
  "windows-exe": {
    filename: "pyCub-Setup-1.0.0-Windows-x64.exe",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pyCub-Setup-1.0.0-Windows-x64.exe",
    size: "436 MB",
  },
  "macos-arm64": {
    filename: "pyCub-1.0.0-macOS-arm64.dmg",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pyCub-1.0.0-macOS-arm64.dmg",
    size: "492 MB",
  },
  "macos-x64": {
    filename: "pyCub-1.0.0-macOS-x86_64.dmg",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pyCub-1.0.0-macOS-x86_64.dmg",
    size: "562 MB",
  },
  "linux-x64-appimage": {
    filename: "pyCub-1.0.0-Linux-x86_64.AppImage",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pyCub-1.0.0-Linux-x86_64.AppImage",
    size: "772 MB",
  },
  "linux-arm64-appimage": {
    filename: "pyCub-1.0.0-Linux-arm64.AppImage",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pyCub-1.0.0-Linux-arm64.AppImage",
    size: "427 MB",
  },
  "linux-amd64-deb": {
    filename: "pycub-studio_1.0.0_amd64.deb",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pycub-studio_1.0.0_amd64.deb",
    size: "612 MB",
  },
  "linux-arm64-deb": {
    filename: "pycub-studio_1.0.0_arm64.deb",
    url: "https://github.com/rustlluk/pyCub/releases/latest/download/pycub-studio_1.0.0_arm64.deb",
    size: "387 MB",
  },
};

export const siteConfig = {
  title: "pyCub Studio",
  version: "1.0.0",
  releaseTag: "v2026-11-26",
  author: "Lukáš Rustler & Matěj Hoffmann (CTU in Prague)",
  url: SITE_URL,
  mail: "lukas.rustler@fel.cvut.cz",
  utm: {
    source: `${SITE_URL}`,
    medium: "referral",
    campaign: "navigation",
  },
  meta: {
    title: "pyCub Studio — Humanoid Robotics Simulator & Learning IDE for iCub",
    description:
      "Cross-platform desktop studio for the iCub humanoid robot in PyBullet. Native OpenGL 3.3 viewport, Blockly Scratch, kid_api Python, full IDE, tactile skin, and stereo cameras.",
    keywords:
      "pycub, pycub studio, icub, pybullet, humanoid robotics, blockly, python robotics, ctu prague, simulator",
    image: "/assets/screenshots/normal-ide.png",
    twitterHandle: "",
  },
  links: {
    github: "https://github.com/rustlluk/pyCub",
    releases: "https://github.com/rustlluk/pyCub/releases",
    releasesLatest: "https://github.com/rustlluk/pyCub/releases/latest",
    releasesLatestApi: "https://api.github.com/repos/rustlluk/pyCub/releases/latest",
    pypiStudio: "https://pypi.org/project/pycub-studio/",
    pypiCore: "https://pypi.org/project/icub-pybullet/",
    handbookEn: "https://rustlluk.github.io/pyCub/handbook/index.html",
    handbookCs: "https://rustlluk.github.io/pyCub/handbook/cs/index.html",
    pdfEn: "https://rustlluk.github.io/pyCub/handbook/pycub.pdf",
    pdfCs: "https://rustlluk.github.io/pyCub/handbook/cs/pycub.pdf",
    arxiv: "https://arxiv.org/abs/2506.01756",
    projectSite: "https://rustlluk.github.io/pyCub",
    exercisesZip:
      "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/exercises/exercises.zip",
    license: "https://creativecommons.org/licenses/by/4.0/",
    thirdPartyLicenses: "https://github.com/rustlluk/pyCub/blob/master/THIRD_PARTY_LICENSES.md",
    icubModelsLicense: "https://creativecommons.org/licenses/by-sa/4.0/",
    authorLukas: "https://www.lukasrustler.cz",
    authorMatej: "https://sites.google.com/site/matejhof/",
    department: "https://cyber.felk.cvut.cz/",
  },
  social: {
    twitter: "",
    twitterName: "",
    github: "https://github.com/rustlluk/pyCub",
    blog: "https://rustlluk.github.io/pyCub/handbook/index.html",
  },
};

export const socialLinks = [
  {
    name: "GitHub",
    url: "https://github.com/rustlluk/pyCub",
    icon: `<svg class="icon ic-github ic-social" viewBox="0 0 1024 1024" xmlns="http://www.w3.org/2000/svg" width="20" height="20"><path d="M511.957333 21.333333C241.024 21.333333 21.333333 240.981333 21.333333 512c0 216.832 140.544 400.725333 335.573334 465.664 24.490667 4.394667 32.256-10.069333 32.256-23.082667 0-11.690667 0.256-44.245333 0-85.205333-136.448 29.610667-164.736-64.64-164.736-64.64-22.314667-56.704-54.4-71.765333-54.4-71.765333-44.586667-30.464 3.285333-29.824 3.285333-29.824 49.194667 3.413333 75.178667 50.517333 75.178667 50.517333 43.776 75.008 114.816 53.333333 142.762666 40.789333 4.522667-31.658667 17.152-53.376 31.189334-65.536-108.970667-12.458667-223.488-54.485333-223.488-242.602666 0-53.546667 19.114667-97.322667 50.517333-131.669334-5.034667-12.330667-21.930667-62.293333 4.778667-129.834666 0 0 41.258667-13.184 134.912 50.346666a469.802667 469.802667 0 0 1 122.88-16.554666c41.642667 0.213333 83.626667 5.632 122.88 16.554666 93.653333-63.488 134.784-50.346667 134.784-50.346666 26.752 67.541333 9.898667 117.504 4.864 129.834666 31.402667 34.346667 50.474667 78.122667 50.474666 131.669334 0 188.586667-114.730667 230.016-224.042666 242.090666 17.578667 15.232 33.578667 44.672 33.578666 90.453334v135.850666c0 13.141333 7.936 27.605333 32.853334 22.869334C862.250667 912.597333 1002.666667 728.746667 1002.666667 512 1002.666667 240.981333 783.018667 21.333333 511.957333 21.333333z" fill="currentColor"></path></svg>`,
  },
  {
    name: "arXiv",
    url: "https://arxiv.org/abs/2506.01756",
    icon: `<svg class="icon ic-social" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" width="18" height="18"><path d="M4 19.5v-15A2.5 2.5 0 0 1 6.5 2H20v20H6.5a2.5 2.5 0 0 1 0-5H20"></path></svg>`,
  },
];
