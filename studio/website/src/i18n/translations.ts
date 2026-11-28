export type Locale = "en" | "cs";

export type ReleaseAssetKey =
  | "windows-exe"
  | "macos-arm64"
  | "macos-x64"
  | "linux-x64-appimage"
  | "linux-arm64-appimage"
  | "linux-amd64-deb"
  | "linux-arm64-deb";

export interface NavItem {
  name: string;
  url: string;
}

export interface ModeInfo {
  id: string;
  badge: string;
  tabLabel: string;
  title: string;
  subtitle: string;
  description: string;
  cliFlag: string;
  audience: string;
  screenshot: string;
  screenshotDark?: string;
  screenshotAlt: string;
  highlights: string[];
  accentColor: string;
}

export interface CodeSample {
  id: string;
  tabLabel: string;
  title: string;
  filename: string;
  language: "bash" | "python" | "yaml";
  description: string;
  code: string;
}

export interface DownloadTarget {
  os: string;
  arch: string;
  format: string;
  notes: string;
  howTo: string;
  downloads: { label: string; assetKey: ReleaseAssetKey }[];
}

export interface KidCommandRow {
  en: string;
  cs: string;
  desc: string;
}

export interface ExerciseInfo {
  num: number;
  title: string;
  summary: string;
  skills: string;
}

export interface ChangelogMilestone {
  version: string;
  date: string;
  title: string;
  highlights: string[];
}

export const BIBTEX_CITATION = `@inproceedings{rustler2026pycub,
  title={Learning with pyCub: A Simulation and Exercise Framework for Humanoid Robotics},
  author={Lukas Rustler and Matej Hoffmann},
  year={2026},
  booktitle={17th International Conference on Robotics in Education (RiE 2026)},
  organization={Springer}
}`;

export function getRoutePrefix(lang: Locale): string {
  return lang === "cs" ? "/cs" : "";
}

export function switchLangPath(currentPath: string, targetLang: Locale): string {
  const clean = currentPath === "/" ? "/" : currentPath.replace(/\/+$/, "");
  const isCurrentlyCs = clean === "/cs" || clean.startsWith("/cs/");
  if (targetLang === "cs") {
    if (isCurrentlyCs) return clean === "/cs" ? "/cs/" : clean;
    return clean === "/" ? "/cs/" : `/cs${clean}`;
  } else {
    if (!isCurrentlyCs) return clean;
    const stripped = clean.slice(3) || "/";
    return stripped;
  }
}

export interface SiteStrings {
  langLabel: string;
  meta: {
    homeTitle: string;
    homeDesc: string;
    modesTitle: string;
    modesDesc: string;
    codeTitle: string;
    codeDesc: string;
    downloadsTitle: string;
    downloadsDesc: string;
    changelogTitle: string;
    changelogDesc: string;
    aboutTitle: string;
    aboutDesc: string;
  };
  nav: {
    navigationLabel: string;
    items: NavItem[];
    github: string;
    starGithub: string;
    light: string;
    dark: string;
    langSwitchLabel: string;
  };
  banner: {
    badge: string;
    text: string;
    cta: string;
    showcaseTitle: string;
    showcaseSubtitle: string;
    quickLinks: { label: string; url: string; external?: boolean }[];
  };
  beta: {
    badge: string;
    statusLabel: string;
    title: string;
    description: string;
    shortNote: string;
    githubCta: string;
    emailCta: string;
  };
  hero: {
    badges: [string, string, string];
    title: string;
    subtitle: string;
    binariesLabel: string;
    binaryChips: { label: string; ext: string; assetKey: ReleaseAssetKey }[];
    allBuildsChip: string;
    pipAltLabel: string;
    copyCmdLabel: string;
    copiedLabel: string;
    primaryCta: string;
    secondaryCta: string;
    tertiaryCta: string;
    frameTitle: string;
    frameCaption: string;
  };
  strip: {
    label: string;
    items: { name: string; sub: string }[];
  };
  pillars: {
    title: string;
    description: string;
    items: { icon: string; title: string; desc: string }[];
  };
  architectureSplit: {
    title: string;
    description: string;
    cards: {
      tag: string;
      title: string;
      pkg: string;
      desc: string;
      bullets: string[];
      cmd: string;
    }[];
  };
  modesSection: {
    title: string;
    description: string;
    launchLabel: string;
    audienceLabel: string;
    viewAllCta: string;
    modes: ModeInfo[];
  };
  gallerySection: {
    title: string;
    description: string;
    items: {
      title: string;
      caption: string;
      src: string;
      tag: string;
    }[];
  };
  codeSection: {
    title: string;
    description: string;
    copyBtn: string;
    copiedBtn: string;
    exploreApiCta: string;
    samples: CodeSample[];
  };
  techStack: {
    title: string;
    description: string;
    cards: { name: string; role: string; desc: string }[];
  };
  downloadsSummary: {
    title: string;
    description: string;
    latestReleaseBadge: string;
    altDevNote: string;
    pipTitle: string;
    pipDesc: string;
    dockerTitle: string;
    dockerDesc: string;
    handbookTitle: string;
    handbookDesc: string;
    targets: DownloadTarget[];
    ctaReleases: string;
    ctaGuide: string;
  };
  faq: {
    title: string;
    description: string;
    items: { question: string; answer: string }[];
  };
  cite: {
    title: string;
    subtitle: string;
    paperBtn: string;
    copyBibtex: string;
    copiedBibtex: string;
    authorsNote: string;
  };
  finalCta: {
    title: string;
    description: string;
    primary: string;
    secondary: string;
  };
  footer: {
    description: string;
    cols: {
      studio: string;
      developers: string;
      handbook: string;
      project: string;
    };
    links: {
      studio: NavItem[];
      developers: NavItem[];
      handbook: NavItem[];
      project: NavItem[];
    };
    licenseText: string;
    thirdPartyLicenseText: string;
    builtWith: string;
  };
  pages: {
    modes: {
      heading: string;
      subheading: string;
      tags: string[];
      workspaceTitle: string;
      workspaceDesc: string;
      workspaceDirs: { dir: string; desc: string }[];
    };
    code: {
      heading: string;
      subheading: string;
      tags: string[];
      kidApiTitle: string;
      kidApiDesc: string;
      colEn: string;
      colCs: string;
      colDesc: string;
      kidCommands: KidCommandRow[];
      exercisesTitle: string;
      exercisesDesc: string;
      exercises: ExerciseInfo[];
    };
    downloads: {
      heading: string;
      subheading: string;
      tags: string[];
      recommendedBadge: string;
      macosGatekeeperNote: {
        title: string;
        description: string;
      };
      binariesIncludedNote: string;
      exercisesZipLabel: string;
      pipSectionTitle: string;
      pipSectionDesc: string;
      installersTitle: string;
      installersDesc: string;
      dockerTitle: string;
      dockerDesc: string;
      handbookTitle: string;
      handbookDesc: string;
      handbookLinks: { label: string; url: string; badge: string }[];
      buildFromSourceTitle: string;
      buildFromSourceDesc: string;
    };
    changelog: {
      heading: string;
      subheading: string;
      tags: string[];
      milestones: ChangelogMilestone[];
    };
    about: {
      heading: string;
      subheading: string;
      tags: string[];
      storyTitle: string;
      storyParagraphs: string[];
      authorsTitle: string;
      authors: { name: string; role: string; org: string; url: string }[];
      highlightsTitle: string;
      highlights: { title: string; desc: string }[];
      licenseSection: {
        title: string;
        subtitle: string;
        items: { component: string; license: string; details: string; url?: string }[];
        primaryLinkLabel: string;
        thirdPartyLinkLabel: string;
      };
    };
  };
}

export const translations: Record<Locale, SiteStrings> = {
  en: {
    langLabel: "English",
    meta: {
      homeTitle: "pyCub Studio — Humanoid Robotics Simulator & IDE for iCub",
      homeDesc:
        "Standalone desktop PyBullet simulator and 5-mode learning studio for the iCub humanoid robot. Pre-built binaries for Windows, macOS & Linux — plus pip install.",
      modesTitle: "Studio Modes — pyCub Studio",
      modesDesc:
        "Explore all 5 integrated modes in pyCub Studio: Kid GUI, Scratch Blockly, Kid Programming, Normal Full IDE, and Classroom Web Selector.",
      codeTitle: "Code Samples & API — pyCub Studio",
      codeDesc:
        "Bilingual EN/CZ kid_api commands, Cartesian Pose + IK, tactile skin sensors, custom YAML worlds, and PyBullet course exercises.",
      downloadsTitle: "Download Binaries & Install — pyCub Studio",
      downloadsDesc:
        "Download latest release binaries for Windows (.exe), macOS (.dmg), and Linux (.AppImage / .deb), or install via pip & Docker.",
      changelogTitle: "Changelog & Milestones — pyCub Studio",
      changelogDesc:
        "Release history and major milestones for pyCub Studio, icub_pybullet, Blockly integration, and standalone installers.",
      aboutTitle: "About & Citation — pyCub Studio",
      aboutDesc:
        "Developed in the Humanoid and Cognitive Robotics Group, Department of Cybernetics, FEE CTU in Prague by Lukáš Rustler and Matěj Hoffmann. RiE 2026 paper and CC BY 4.0 license.",
    },
    nav: {
      navigationLabel: "Navigation",
      items: [
        { name: "Home", url: "/" },
        { name: "Modes", url: "/modes" },
        { name: "Code & API", url: "/code" },
        { name: "Download", url: "/downloads" },
        { name: "Changelog", url: "/changelog" },
        { name: "About", url: "/about" },
      ],
      github: "GitHub",
      starGithub: "View on GitHub",
      light: "Light",
      dark: "Dark",
      langSwitchLabel: "Čeština",
    },
    banner: {
      badge: "Standalone Binaries & PyPI",
      text: "Download latest release binaries for Windows (.exe), macOS (.dmg) & Linux (.AppImage / .deb) — zero Python setup required",
      cta: "Download Latest",
      showcaseTitle: "pyCub · Learning Humanoid Robotics with iCub",
      showcaseSubtitle: "Self-contained Desktop Binaries · Dual-theme OpenGL Studio · 5 Progressive Modes · CTU in Prague",
      quickLinks: [
        { label: "Latest Release Assets", url: "https://github.com/rustlluk/pyCub/releases/latest", external: true },
        { label: "Paper (arXiv)", url: "https://arxiv.org/abs/2506.01756", external: true },
        { label: "HTML Handbook", url: "https://rustlluk.github.io/pyCub/handbook/index.html", external: true },
        { label: "PDF Manual", url: "https://rustlluk.github.io/pyCub/handbook/pycub.pdf", external: true },
        { label: "Exercises (.zip)", url: "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/exercises/exercises.zip", external: true },
        { label: "5 Studio Modes", url: "#modes", external: false },
      ],
    },
    beta: {
      badge: "Beta",
      statusLabel: "Beta Release",
      title: "pyCub Studio is currently in beta",
      description:
        "If you encounter any issues, bugs, or installation hiccups, please open an issue on GitHub or contact the maintainer directly via email.",
      shortNote: "pyCub Studio is in beta — encounter a problem? Reach out on",
      githubCta: "Report on GitHub",
      emailCta: "Email Maintainer",
    },
    hero: {
      badges: ["Windows · macOS · Linux Binaries", "No Python Required", "English & Čeština"],
      title: "Learn humanoid robotics with the simulated iCub",
      subtitle:
        "pyCub Studio is a self-contained desktop app uniting a hardware-accelerated OpenGL 3D viewport, five progressive modes from visual Blockly to a full Python IDE, bilingual English/Czech UI, and offline course handbooks — just download and launch.",
      binariesLabel: "Direct download (latest release):",
      binaryChips: [
        { label: "Windows x64", ext: ".exe", assetKey: "windows-exe" },
        { label: "macOS Silicon", ext: ".dmg", assetKey: "macos-arm64" },
        { label: "macOS Intel", ext: ".dmg", assetKey: "macos-x64" },
        { label: "Linux x64", ext: ".AppImage", assetKey: "linux-x64-appimage" },
      ],
      allBuildsChip: "All 7 builds (.deb / arm64) →",
      pipAltLabel: "Or install via pip (Python 3.10–3.12):",
      copyCmdLabel: "pip install pycub-studio",
      copiedLabel: "Copied!",
      primaryCta: "Download Desktop App",
      secondaryCta: "Explore 5 Modes",
      tertiaryCta: "Code & API",
      frameTitle: "pyCub Studio — Normal Mode (IDE + OpenGL + Eye Cameras)",
      frameCaption: "Real capture of pyCub Studio running push_the_ball_cartesian.py with l_eye RGB & r_eye Depth cameras.",
    },
    strip: {
      label: "Pre-built standalone binaries · Plus pip & Docker",
      items: [
        { name: "Windows x64", sub: "Setup .exe installer" },
        { name: "macOS Apple Silicon", sub: "Native arm64 .dmg" },
        { name: "macOS Intel", sub: "Native x86_64 .dmg" },
        { name: "Linux x64 & arm64", sub: "Portable .AppImage & .deb" },
        { name: "PyPI & Docker", sub: "pip install pycub-studio" },
      ],
    },
    pillars: {
      title: "Built for classrooms, labs, and first-time coders",
      description:
        "Most robotics simulators require complex ROS setups or Python environment wrangling. pyCub Studio ships as ready-to-run desktop binaries and scales from first-time exploration to advanced robotics coursework.",
      items: [
        {
          icon: "/assets/icon/components.png",
          title: "Five Progressive Modes",
          desc: "Start with clickable preset buttons, advance to bilingual Blockly blocks or simplified kid_api Python, and graduate to a full multi-tab IDE with F12 Go-to-Definition.",
        },
        {
          icon: "/assets/icon/fast.png",
          title: "Self-Contained Desktop Binaries",
          desc: "Every .exe, .dmg, .AppImage, and .deb bundles Python, PyBullet, PySide6, OpenGL 3.3, Blockly, and the offline handbook — double-click to launch with zero configuration.",
        },
        {
          icon: "/assets/icon/customize.png",
          title: "Full Czech & English Localization",
          desc: "Switch between English and Čeština live across the launcher, menus, Blockly blocks, Python helper names (zamávej, dotkl_se_míčku), and bundled Sphinx handbooks.",
        },
      ],
    },
    architectureSplit: {
      title: "Download the standalone Studio — or install via Python",
      description:
        "Teachers and students can run the packaged pyCub Studio binary right away without Python, while developers and researchers can install pycub-studio or icub_pybullet via pip.",
      cards: [
        {
          tag: "Desktop IDE & Standalone App",
          title: "pyCub Studio",
          pkg: "Binaries + pycub-studio",
          desc: "Complete cross-platform desktop application bundling all 5 modes, offline Blockly, an isolated script runner, built-in PyPI package installer, course exercises 1–5, and EN/CS handbooks.",
          bullets: [
            "Zero system Python needed — standalone .exe, .dmg, .AppImage & .deb",
            "Writable ~/Documents/pyCub/ workspace for custom worlds & meshes",
            "Also installable with one command via pip install pycub-studio",
          ],
          cmd: "Download Latest Binaries (Windows · macOS · Linux)",
        },
        {
          tag: "Lightweight Robotics Core",
          title: "icub_pybullet Core",
          pkg: "pip install icub_pybullet",
          desc: "For Python developers and headless servers: pure PyBullet simulator package for joint, velocity, and Cartesian IK control, tactile artificial skin, stereo cameras, and kid_api.",
          bullets: [
            "No GUI/Qt bloat — ideal for headless runs & automated benchmarks",
            "YAML world configs (default.yaml, with_ball.yaml, skin_test.yaml)",
            "Dual-language kid_api functions synced with Studio runtime",
          ],
          cmd: "pip install icub_pybullet",
        },
      ],
    },
    modesSection: {
      title: "Five integrated modes in one window",
      description:
        "Click any mode below to inspect actual captures from pyCub Studio — tailored for every skill level from tactile buttons to full robotics IDE.",
      launchLabel: "Direct CLI flag (optional)",
      audienceLabel: "Designed for",
      viewAllCta: "All Modes & Workspace Details",
      modes: [
        {
          id: "kid_native",
          badge: "Mode 1",
          tabLabel: "Kid GUI (Buttons)",
          title: "Kid GUI — Native Tactile Playground",
          subtitle: "Zero typing required — drive iCub with preset actions and single-joint jog controls.",
          description:
            "Designed for newcomers, science fairs, and first interactions. Features large pose presets (Wave, Both Arms Up, Look Center), per-joint step buttons, step-size & speed sliders, custom .yaml world selection, live ball distance telemetry, and confetti celebration when the ball is hit.",
          cliFlag: "pycub-studio --mode kid_native",
          audience: "First-time visitors, demos & outreach events",
          screenshot: "/assets/screenshots/kid-native.png",
          screenshotAlt: "pyCub Studio Mode 1 Kid GUI Native showing control buttons and 3D OpenGL iCub with red ball",
          highlights: [
            "One-click preset motions (Wave Left/Right, Reset, Stand Straight)",
            "Individual joint sliders & delta step buttons with configurable speed",
            "Live distance-to-ball telemetry & hit counter with confetti reward",
          ],
          accentColor: "emerald",
        },
        {
          id: "kid_scratch",
          badge: "Mode 2",
          tabLabel: "Scratch Blocks",
          title: "Kid GUI \"Scratch\" — Offline Visual Blockly",
          subtitle: "Drag-and-drop visual programming with real-time execution highlighting and live Python generation.",
          description:
            "Powered by an offline Google Blockly workspace with full English & Czech block translations. As the program runs, each active block lights up live alongside the 3D robot, while a side inspector previews generated Python code and lets students graduate to Kid Programming in one click.",
          cliFlag: "pycub-studio --mode kid_scratch --lang en",
          audience: "Beginners & visual programming learners",
          screenshot: "/assets/screenshots/kid-scratch.png",
          screenshotAlt: "pyCub Studio Mode 2 Scratch visual Blockly blocks beside 3D iCub simulator",
          highlights: [
            "100% offline Google Blockly with EN/CZ bilingual block library",
            "Live yellow highlight synced step-by-step with robot motion",
            "Save/load .pycub_blocks projects and one-click export to Python",
          ],
          accentColor: "amber",
        },
        {
          id: "kid_code",
          badge: "Mode 3",
          tabLabel: "Kid Programming",
          title: "Kid Programming — High-Level Python API",
          subtitle: "Bridge visual blocks and real Python using intuitive kid_api commands without radians or simulation loops.",
          description:
            "Students write clean, readable Python using icub_pybullet.kid_api functions in degrees and seconds. Includes a clickable Command Palette that inserts snippets, inline call signatures, starter templates, and friendly localized error hints.",
          cliFlag: "pycub-studio --mode kid_code",
          audience: "Beginner & intermediate Python learners",
          screenshot: "/assets/screenshots/kid-code.png",
          screenshotAlt: "pyCub Studio Mode 3 Kid Programming showing Command Palette, Python editor, and 3D iCub",
          highlights: [
            "Intuitive degrees/seconds API (wave('right'), shoulder_up, ball_touched())",
            "Bilingual English & Czech function names (zamávej, zvedni_rameno)",
            "Clickable Command Palette, starter templates & friendly traceback hints",
          ],
          accentColor: "violet",
        },
        {
          id: "normal",
          badge: "Mode 4",
          tabLabel: "Full IDE",
          title: "Normal Mode — Full pyCub IDE & Script Runner",
          subtitle: "Multi-tab Python & YAML development environment with stereo eye cameras, coordinate axes, and Exercise Tester.",
          description:
            "Built for advanced learners, course instructors, and robotics researchers. Offers multi-tab Python/YAML editing with autocomplete, hover docstrings, F12 Go to Definition, external --run-script runner with auto-run on save, dockable l_eye / r_eye RGB & Depth feeds, 3D coordinate frame overlays, and one-click unpacking + grading for Course Exercises 1–5.",
          cliFlag: "pycub-studio --mode normal",
          audience: "Robotics courses & researchers",
          screenshot: "/assets/screenshots/normal-ide.png",
          screenshotDark: "/assets/screenshots/normal-ide-dark.png",
          screenshotAlt: "pyCub Studio Mode 4 Normal IDE running Cartesian ball push with l_eye RGB, r_eye Depth, and console",
          highlights: [
            "Dockable l_eye / r_eye RGB & Depth cameras + 3D Coordinate Axes toggle",
            "Multi-tab editor with autocomplete, Go to Definition (F12) & YAML loader",
            "Built-in Course Exercises (1–5) installer, tester & package manager",
          ],
          accentColor: "blue",
        },
        {
          id: "web",
          badge: "Mode 5",
          tabLabel: "Classroom Web",
          title: "Classroom Web Selector — Multi-Instance Server",
          subtitle: "Launch and supervise N independent browser simulators behind one load-balanced QR landing page.",
          description:
            "Run workshops or classroom labs from a single host machine. Spin up any number of isolated browser kid_ui instances, share the LAN URL or scannable Wi-Fi QR code with students on tablets or Chromebooks, monitor live user counts, restart individual workers, or inspect logs.",
          cliFlag: "pycub-studio --mode web",
          audience: "Teachers, workshops & computer labs",
          screenshot: "/assets/screenshots/web-selector.png",
          screenshotAlt: "pyCub Studio Mode 5 Classroom Web Selector showing 2 running instances, LAN URL, and QR code",
          highlights: [
            "Spawn N isolated browser simulator instances behind kid_ui_landing.py",
            "Instant LAN URL + scannable Wi-Fi QR code for student devices",
            "Live per-instance health, user count, restart button & log viewer",
          ],
          accentColor: "cyan",
        },
      ],
    },
    gallerySection: {
      title: "Inside the Studio — Light, Dark & Bilingual UI",
      description:
        "Every desktop view adapts cleanly between Light and Dark themes and switches live between English and Čeština.",
      items: [
        {
          title: "Home Launcher (Light Theme)",
          caption: "Mode cards, live EN/CZ language switcher, Dark/Light theme, and quick Handbook access.",
          src: "/assets/screenshots/home-light.png",
          tag: "Launcher · Light",
        },
        {
          title: "Home Launcher (Dark Theme)",
          caption: "High-contrast slate workspace matching coding environments and lab projectors.",
          src: "/assets/screenshots/home-dark.png",
          tag: "Launcher · Dark",
        },
        {
          title: "Czech Kid GUI (Dětské ovládání)",
          caption: "Full Czech labels across buttons, joints, ball telemetry, and status bar.",
          src: "/assets/screenshots/kid-native-cs.png",
          tag: "Čeština · Mode 1",
        },
        {
          title: "Czech Scratch Blocks (Bloky Blockly)",
          caption: "Localized Czech visual blocks generating Python right beside the 3D viewport.",
          src: "/assets/screenshots/kid-scratch-cs.png",
          tag: "Čeština · Mode 2",
        },
      ],
    },
    codeSection: {
      title: "From first wave to Cartesian IK & tactile skin",
      description:
        "Run these scripts directly inside the standalone pyCub Studio app (Mode 3 or Mode 4) — or from your Python environment via pip.",
      copyBtn: "Copy",
      copiedBtn: "Copied",
      exploreApiCta: "Full Code & Command Reference",
      samples: [
        {
          id: "kid_en",
          tabLabel: "kid_api (EN)",
          title: "Beginner Python inside Studio Mode 3 (or standalone)",
          filename: "kid_wave_and_push.py",
          language: "python",
          description: "Zero radians, zero manual stepping loops — works immediately inside the bundled pyCub Studio binary.",
          code: `from icub_pybullet.kid_api import *

# Greet and inspect the scene
wave("right")
look_down(15)
print(f"Ball distance: {ball_distance():.1f} cm")

# Lower the right shoulder until the hand reaches the ball
while not ball_touched():
    shoulder_down("right", degrees=10, speed=1.8)
    wait(0.2)

print(f"Hit count: {ball_hits()}")
reset_pose()`,
        },
        {
          id: "kid_cs",
          tabLabel: "kid_api (CZ)",
          title: "Stejné API plně česky — aliases v icub_pybullet.kid_api",
          filename: "detsky_program_cz.py",
          language: "python",
          description: "Students can write clean Czech commands directly inside Mode 3 without installing Python separately.",
          code: `from icub_pybullet.kid_api import *

# Česká verze příkazů pro Dětské programování
zamávej("pravá")
podívej_se_dolů(15)

for _ in range(4):
    if dotkl_se_míčku():
        break
    spusť_rameno("pravá", 12)
    počkej(0.2)

print("Počet zásahů:", počet_zásahů())
obnov_polohu()`,
        },
        {
          id: "cartesian",
          tabLabel: "Cartesian IK",
          title: "Push the ball in Cartesian space (Pose + IK)",
          filename: "push_the_ball_cartesian.py",
          language: "python",
          description: "Runs out of the box in Studio Mode 4 using the bundled Python runtime.",
          code: `from icub_pybullet.pycub import pyCub, Pose

client = pyCub(config="with_ball.yaml")

# Read current end-effector Pose (3D position + quaternion orientation)
cur_pose = client.end_effector.get_position()

# Drop 15 cm lower and move 10 cm closer to torso
new_pose = Pose(cur_pose.pos, cur_pose.ori)
new_pose.pos[2] -= 0.15
new_pose.pos[0] -= 0.10
client.move_cartesian(new_pose)

# Sweep hand laterally (wait=False lets you drive the loop manually)
pose = client.end_effector.get_position()
pose.pos[1] -= 0.20
client.move_cartesian(pose, wait=False, velocity=2)

while not client.motion_done():
    client.update_simulation()`,
        },
        {
          id: "skin",
          tabLabel: "Tactile Skin & YAML",
          title: "Tactile skin simulation & custom world YAML",
          filename: "custom_world.yaml",
          language: "yaml",
          description: "Drop a custom world in ~/Documents/pyCub/worlds/ — picked up automatically by the Studio binary.",
          code: `# Save in ~/Documents/pyCub/worlds/skin_demo.yaml
parent: skin_test.yaml

end_effector: "r_hand"
skin:
  use: True
  radius: 0.015      # ray-cast distance from iCub skin links (meters)
  num_cores: 0       # 0 = use all available CPU cores

eyes:
  l_eye: True        # dockable RGB view in Studio Normal Mode
  r_eye: True        # dockable Depth view in Studio Normal Mode`,
        },
        {
          id: "cli",
          tabLabel: "Launch Flags / pip",
          title: "Launch a binary or install via pip",
          filename: "launch_studio.sh",
          language: "bash",
          description: "Works with both downloaded OS binaries and pip-installed pycub-studio (Python 3.10–3.12).",
          code: `# Standalone Linux AppImage (no Python install needed):
chmod +x pyCub-*-Linux-x86_64.AppImage
./pyCub-*-Linux-x86_64.AppImage --mode kid_scratch --lang cs

# Windows / macOS binaries accept the exact same flags:
# pyCub.exe --mode normal --run-script my_controller.py

# Or install via pip (Python 3.10–3.12):
python3 -m pip install pycub-studio
pycub-studio`,
        },
      ],
    },
    techStack: {
      title: "Engineered with proven open-source foundations",
      description: "Reliable physics, hardware-accelerated Qt rendering, and offline visual blocks — bundled in every binary with zero cloud dependency.",
      cards: [
        { name: "PyBullet", role: "Physics & IK Engine", desc: "Rigid-body dynamics, self-collisions, VHACD meshes, and inverse kinematics." },
        { name: "PySide6 (Qt 6)", role: "Desktop Shell", desc: "Native menus, multi-tab IDE, docked camera panels, and light/dark palettes." },
        { name: "OpenGL 3.3 / 2.1", role: "3D Viewport", desc: "Embedded hardware-accelerated robot, skin, and coordinate frame rendering." },
        { name: "Google Blockly", role: "Visual Programming", desc: "100% offline EN/CS block editor synced with live Python generation." },
        { name: "Sphinx + LaTeX", role: "Offline Handbook", desc: "Bilingual EN/CS HTML & PDF manual bundled right inside the app." },
      ],
    },
    downloadsSummary: {
      title: "Download Latest Release Binaries — No Python Required",
      description:
        "Direct 1-click downloads always pointing to the latest GitHub Release. Everything is bundled inside — just download and run.",
      latestReleaseBadge: "Always synced to Latest Release",
      altDevNote: "Prefer Python packages or containers?",
      pipTitle: "PyPI Installation (Python 3.10–3.12)",
      pipDesc: "For developers who prefer pip over standalone binaries:",
      dockerTitle: "Docker Native & VNC",
      dockerDesc: "Skip host driver setup with Ubuntu containers or browser-accessible noVNC (port 6080).",
      handbookTitle: "Offline & Online Handbooks",
      handbookDesc: "Includes Mode guides, API reference, Exercises 1–5, and downloadable PDFs in EN & CS.",
      targets: [
        {
          os: "Windows",
          arch: "x64 (64-bit)",
          format: "Setup .exe",
          notes: "Self-contained installer with Start Menu shortcut — no Python needed.",
          howTo: "Run the .exe installer and launch pyCub Studio from Desktop or Start Menu.",
          downloads: [{ label: "Setup .exe (x64)", assetKey: "windows-exe" }],
        },
        {
          os: "macOS",
          arch: "arm64 (Apple Silicon M1–M4)",
          format: ".dmg",
          notes: "Native Apple Silicon bundle with bundled runtime and offline handbook.",
          howTo: "Drag pyCub.app to Applications. First launch: allow in System Settings -> Privacy & Security.",
          downloads: [{ label: "Apple Silicon .dmg", assetKey: "macos-arm64" }],
        },
        {
          os: "macOS",
          arch: "x86_64 (Intel Mac)",
          format: ".dmg",
          notes: "Universal Intel Mac bundle including pinned Open3D 0.19.0 support.",
          howTo: "Drag pyCub.app to Applications. First launch: allow in System Settings -> Privacy & Security.",
          downloads: [{ label: "Intel Mac .dmg", assetKey: "macos-x64" }],
        },
        {
          os: "Linux",
          arch: "x86_64 & arm64",
          format: ".AppImage & .deb",
          notes: "Portable single-file AppImage or Debian/Ubuntu .deb package.",
          howTo: "chmod +x *.AppImage && ./pyCub-*.AppImage (or sudo apt install ./*.deb).",
          downloads: [
            { label: "x86_64 .AppImage", assetKey: "linux-x64-appimage" },
            { label: "arm64 .AppImage", assetKey: "linux-arm64-appimage" },
            { label: "amd64 .deb", assetKey: "linux-amd64-deb" },
            { label: "arm64 .deb", assetKey: "linux-arm64-deb" },
          ],
        },
      ],
      ctaReleases: "All Options: pip, Handbooks & Docker",
      ctaGuide: "Browse GitHub Releases",
    },
    faq: {
      title: "Frequently asked questions",
      description: "Answers about standalone binaries, pip installation, macOS/Linux tips, and classroom deployment.",
      items: [
        {
          question: "Do I need Python, pip, or PyBullet installed on my computer?",
          answer:
            "No! The standalone Windows (.exe), macOS (.dmg), and Linux (.AppImage / .deb) binaries bundle a complete Python runtime, pybullet, numpy, scipy, PySide6, OpenGL, Blockly, and the offline handbook inside one package. Just download and double-click.",
        },
        {
          question: "Can I still install extra Python libraries (like OpenCV or scikit-learn) in the standalone binary?",
          answer:
            "Yes. Inside pyCub Studio Mode 4, click Packages / Python in the top toolbar to install any PyPI package directly into your ~/Documents/pyCub/extra_packages/ folder — no system Python needed.",
        },
        {
          question: "How do I open pyCub.app on macOS if Gatekeeper shows an unidentified developer warning?",
          answer:
            "Because the open-source .dmg builds use ad-hoc signing, macOS Gatekeeper blocks the first launch. Try opening pyCub.app, then go to System Settings -> Privacy & Security and click \"Open Anyway\" (on older macOS you can Right-Click -> Open), or run: xattr -cr /Applications/pyCub.app in Terminal.",
        },
        {
          question: "What is the difference between the pyCub Studio binary and the pip packages?",
          answer:
            "Standalone binaries are double-click desktop apps needing zero Python setup. For Python developers, pip install pycub-studio installs the exact same 5-mode Studio into your virtual environment, while pip install icub_pybullet installs just the lightweight headless/scriptable simulator.",
        },
        {
          question: "Can I add my own objects, URDFs, or custom simulation scenarios?",
          answer:
            "Yes. Place custom .yaml world files (which can inherit from default.yaml or with_ball.yaml via parent:) into ~/Documents/pyCub/worlds/, and custom .obj or .urdf models into ~/Documents/pyCub/meshes/. Any .obj without a *_vhacd.obj automatically runs VHACD convex decomposition and caches it.",
        },
        {
          question: "How can I teach a whole classroom without installing anything on student laptops?",
          answer:
            "Launch Mode 5 (Classroom Web Selector) from your host binary, set the desired number of worker instances, and display the built-in Wi-Fi QR code on the projector. Students scan it on tablets, laptops, or Chromebooks to control their own iCub instance in the browser.",
        },
      ],
    },
    cite: {
      title: "Academic Paper & Citation",
      subtitle: "Presented at the 17th International Conference on Robotics in Education (RiE 2026) · Springer",
      paperBtn: "Read Paper on arXiv",
      copyBibtex: "Copy BibTeX",
      copiedBibtex: "BibTeX Copied!",
      authorsNote: "Humanoid and Cognitive Robotics Group, Department of Cybernetics, Faculty of Electrical Engineering, Czech Technical University in Prague",
    },
    finalCta: {
      title: "Start simulating iCub in minutes",
      description:
        "Download the latest standalone binary for Windows, macOS, or Linux, run pip install pycub-studio, or browse the bilingual EN/CZ handbook.",
      primary: "Download Binaries",
      secondary: "Read Online Handbook",
    },
    footer: {
      description:
        "Self-contained PyBullet humanoid robotics simulator and 5-mode learning Studio for the iCub robot, built at CTU in Prague.",
      cols: {
        studio: "Studio",
        developers: "Developers",
        handbook: "Handbook & Courses",
        project: "Project",
      },
      links: {
        studio: [
          { name: "Home", url: "/" },
          { name: "Download Latest Binaries", url: "/downloads" },
          { name: "All 5 Modes", url: "/modes" },
          { name: "Changelog", url: "/changelog" },
        ],
        developers: [
          { name: "Code Samples & kid_api", url: "/code" },
          { name: "GitHub Releases (Latest)", url: "https://github.com/rustlluk/pyCub/releases/latest" },
          { name: "PyPI: pycub-studio", url: "https://pypi.org/project/pycub-studio/" },
          { name: "PyPI: icub_pybullet", url: "https://pypi.org/project/icub-pybullet/" },
        ],
        handbook: [
          { name: "HTML Handbook", url: "https://rustlluk.github.io/pyCub/handbook/index.html" },
          { name: "PDF Manual", url: "https://rustlluk.github.io/pyCub/handbook/pycub.pdf" },
        ],
        project: [
          { name: "About & CTU Team", url: "/about" },
          { name: "GitHub Repository", url: "https://github.com/rustlluk/pyCub" },
          { name: "Report Issue (GitHub)", url: "https://github.com/rustlluk/pyCub/issues" },
          { name: "arXiv Paper (2506.01756)", url: "https://arxiv.org/abs/2506.01756" },
          { name: "Course Exercises (.zip)", url: "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/exercises/exercises.zip" },
        ],
      },
      licenseText: "Original code & materials licensed under CC BY 4.0.",
      thirdPartyLicenseText: "Third-party assets & runtime notices",
      builtWith: "pyCub Studio · CTU Prague",
    },
    pages: {
      modes: {
        heading: "Five Modes Built for Every Stage",
        subheading:
          "From a zero-code tactile playground to a full multi-tab robotics IDE and browser-based classroom server — every mode shares the same live physics engine.",
        tags: ["Mode 1: Kid GUI", "Mode 2: Blockly Scratch", "Mode 3: Kid Python", "Mode 4: Full IDE", "Mode 5: Web Classroom"],
        workspaceTitle: "User Workspace (~/Documents/pyCub/)",
        workspaceDesc:
          "On first launch, pyCub Studio initializes a writable workspace in ~/Documents/pyCub/ so students and teachers can extend the simulator without modifying app bundles:",
        workspaceDirs: [
          { dir: "worlds/", desc: "Custom .yaml configs automatically listed in the world picker. Inherit built-in worlds via parent: with_ball.yaml." },
          { dir: "meshes/", desc: "Drop custom .obj or .urdf models here. Missing *_vhacd.obj files automatically generate and cache convex hulls." },
          { dir: "extra_packages/", desc: "Auto-added to sys.path — install extra PyPI packages directly from Studio without needing system Python." },
          { dir: "scripts/ & projects/", desc: "Your Python scripts (.py) and Scratch visual projects (.pycub_blocks)." },
          { dir: "exercises/ & examples/", desc: "Unpacked Course Exercises 1–5 with automated testers and reference example controllers." },
          { dir: "images/", desc: "Exported l_eye / r_eye RGB & Depth camera snapshots from Normal Mode." },
        ],
      },
      code: {
        heading: "Code Samples, API & Exercises",
        subheading:
          "Copy-ready Python snippets for beginners (kid_api in English & Czech) through advanced Cartesian IK, tactile skin sensors, and custom YAML scenes.",
        tags: ["kid_api (EN & CZ)", "Cartesian IK", "Tactile Skin", "Custom YAML", "Exercises 1–5"],
        kidApiTitle: "kid_api Bilingual Command Reference",
        kidApiDesc:
          "Every high-level function works identically inside the standalone pyCub Studio binary (Mode 3 & Mode 2) as well as standalone Python scripts. Czech aliases map directly to the exact same functions:",
        colEn: "English Function",
        colCs: "Czech Alias (Čeština)",
        colDesc: "Action / Behavior",
        kidCommands: [
          { en: 'wave("right") / left_wave()', cs: 'zamávej("pravá") / zamávej_levou()', desc: "Perform a friendly wave gesture with left or right arm" },
          { en: "both_arms_up() / both_arms_down()", cs: "zvedni_obě_ruce() / spusť_obě_ruce()", desc: "Raise or lower both arms to preset poses" },
          { en: 'shoulder_up("left", 15)', cs: 'zvedni_rameno("levá", 15)', desc: "Lift shoulder relative to current angle (degrees)" },
          { en: 'shoulder_down("right", 15)', cs: 'spusť_rameno("pravá", 15)', desc: "Lower shoulder relative to current angle (degrees)" },
          { en: 'bend_elbow("right", 20)', cs: 'ohni_loket("pravá", 20)', desc: "Flex the elbow by the given number of degrees" },
          { en: "look_up(15) / look_down(15) / turn_head(20)", cs: "podívej_se_nahoru(15) / podívej_se_dolů(15)", desc: "Pitch or pan the iCub head in degrees" },
          { en: "bend_forward(10) / turn_body(15)", cs: "předkloň_se(10) / otoč_trup(15)", desc: "Move the torso pitch or yaw joint" },
          { en: "wait(0.5) / reset_pose() / stop()", cs: "počkej(0.5) / obnov_polohu() / zastav()", desc: "Pause cleanly while stepping physics, reset, or halt joints" },
          { en: "ball_touched() / ball_distance() / ball_hits()", cs: "dotkl_se_míčku() / vzdálenost_míčku() / počet_zásahů()", desc: "Read live ball telemetry (cm, hit boolean, counter)" },
          { en: "skin_touched()", cs: "dotyk_kůže()", desc: "Check whether any enabled tactile skin taxel is active" },
        ],
        exercisesTitle: "Course Exercises (1–5)",
        exercisesDesc:
          "Progressive assignments ready to unpack inside Studio Mode 4, complete with README instructions and an automated Exercise Tester:",
        exercises: [
          { num: 1, title: "Push the Ball", summary: "Get acquainted with pyCub simulation loops and use position or Cartesian control to hit the ball as far off the table as possible without table collisions.", skills: "Joint & Cartesian Motion" },
          { num: 2, title: "Smooth Movements", summary: "Plan continuous, jerk-free Cartesian trajectories to move the robot's end-effector along straight lines or circles across specified axes.", skills: "Cartesian Trajectories" },
          { num: 3, title: "Gaze Control", summary: "Implement a non-blocking neck controller that tracks a ball moving in 2D on the table by aligning head gaze and head-to-ball direction vectors.", skills: "Neck Control & Tracking" },
          { num: 4, title: "Resolved-Rate Motion Control", summary: "Use Jacobian-based velocity control (RRMC) and artificial skin taxel clusters to move single or multiple limbs away from falling balls against the contact normal.", skills: "RRMC & Tactile Skin" },
          { num: 5, title: "Grasp It!", summary: "Detect the green ball in the eye camera's RGB image, deproject its 2D center into 3D using the depth buffer, and grasp and lift it above the table.", skills: "Vision, Depth & Grasping" },
        ],
      },
      downloads: {
        heading: "Download Binaries & Installation",
        subheading:
          "Download the latest standalone binary directly for Windows, macOS, or Linux (no Python needed) — or install via pip or Docker.",
        tags: ["Windows .exe", "macOS .dmg", "Linux .AppImage / .deb", "Latest Release", "pip & Docker"],
        recommendedBadge: "Recommended · No Python Needed",
        installersTitle: "1. Standalone Desktop Binaries (Latest Release)",
        installersDesc:
          "Each download link resolves automatically to the latest GitHub Release asset and bundles Python, PyBullet, OpenGL, Blockly, and the offline EN/CS handbook:",
        macosGatekeeperNote: {
          title: "macOS (.dmg): Allow app in System Settings",
          description:
            "Because the open-source .dmg builds are ad-hoc signed, macOS Gatekeeper blocks the first launch. After dragging pyCub.app to Applications and trying to open it, go to System Settings -> Privacy & Security, scroll to Security, and click \"Open Anyway\" (or run: xattr -cr /Applications/pyCub.app in Terminal).",
        },
        binariesIncludedNote:
          "Even without system Python, Mode 4 includes a built-in Packages / Python dialog to install extra PyPI packages into ~/Documents/pyCub/extra_packages/, or point to an external virtual environment later.",
        handbookTitle: "2. Bilingual Handbooks (HTML, PDF & Course Exercises)",
        handbookDesc: "Offline handbooks are bundled inside every binary; online links and standalone exercise packs are below:",
        exercisesZipLabel: "Download Course Exercises (exercises.zip)",
        handbookLinks: [
          { label: "English HTML Handbook", url: "https://rustlluk.github.io/pyCub/handbook/index.html", badge: "HTML · EN" },
          { label: "Česká HTML příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/index.html", badge: "HTML · CS" },
          { label: "English PDF Manual", url: "https://rustlluk.github.io/pyCub/handbook/pycub.pdf", badge: "PDF · EN" },
          { label: "Česká PDF příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/pycub.pdf", badge: "PDF · CS" },
        ],
        pipSectionTitle: "3. Install via pip (Python 3.10–3.12)",
        pipSectionDesc: "Works from any terminal and installs either full pycub-studio or the lightweight icub_pybullet core package:",
        dockerTitle: "4. Docker (Native GUI & Cross-Platform VNC)",
        dockerDesc: "Use Docker/deploy.py to launch either native X11/OpenGL on GNU/Linux or browser noVNC (localhost:6080) on Windows/macOS/Linux.",
        buildFromSourceTitle: "5. Building Binaries from Source",
        buildFromSourceDesc: "Package your own .AppImage, .deb, .exe, or .dmg locally using the scripts under packaging/:",
      },
      changelog: {
        heading: "Releases & Milestones",
        subheading: "Key milestones in the evolution of pyCub and pyCub Studio.",
        tags: ["pyCub Studio", "Standalone Binaries", "Bilingual EN/CS"],
        milestones: [
          {
            version: "Studio Release",
            date: "2026",
            title: "pyCub Studio — Standalone OS Binaries, 5-Mode Desktop App & PyPI",
            highlights: [
              "Cross-platform CI producing standalone Windows Setup .exe, macOS arm64/x86_64 .dmg, and Linux .AppImage/.deb binaries",
              "Embedded Qt + OpenGL 3.3 / 2.1 viewport inside Modes 1–4 with coordinate axes and l_eye / r_eye RGB & Depth docks",
              "Offline Google Blockly workspace (Mode 2) with live active-block highlighting and Python export",
              "Beginner kid_api with full English and Czech command aliases (zamávej, dotkl_se_míčku, etc.)",
              "Mode 5 Classroom Web Selector with load-balanced landing page and Wi-Fi QR code",
              "Clean split into pycub-studio desktop distribution and standalone icub_pybullet core package",
            ],
          },
          {
            version: "v1.1.1",
            date: "2026",
            title: "OS Compatibility, Bilingual Handbook & Exercises 1–5",
            highlights: [
              "Added full Czech localization of documentation and PDF handbook builds via docs/build.py",
              "Bundled five progressive course exercises with starter templates and automated verification scripts",
              "Published arXiv preprint (2506.01756) and RiE 2026 conference submission",
            ],
          },
          {
            version: "v1.0.0",
            date: "2025",
            title: "icub_pybullet Simulation & Tactile Skin Core",
            highlights: [
              "PyBullet simulation of full iCub URDF with VHACD convex collision meshes",
              "Joint, velocity, and Cartesian Pose + IK controllers with YAML scene configuration",
              "Multi-core artificial skin ray-casting simulation and stereo eye camera buffers",
            ],
          },
        ],
      },
      about: {
        heading: "About pyCub & CTU Prague Team",
        subheading: "An open-source educational robotics framework built in the Humanoid and Cognitive Robotics Group, Department of Cybernetics, Faculty of Electrical Engineering, Czech Technical University in Prague.",
        tags: ["CTU Prague FEE", "RiE 2026", "CC BY 4.0"],
        storyTitle: "Why pyCub Studio",
        storyParagraphs: [
          "Teaching humanoid robotics often hits a tooling wall: real hardware like the iCub robot is rare and fragile, while research simulators frequently demand Linux-only ROS middleware and fragile setup steps before a student moves a single joint.",
          "pyCub was created at the Czech Technical University in Prague to bridge that gap. Its core package, icub_pybullet, wraps PyBullet with accurate iCub kinematics, artificial skin ray-casting, stereo cameras, and reproducible YAML scenes.",
          "pyCub Studio packages everything into self-contained binaries for Windows, macOS, and Linux (as well as pip packages) — letting beginners snap Blockly blocks in Czech or English, learners experiment with kid_api, and advanced classes debug Cartesian controllers with live RGB/Depth eye feeds.",
        ],
        authorsTitle: "Authors & Institution",
        authors: [
          {
            name: "Lukáš Rustler",
            role: "Lead Author & Maintainer",
            org: "Humanoid and Cognitive Robotics Group, Department of Cybernetics, Faculty of Electrical Engineering, Czech Technical University in Prague",
            url: "https://www.lukasrustler.cz",
          },
          {
            name: "Matěj Hoffmann",
            role: "Supervisor & Co-Author",
            org: "Humanoid and Cognitive Robotics Group, Department of Cybernetics, Faculty of Electrical Engineering, Czech Technical University in Prague",
            url: "https://sites.google.com/site/matejhof/",
          },
        ],
        highlightsTitle: "Core Principles",
        highlights: [
          { title: "Accessible Progression", desc: "One consistent 3D robot from clickable buttons and Blockly up to low-level IK and sensor feedback." },
          { title: "Zero-Friction Binaries & pip", desc: "Ready-to-run .exe, .dmg, .AppImage, and .deb packages plus pip wheels and browser QR classroom mode." },
          { title: "Bilingual by Design", desc: "Full English & Czech experience across GUI, visual blocks, Python kid_api aliases, and handbooks." },
          { title: "Open Educational Resource", desc: "Original code, handbook, and exercises are released under CC BY 4.0, while bundled robot models and runtime libraries retain their own open licenses." },
        ],
        licenseSection: {
          title: "Licensing & Third-Party Notices",
          subtitle:
            "Original pyCub code, documentation, course exercises, and project-created meshes are released under Creative Commons Attribution 4.0 International (CC BY 4.0). Bundled third-party assets and standalone runtime dependencies remain under their respective open-source licenses:",
          items: [
            {
              component: "Original pyCub, icub_pybullet & pyCub Studio",
              license: "CC BY 4.0",
              details: "All original Python/JS source code, docs/handbook sources, Course Exercises 1–5, and project-created meshes (icub_pybullet/other_meshes/).",
              url: "https://creativecommons.org/licenses/by/4.0/",
            },
            {
              component: "iCub Robot Models & Skin Data (icub_pybullet/iCub/)",
              license: "CC-BY-SA-4.0",
              details: "URDFs, visual meshes, derived VHACD collision meshes, and skin taxel data from robotology/icub-models (v1.25.0).",
              url: "https://creativecommons.org/licenses/by-sa/4.0/",
            },
            {
              component: "Offline Blockly Visual Editor (studio/assets/blockly/)",
              license: "Apache-2.0",
              details: "Google / Raspberry Pi Foundation Blockly v10.4.3 bundles, localizations, and media (pycub_blocks.js and index.html remain CC BY 4.0).",
              url: "https://www.apache.org/licenses/LICENSE-2.0",
            },
            {
              component: "Standalone Desktop Runtime (PySide6 / Qt 6, OpenCV)",
              license: "LGPLv3 / LGPLv2.1",
              details: "Dynamically linked PySide6 / Qt 6 / QtWebEngine in onedir bundles (LGPL-3.0, Chromium LGPL-2.1), opencv-python (Apache-2.0 / FFmpeg LGPL-2.1), and permissive Python packages (MIT, BSD, zlib).",
              url: "https://github.com/rustlluk/pyCub/blob/master/THIRD_PARTY_LICENSES.md",
            },
          ],
          primaryLinkLabel: "CC BY 4.0 License",
          thirdPartyLinkLabel: "Full Third-Party Notices (GitHub)",
        },
      },
    },
  },

  cs: {
    langLabel: "Čeština",
    meta: {
      homeTitle: "pyCub Studio — Simulátor humanoidního robota iCub a výukové IDE",
      homeDesc:
        "Samostatná desktopová aplikace v PyBullet pro humanoidního robota iCub. Hotové instalátory a binárky pro Windows, macOS i Linux i instalace přes pip.",
      modesTitle: "Režimy Studia — pyCub Studio",
      modesDesc:
        "Prozkoumejte všech 5 režimů v pyCub Studio: Dětské ovládání, Scratch Blockly, Dětské programování, Normální IDE a Webový rozcestník pro výuku.",
      codeTitle: "Ukázky kódu a API — pyCub Studio",
      codeDesc:
        "České i anglické příkazy kid_api, kartézská kinematika Pose + IK, simulace hmatové kůže, vlastní YAML světy a cvičení 1–5.",
      downloadsTitle: "Stažení aplikace a instalace — pyCub Studio",
      downloadsDesc:
        "Přímé odkazy ke stažení nejnovějších binárek pyCub Studio pro Windows (.exe), macOS (.dmg) a Linux (.AppImage / .deb) i instalace přes pip.",
      changelogTitle: "Novinky a milníky — pyCub Studio",
      changelogDesc:
        "Přehled verzí a hlavních milníků projektu pyCub Studio, knihovny icub_pybullet, Blockly a samostatných instalátorů.",
      aboutTitle: "O projektu a citace — pyCub Studio",
      aboutDesc:
        "Vyvinuto ve skupině Humanoid and Cognitive Robotics Group na Katedře kybernetiky FEL ČVUT v Praze (Lukáš Rustler & Matěj Hoffmann). Článek RiE 2026 a licence CC BY 4.0.",
    },
    nav: {
      navigationLabel: "Navigace",
      items: [
        { name: "Domů", url: "/cs/" },
        { name: "Režimy", url: "/cs/modes" },
        { name: "Kód a API", url: "/cs/code" },
        { name: "Stáhnout", url: "/cs/downloads" },
        { name: "Novinky", url: "/cs/changelog" },
        { name: "O projektu", url: "/cs/about" },
      ],
      github: "GitHub",
      starGithub: "Zobrazit na GitHubu",
      light: "Světlý",
      dark: "Tmavý",
      langSwitchLabel: "English",
    },
    banner: {
      badge: "Samostatné binárky & PyPI",
      text: "Stáhněte nejnovější instalátory pro Windows (.exe), macOS (.dmg) a Linux (.AppImage / .deb) — bez instalace Pythonu",
      cta: "Stáhnout aplikaci",
      showcaseTitle: "pyCub · Výuka humanoidní robotiky s robotem iCub",
      showcaseSubtitle: "Hotové desktopové binárky · Světlé i tmavé OpenGL Studio · 5 navazujících režimů · ČVUT v Praze",
      quickLinks: [
        { label: "Nejnovější verze (Assets)", url: "https://github.com/rustlluk/pyCub/releases/latest", external: true },
        { label: "Článek (arXiv)", url: "https://arxiv.org/abs/2506.01756", external: true },
        { label: "Česká HTML příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/index.html", external: true },
        { label: "Česká PDF příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/pycub.pdf", external: true },
        { label: "Cvičení (.zip)", url: "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/exercises/exercises.zip", external: true },
        { label: "5 režimů Studia", url: "#modes", external: false },
      ],
    },
    beta: {
      badge: "Beta",
      statusLabel: "Beta verze",
      title: "pyCub Studio je momentálně v beta verzi",
      description:
        "Pokud narazíte na jakýkoli problém, chybu nebo potíže při spuštění, nahlaste je prosím na GitHubu nebo napište přímo správci projektu e-mailem.",
      shortNote: "pyCub Studio je v beta verzi — narazili jste na problém? Ozvěte se na",
      githubCta: "Nahlásit na GitHubu",
      emailCta: "Napsat správci",
    },
    hero: {
      badges: ["Windows · macOS · Linux aplikace", "Bez instalace Pythonu", "Čeština & English"],
      title: "Poznejte humanoidní robotiku se simulovaným robotem iCub",
      subtitle:
        "pyCub Studio spojuje hardwarově akcelerovaný 3D OpenGL náhled, pět postupně navazujících režimů od vizuálních bloků po plné Python IDE, přepínání Čeština/English i offline příručky v jedné samostatné aplikaci — stačí stáhnout a spustit.",
      binariesLabel: "Přímé stažení (nejnovější verze):",
      binaryChips: [
        { label: "Windows x64", ext: ".exe", assetKey: "windows-exe" },
        { label: "macOS Silicon", ext: ".dmg", assetKey: "macos-arm64" },
        { label: "macOS Intel", ext: ".dmg", assetKey: "macos-x64" },
        { label: "Linux x64", ext: ".AppImage", assetKey: "linux-x64-appimage" },
      ],
      allBuildsChip: "Všech 7 balíčků (.deb / arm64) →",
      pipAltLabel: "Nebo nainstalujte přes pip (Python 3.10–3.12):",
      copyCmdLabel: "pip install pycub-studio",
      copiedLabel: "Zkopírováno!",
      primaryCta: "Stáhnout aplikaci",
      secondaryCta: "Prozkoumat 5 režimů",
      tertiaryCta: "Ukázky kódu",
      frameTitle: "pyCub Studio — Normální režim (IDE + OpenGL + Oční kamery)",
      frameCaption: "Reálný snímek z pyCub Studio při běhu push_the_ball_cartesian.py s kamerami l_eye RGB a r_eye Depth.",
    },
    strip: {
      label: "Samostatné binárky · Také přes pip a Docker",
      items: [
        { name: "Windows x64", sub: "Instalátor Setup .exe" },
        { name: "macOS Apple Silicon", sub: "Nativní arm64 .dmg" },
        { name: "macOS Intel", sub: "Nativní x86_64 .dmg" },
        { name: "Linux x64 & arm64", sub: "Přenosný .AppImage i .deb" },
        { name: "PyPI & Docker", sub: "pip install pycub-studio" },
      ],
    },
    pillars: {
      title: "Navrženo pro školy, laboratoře i úplné začátečníky",
      description:
        "Většina robotických simulátorů vyžaduje složitý ROS nebo ladění prostředí Pythonu. pyCub Studio vychází jako hotová desktopová aplikace pro každou platformu.",
      items: [
        {
          icon: "/assets/icon/components.png",
          title: "Pět navazujících režimů",
          desc: "Začněte tlačítky přednastavených pohybů, pokračujte bloky Scratch/Blockly či českým kid_api v Pythonu a přejděte k plnému IDE s F12 Přejít na definici.",
        },
        {
          icon: "/assets/icon/fast.png",
          title: "Hotové samostatné binárky",
          desc: "Každý balíček (.exe, .dmg, .AppImage, .deb) už v sobě nese Python, PyBullet, PySide6, OpenGL 3.3, Blockly i českou příručku — stačí rozkliknout a spustit.",
        },
        {
          icon: "/assets/icon/customize.png",
          title: "Kompletní čeština i angličtina",
          desc: "Živé přepínání jazyka v rozcestníku, menu, blocích Blockly, funkcích v Pythonu (zamávej, dotkl_se_míčku) i v přibalené HTML/PDF příručce.",
        },
      ],
    },
    architectureSplit: {
      title: "Stáhněte si hotové Studio — nebo nainstalujte přes Python",
      description:
        "Učitelé i studenti mohou ihned spustit samostatný instalační balíček pyCub Studio bez Pythonu. Výzkumníci a vývojáři mohou navíc nainstalovat pycub-studio či icub_pybullet přes pip.",
      cards: [
        {
          tag: "Desktopové IDE & samostatná aplikace",
          title: "pyCub Studio",
          pkg: "Binárky + pycub-studio",
          desc: "Kompletní multiplatformní aplikace s 5 režimy, offline editorem Blockly, vestavěným spouštěčem skriptů, správcem balíčků, cvičeními 1–5 a českou/anglickou příručkou.",
          bullets: [
            "Bez nutnosti instalovat Python — .exe, .dmg, .AppImage i .deb",
            "Zapisovatelná složka ~/Documents/pyCub/ pro vlastní světy a 3D modely",
            "Případně také jedním příkazem přes pip install pycub-studio",
          ],
          cmd: "Stáhnout nejnovější aplikaci (Windows · macOS · Linux)",
        },
        {
          tag: "Lehká knihovna pro Python",
          title: "Jádro icub_pybullet",
          pkg: "pip install icub_pybullet",
          desc: "Pro vývojáře v Pythonu a výpočetní servery: čistý PyBullet simulátor pro kloubové, rychlostní a kartézské řízení (IK), hmatovou kůži, stereo kamery i kid_api.",
          bullets: [
            "Bez závislostí na Qt GUI — ideální pro servery a automatické testy",
            "Konfigurace světů v YAML (default.yaml, with_ball.yaml, skin_test.yaml)",
            "Dvojjazyčné příkazy kid_api sdílené s prostředím Studia",
          ],
          cmd: "pip install icub_pybullet",
        },
      ],
    },
    modesSection: {
      title: "Pět integrovaných režimů v jedné aplikaci",
      description:
        "Přepínejte mezi kartami níže a prohlédněte si skutečné snímky obrazovky z pyCub Studia — od hravých tlačítek po profesionální vývojové prostředí.",
      launchLabel: "Přímý parametr z příkazové řádky (volitelné)",
      audienceLabel: "Vhodné pro",
      viewAllCta: "Podrobnosti o režimech a složkách",
      modes: [
        {
          id: "kid_native",
          badge: "Režim 1",
          tabLabel: "Dětské ovládání",
          title: "Dětské ovládání — Hravá tlačítka bez psaní kódu",
          subtitle: "Ovládejte robota iCub velkými tlačítky přednastavených póz i kroků jednotlivých kloubů.",
          description:
            "Ideální pro dny otevřených dveří, workshopy a úplně první setkání s robotikou. Nabízí tlačítka přednastavených pohybů (Zamávání, Zvednutí rukou, Pohled na střed), posuvníky kroku a rychlosti, výběr vlastního .yaml světa, živou vzdálenost k míčku a konfety při zásahu.",
          cliFlag: "pycub-studio --mode kid_native --lang cs",
          audience: "Úplní začátečníci, ukázky a popularizační akce",
          screenshot: "/assets/screenshots/kid-native-cs.png",
          screenshotAlt: "pyCub Studio Režim 1 Dětské ovládání v češtině s 3D modelem iCub a červeným míčkem",
          highlights: [
            "Okamžité akce na jedno kliknutí (Zamávání levou/pravou, Obnovit polohu)",
            "Ovládání jednotlivých kloubů s nastavitelným krokem ve stupních a rychlostí",
            "Telemetrie vzdálenosti od míčku a oslava konfetami při úspěšném zásahu",
          ],
          accentColor: "emerald",
        },
        {
          id: "kid_scratch",
          badge: "Režim 2",
          tabLabel: "Bloky Scratch",
          title: "Dětské „Scratch“ Bloky — Vizuální programování offline",
          subtitle: "Skládejte programy z barevných bloků Blockly se zvýrazněním běžícího bloku a živým náhledem Pythonu.",
          description:
            "Postaveno na offline knihovně Google Blockly s kompletním českým překladem všech bloků. Během spuštění se aktuálně vykonávaný blok žlutě zvýrazňuje souběžně s pohybem robota v 3D scéně a postranní panel ukazuje vygenerovaný Python kód s možností exportu do Režimu 3.",
          cliFlag: "pycub-studio --mode kid_scratch --lang cs",
          audience: "Začátečníci a zájemci o blokové programování",
          screenshot: "/assets/screenshots/kid-scratch-cs.png",
          screenshotAlt: "pyCub Studio Režim 2 České bloky Blockly vedle 3D simulátoru iCub",
          highlights: [
            "100% offline Google Blockly s úplným překladem do češtiny i angličtiny",
            "Živé zvýrazňování právě vykonávaného bloku synchronizované s robotem",
            "Ukládání projektů (.pycub_blocks) a převod do Pythonu na jedno kliknutí",
          ],
          accentColor: "amber",
        },
        {
          id: "kid_code",
          badge: "Režim 3",
          tabLabel: "Dětské programování",
          title: "Dětské programování — Vysokoúrovňový Python",
          subtitle: "Most od bloků k reálnému Pythonu díky přehledným příkazům kid_api ve stupních a sekundách.",
          description:
            "Studenti píší čitelné skripty pomocí knihovny icub_pybullet.kid_api bez nutnosti řešit radiány či simulační smyčky. Editor nabízí klikací paletu příkazů, nápovědu signatur, startovní šablony a srozumitelné hlášky chyb.",
          cliFlag: "pycub-studio --mode kid_code --lang cs",
          audience: "Začínající a mírně pokročilí v Pythonu",
          screenshot: "/assets/screenshots/kid-code.png",
          screenshotAlt: "pyCub Studio Režim 3 Dětské programování s paletou příkazů, editorem a 3D scénou",
          highlights: [
            "Intuitivní příkazy ve stupních (wave/zamávej, shoulder_up/zvedni_rameno)",
            "Plně funkční české i anglické názvy všech příkazů",
            "Klikací paleta příkazů, vzorové šablony a přívětivá nápověda k chybám",
          ],
          accentColor: "violet",
        },
        {
          id: "normal",
          badge: "Režim 4",
          tabLabel: "Plné IDE",
          title: "Normální režim — Plnohodnotné pyCub IDE",
          subtitle: "Vícepanelový editor Python/YAML s kamerami z očí robota, zobrazením os a testerem cvičení.",
          description:
            "Pro pokročilou výuku a výzkumné experimenty. Obsahuje vícepanelový editor s doplňováním kódu, dokumentací při najetí myší, přechodem na definici (F12), spouštěčem externích skriptů (--run-script + automatické spuštění při uložení), dokovatelnými pohledy z levého a pravého oka (RGB i hloubka) a instalátorem + testerem kurzových cvičení 1–5.",
          cliFlag: "pycub-studio --mode normal --lang cs",
          audience: "Kurzy robotiky a výzkumníci",
          screenshot: "/assets/screenshots/normal-ide.png",
          screenshotDark: "/assets/screenshots/normal-ide-dark.png",
          screenshotAlt: "pyCub Studio Režim 4 Plné IDE při spuštění kartézského skriptu s pohledy l_eye RGB a r_eye Depth",
          highlights: [
            "Dokovatelné kamery l_eye / r_eye (RGB i Depth) a přepínač 3D os souřadnic",
            "Editor Python/YAML s F12 Přejít na definici a živou konzolí výstupu",
            "Vestavěný instalátor kurzových cvičení 1–5, tester řešení a správce balíčků",
          ],
          accentColor: "blue",
        },
        {
          id: "web",
          badge: "Režim 5",
          tabLabel: "Školní webový server",
          title: "Webový rozcestník — Multi-instanční server pro třídu",
          subtitle: "Spusťte N nezávislých webových simulátorů za jedním vyvažovacím rozcestníkem s QR kódem.",
          description:
            "Vedení workshopů a cvičení v učebně z jediného počítače. Spusťte libovolný počet oddělených webových instancí kid_ui, zobrazte studentům lokální LAN adresu nebo Wi-Fi QR kód pro tablety a notebooky, sledujte počet připojených uživatelů a kontrolujte logy každé instance.",
          cliFlag: "pycub-studio --mode web --lang cs",
          audience: "Učitelé, kroužky a počítačové učebny",
          screenshot: "/assets/screenshots/web-selector.png",
          screenshotAlt: "pyCub Studio Režim 5 Webový rozcestník pro třídu se 2 běžícími instancemi a QR kódem",
          highlights: [
            "Správa N nezávislých prohlížečových instancí za kid_ui_landing.py",
            "LAN URL a naskenovatelný QR kód pro okamžité připojení třídy",
            "Živý stav instancí, počet studentů, restart jednotlivců a prohlížeč logů",
          ],
          accentColor: "cyan",
        },
      ],
    },
    gallerySection: {
      title: "Pohled do Studia — Světlý, tmavý i český motiv",
      description:
        "Každá část aplikace plynule podporuje světlý i tmavý režim a okamžité přepínání mezi češtinou a angličtinou.",
      items: [
        {
          title: "Úvodní rozcestník (Světlý motiv)",
          caption: "Karty pěti režimů, přepínač EN/CZ, volba motivu a rychlý odkaz na příručku.",
          src: "/assets/screenshots/home-light.png",
          tag: "Rozcestník · Světlý",
        },
        {
          title: "Úvodní rozcestník (Tmavý motiv)",
          caption: "Kontrastní tmavé rozhraní vhodné pro projektory v učebnách i večerní programování.",
          src: "/assets/screenshots/home-dark.png",
          tag: "Rozcestník · Tmavý",
        },
        {
          title: "Dětské ovládání v češtině",
          caption: "České popisky tlačítek, jednotlivých kloubů i telemetrie míčku.",
          src: "/assets/screenshots/kid-native-cs.png",
          tag: "Čeština · Režim 1",
        },
        {
          title: "České bloky Scratch (Blockly)",
          caption: "Přehledné české bloky generující čistý Python kód přímo vedle 3D robota.",
          src: "/assets/screenshots/kid-scratch-cs.png",
          tag: "Čeština · Režim 2",
        },
      ],
    },
    codeSection: {
      title: "Od prvního zamávání ke kartézské IK a kůži",
      description:
        "Tyto ukázky můžete spustit přímo v stažené aplikaci pyCub Studio (Režim 3 nebo 4), případně z vlastního Pythonu přes pip.",
      copyBtn: "Kopírovat",
      copiedBtn: "Zkopírováno",
      exploreApiCta: "Všechny ukázky kódu a přehled API",
      samples: [
        {
          id: "kid_cs",
          tabLabel: "kid_api (CZ)",
          title: "Začátečnický Python česky — přímo v Režimu 3",
          filename: "detsky_program_cz.py",
          language: "python",
          description: "Ve stupních a sekundách, bez ruční simulační smyčky — funguje rovnou v stažené aplikaci.",
          code: `from icub_pybullet.kid_api import *

# Pozdrav a pohled na stůl s míčkem
zamávej("pravá")
podívej_se_dolů(15)
print(f"Vzdálenost míčku: {vzdálenost_míčku():.1f} cm")

# Spouštěj pravé rameno, dokud ruka neťukne do míčku
while not dotkl_se_míčku():
    spusť_rameno("pravá", degrees=10, speed=1.8)
    počkej(0.2)

print(f"Počet zásahů: {počet_zásahů()}")
obnov_polohu()`,
        },
        {
          id: "kid_en",
          tabLabel: "kid_api (EN)",
          title: "Stejné API v angličtině — icub_pybullet.kid_api",
          filename: "kid_wave_and_push.py",
          language: "python",
          description: "České i anglické názvy funkcí sdílejí stejný řadič a lze je libovolně kombinovat.",
          code: `from icub_pybullet.kid_api import *

wave("right")
look_down(15)

while not ball_touched():
    shoulder_down("right", degrees=10, speed=1.8)
    wait(0.2)

print(f"Hit count: {ball_hits()}")
reset_pose()`,
        },
        {
          id: "cartesian",
          tabLabel: "Kartézská IK",
          title: "Postrčení míčku v kartézském prostoru (Pose + IK)",
          filename: "push_the_ball_cartesian.py",
          language: "python",
          description: "Běží přímo v Režimu 4 desktopové aplikace pomocí přibaleného Pythonu.",
          code: `from icub_pybullet.pycub import pyCub, Pose

client = pyCub(config="with_ball.yaml")

# Aktuální poloha a orientace koncového efektoru
cur_pose = client.end_effector.get_position()

# Posun o 15 cm níže a 10 cm blíže k trupu
new_pose = Pose(cur_pose.pos, cur_pose.ori)
new_pose.pos[2] -= 0.15
new_pose.pos[0] -= 0.10
client.move_cartesian(new_pose)

# Posun ruky do strany; wait=False umožní řídit smyčku ručně
pose = client.end_effector.get_position()
pose.pos[1] -= 0.20
client.move_cartesian(pose, wait=False, velocity=2)

while not client.motion_done():
    client.update_simulation()`,
        },
        {
          id: "skin",
          tabLabel: "Hmatová kůže & YAML",
          title: "Simulace hmatové kůže a vlastní YAML scéna",
          filename: "vlastni_svet.yaml",
          language: "yaml",
          description: "Uložte do ~/Documents/pyCub/worlds/ — aplikace scénu sama nabídne v menu.",
          code: `# Uložte do ~/Documents/pyCub/worlds/skin_demo.yaml
parent: skin_test.yaml

end_effector: "r_hand"
skin:
  use: True
  radius: 0.015      # dosah paprsků od povrchu kůže iCuba (v metrech)
  num_cores: 0       # 0 = využije všechna volná jádra CPU

eyes:
  l_eye: True        # dokovatelný RGB obraz v Normálním režimu
  r_eye: True        # dokovatelný Depth obraz v Normálním režimu`,
        },
        {
          id: "cli",
          tabLabel: "Parametry / pip",
          title: "Spuštění binárky s parametry nebo instalace přes pip",
          filename: "launch_studio.sh",
          language: "bash",
          description: "Funguje jak se staženými binárkami, tak s balíčkem pycub-studio z PyPI (Python 3.10–3.12).",
          code: `# Samostatná linuxová binárka AppImage (bez instalace Pythonu):
chmod +x pyCub-*-Linux-x86_64.AppImage
./pyCub-*-Linux-x86_64.AppImage --mode kid_scratch --lang cs

# Stejné přepínače přijímá i Windows (.exe) a macOS (.app):
# pyCub.exe --mode normal --run-script muj_skript.py

# Nebo instalace přes pip (Python 3.10–3.12):
python3 -m pip install pycub-studio
pycub-studio`,
        },
      ],
    },
    techStack: {
      title: "Postaveno na osvědčených open-source technologiích",
      description: "Spolehlivá fyzika, hardwarově akcelerované Qt/OpenGL rozhraní a offline vizuální bloky — vše zabaleno v aplikaci bez závislosti na cloudu.",
      cards: [
        { name: "PyBullet", role: "Fyzika & kinematika", desc: "Dynamika tuhých těles, vlastní kolize, VHACD sítě a inverzní kinematika." },
        { name: "PySide6 (Qt 6)", role: "Desktopová aplikace", desc: "Nativní rozhraní, vícepanelový editor, doky pro kamery a světlý/tmavý režim." },
        { name: "OpenGL 3.3 / 2.1", role: "3D zobrazení", desc: "Vestavěné akcelerované vykreslování robota, kůže i os souřadnic." },
        { name: "Google Blockly", role: "Vizuální programování", desc: "100% offline editor bloků v češtině i angličtině s náhledem Pythonu." },
        { name: "Sphinx + LaTeX", role: "Offline příručka", desc: "Kompletní česká i anglická HTML a PDF dokumentace přímo v aplikaci." },
      ],
    },
    downloadsSummary: {
      title: "Stáhněte nejnovější binárky — bez instalace Pythonu",
      description:
        "Přímé odkazy vždy směřují na nejnovější GitHub Release. Vše je zabaleno uvnitř — stačí stáhnout a spustit.",
      latestReleaseBadge: "Vždy z nejnovějšího vydání",
      altDevNote: "Preferujete Python nebo kontejnery?",
      pipTitle: "Instalace přes PyPI (Python 3.10–3.12)",
      pipDesc: "Pro vývojáře, kteří upřednostňují pip před samostatnými binárkami:",
      dockerTitle: "Docker (Nativní & VNC)",
      dockerDesc: "Bez nutnosti řešit grafické ovladače na hostiteli — nativní Linux GUI nebo prohlížeč přes VNC (port 6080).",
      handbookTitle: "Online i offline příručky",
      handbookDesc: "Návody k režimům, API reference, cvičení 1–5 a PDF verze v češtině i angličtině.",
      targets: [
        {
          os: "Windows",
          arch: "x64 (64-bit)",
          format: "Instalátor .exe",
          notes: "Kompletní instalátor se zástupcem v nabídce Start — Python není potřeba.",
          howTo: "Spusťte .exe instalátor a otevřete pyCub Studio z plochy či nabídky Start.",
          downloads: [{ label: "Setup .exe (x64)", assetKey: "windows-exe" }],
        },
        {
          os: "macOS",
          arch: "arm64 (Apple Silicon M1–M4)",
          format: ".dmg",
          notes: "Nativní aplikace pro čipy Apple M včetně přibalené příručky.",
          howTo: "Přetáhněte pyCub.app do Aplikací. První spuštění: povolte v Nastavení systému -> Soukromí a zabezpečení.",
          downloads: [{ label: "Apple Silicon .dmg", assetKey: "macos-arm64" }],
        },
        {
          os: "macOS",
          arch: "x86_64 (Intel Mac)",
          format: ".dmg",
          notes: "Nativní balíček pro Macy s procesory Intel (včetně Open3D 0.19.0).",
          howTo: "Přetáhněte pyCub.app do Aplikací. První spuštění: povolte v Nastavení systému -> Soukromí a zabezpečení.",
          downloads: [{ label: "Intel Mac .dmg", assetKey: "macos-x64" }],
        },
        {
          os: "Linux",
          arch: "x86_64 & arm64",
          format: ".AppImage & .deb",
          notes: "Přenosný jediný soubor AppImage nebo Debian/Ubuntu balíček .deb.",
          howTo: "chmod +x *.AppImage && ./pyCub-*.AppImage (nebo sudo apt install ./*.deb).",
          downloads: [
            { label: "x86_64 .AppImage", assetKey: "linux-x64-appimage" },
            { label: "arm64 .AppImage", assetKey: "linux-arm64-appimage" },
            { label: "amd64 .deb", assetKey: "linux-amd64-deb" },
            { label: "arm64 .deb", assetKey: "linux-arm64-deb" },
          ],
        },
      ],
      ctaReleases: "Všechny možnosti: pip, příručky a Docker",
      ctaGuide: "Otevřít GitHub Releases",
    },
    faq: {
      title: "Často kladené otázky",
      description: "Odpovědi k samostatným binárkám, instalaci přes pip, macOS/Linuxu a nasazení ve výuce.",
      items: [
        {
          question: "Musím mít na počítači nainstalovaný Python, pip nebo PyBullet?",
          answer:
            "Ne! Samostatné binárky a instalátory pro Windows (.exe), macOS (.dmg) a Linux (.AppImage / .deb) už obsahují kompletní Python, pybullet, numpy, scipy, PySide6, OpenGL, Blockly i offline českou příručku. Stačí stáhnout a dvakrát kliknout.",
        },
        {
          question: "Mohu si v samostatné binárce doinstalovat další knihovny (např. OpenCV)?",
          answer:
            "Ano. V Režimu 4 klikněte v horní liště na Balíčky / Python — jakýkoli balíček z PyPI se stáhne do složky ~/Documents/pyCub/extra_packages/, aniž byste potřebovali systémový Python.",
        },
        {
          question: "Co dělat, když macOS při prvním spuštění zobrazí varování Gatekeeperu?",
          answer:
            "Open-source balíčky .dmg jsou podepsané lokálně (ad-hoc), takže je macOS Gatekeeper při prvním spuštění zablokuje. Zkuste pyCub.app spustit, poté otevřete Nastavení systému -> Soukromí a zabezpečení a klikněte na „Přesto otevřít“ (na starších macOS stačí Pravý klik -> Otevřít), případně v Terminálu spusťte: xattr -cr /Applications/pyCub.app.",
        },
        {
          question: "Jaký je rozdíl mezi staženým pyCub Studiem a instalací přes pip?",
          answer:
            "Samostatné binárky jsou hotová desktopová aplikace na jedno kliknutí bez nastavování Pythonu. Pro vývojáře nainstaluje pip install pycub-studio stejné 5režimové Studio do virtuálního prostředí, zatímco pip install icub_pybullet nainstaluje jen samotný lehký simulátor.",
        },
        {
          question: "Mohu do simulace přidat vlastní objekty, URDF nebo vlastní scénáře?",
          answer:
            "Ano. Vlastní soubory .yaml (které mohou dědit z default.yaml nebo with_ball.yaml pomocí klíče parent:) vložte do ~/Documents/pyCub/worlds/ a vlastní .obj nebo .urdf modely do ~/Documents/pyCub/meshes/. U .obj bez souboru *_vhacd.obj se automaticky vypočítá a uloží VHACD kolizní síť.",
        },
        {
          question: "Jak mohu učit celou třídu bez instalace na studentské notebooky?",
          answer:
            "Spusťte Režim 5 (Webový rozcestník pro výuku) ze své stažené aplikace, zvolte počet paralelních instancí a promítněte QR kód na plátno. Studenti se připojí z tabletů, Chromebooků nebo notebooků přímo přes prohlížeč.",
        },
      ],
    },
    cite: {
      title: "Odborný článek a citace",
      subtitle: "Publikováno na 17th International Conference on Robotics in Education (RiE 2026) · Springer",
      paperBtn: "Přečíst článek na arXiv",
      copyBibtex: "Kopírovat BibTeX",
      copiedBibtex: "BibTeX zkopírován!",
      authorsNote: "Humanoid and Cognitive Robotics Group, Katedra kybernetiky, Fakulta elektrotechnická, České vysoké učení technické v Praze",
    },
    finalCta: {
      title: "Rozběhněte simulaci iCuba během pár minut",
      description:
        "Stáhněte si nejnovější binárku pro Windows, macOS nebo Linux, nainstalujte přes pip, nebo otevřete českou příručku.",
      primary: "Stáhnout aplikaci",
      secondary: "Otevřít českou příručku",
    },
    footer: {
      description:
        "Samostatný simulátor humanoidního robota iCub postavený na PyBullet a pěti-režimové výukové Studio vyvinuté na ČVUT v Praze.",
      cols: {
        studio: "Studio",
        developers: "Pro vývojáře",
        handbook: "Příručka & výuka",
        project: "Projekt",
      },
      links: {
        studio: [
          { name: "Domů", url: "/cs/" },
          { name: "Stáhnout nejnovější binárky", url: "/cs/downloads" },
          { name: "Všech 5 režimů", url: "/cs/modes" },
          { name: "Novinky", url: "/cs/changelog" },
        ],
        developers: [
          { name: "Ukázky kódu & kid_api", url: "/cs/code" },
          { name: "GitHub Releases (Nejnovější)", url: "https://github.com/rustlluk/pyCub/releases/latest" },
          { name: "PyPI: pycub-studio", url: "https://pypi.org/project/pycub-studio/" },
          { name: "PyPI: icub_pybullet", url: "https://pypi.org/project/icub-pybullet/" },
        ],
        handbook: [
          { name: "HTML příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/index.html" },
          { name: "PDF příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/pycub.pdf" },
        ],
        project: [
          { name: "O projektu & autoři", url: "/cs/about" },
          { name: "GitHub repozitář", url: "https://github.com/rustlluk/pyCub" },
          { name: "Nahlásit chybu (GitHub)", url: "https://github.com/rustlluk/pyCub/issues" },
          { name: "Článek arXiv (2506.01756)", url: "https://arxiv.org/abs/2506.01756" },
          { name: "Kurzová cvičení (.zip)", url: "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/exercises/exercises.zip" },
        ],
      },
      licenseText: "Původní kód a materiály pod licencí CC BY 4.0.",
      thirdPartyLicenseText: "Licence třetích stran",
      builtWith: "pyCub Studio · ČVUT v Praze",
    },
    pages: {
      modes: {
        heading: "Pět režimů pro každou úroveň znalostí",
        subheading:
          "Od klikacích tlačítek přes bloky Scratch až po plnohodnotné robotické IDE a školní webový server — všechny režimy sdílejí stejnou fyziku PyBullet.",
        tags: ["Režim 1: Dětské GUI", "Režim 2: Scratch Blockly", "Režim 3: Dětský Python", "Režim 4: Plné IDE", "Režim 5: Školní web"],
        workspaceTitle: "Uživatelská složka (~/Documents/pyCub/)",
        workspaceDesc:
          "Při prvním spuštění vytvoří pyCub Studio zapisovatelný pracovní prostor v ~/Documents/pyCub/, díky kterému mohou učitelé i studenti rozšiřovat scény bez zásahu do instalace:",
        workspaceDirs: [
          { dir: "worlds/", desc: "Vlastní .yaml scény automaticky dostupné ve výběru světa. Mohou dědit přes parent: with_ball.yaml." },
          { dir: "meshes/", desc: "Vlastní .obj a .urdf modely. Chybějící soubory *_vhacd.obj se při prvním načtení automaticky vygenerují a uloží do mezipaměti." },
          { dir: "extra_packages/", desc: "Automaticky přidáno do sys.path — doinstalujte další PyPI knihovny přímo ze Studia i bez systémového Pythonu." },
          { dir: "scripts/ & projects/", desc: "Uložené Python skripty (.py) a projekty z Blockly (.pycub_blocks)." },
          { dir: "exercises/ & examples/", desc: "Rozbalená kurzová cvičení 1–5 s testerem řešení a vzorové příklady." },
          { dir: "images/", desc: "Exportované snímky RGB a hloubkových map z kamer l_eye / r_eye." },
        ],
      },
      code: {
        heading: "Ukázky kódu, API a cvičení",
        subheading:
          "Připravené ukázky v Pythonu pro začátečníky (české i anglické kid_api) i pokročilé skripty s kartézskou IK, kůží a vlastními YAML světy.",
        tags: ["kid_api (CZ & EN)", "Kartézská IK", "Hmatová kůže", "Vlastní YAML", "Cvičení 1–5"],
        kidApiTitle: "Dvojjazyčný přehled příkazů kid_api",
        kidApiDesc:
          "Každá vysokoúrovňová funkce funguje stejně přímo v stažené desktopové aplikaci (Režim 3 i Režim 2) i v samostatném Pythonu. České názvy jsou přímými aliasy anglických funkcí:",
        colEn: "Anglický příkaz",
        colCs: "Český alias",
        colDesc: "Akce / popis",
        kidCommands: [
          { en: 'wave("right") / left_wave()', cs: 'zamávej("pravá") / zamávej_levou()', desc: "Robot zamává levou nebo pravou rukou" },
          { en: "both_arms_up() / both_arms_down()", cs: "zvedni_obě_ruce() / spusť_obě_ruce()", desc: "Zvedne nebo spustí obě paže do přednastavené polohy" },
          { en: 'shoulder_up("left", 15)', cs: 'zvedni_rameno("levá", 15)', desc: "Zvedne vybrané rameno o zadaný počet stupňů" },
          { en: 'shoulder_down("right", 15)', cs: 'spusť_rameno("pravá", 15)', desc: "Spustí vybrané rameno o zadaný počet stupňů" },
          { en: 'bend_elbow("right", 20)', cs: 'ohni_loket("pravá", 20)', desc: "Ohne loket o požadovaný úhel ve stupních" },
          { en: "look_up(15) / look_down(15) / turn_head(20)", cs: "podívej_se_nahoru(15) / podívej_se_dolů(15)", desc: "Nakloní nebo otočí hlavu robota" },
          { en: "bend_forward(10) / turn_body(15)", cs: "předkloň_se(10) / otoč_trup(15)", desc: "Předkloní nebo pootočí trup robota" },
          { en: "wait(0.5) / reset_pose() / stop()", cs: "počkej(0.5) / obnov_polohu() / zastav()", desc: "Pozastaví běh se krokováním fyziky, obnoví výchozí postoj či zastaví pohyb" },
          { en: "ball_touched() / ball_distance() / ball_hits()", cs: "dotkl_se_míčku() / vzdálenost_míčku() / počet_zásahů()", desc: "Vrátí živou telemetrii míčku (dotek, vzdálenost v cm, počet zásahů)" },
          { en: "skin_touched()", cs: "dotyk_kůže()", desc: "Zjistí, zda některý senzor hmatové kůže zaznamenal dotek" },
        ],
        exercisesTitle: "Kurzová cvičení (1–5)",
        exercisesDesc:
          "Navazující úlohy připravené k rozbalení přímo v Režimu 4 Studia, včetně zadání README a automatického testeru:",
        exercises: [
          { num: 1, title: "Odtlačení míčku (Push the Ball)", summary: "Seznámení se simulační smyčkou pyCub a využití pozičního či kartézského řízení k bezkoliznímu odtlačení míčku co nejdále od stolu.", skills: "Kloubové & kartézské řízení" },
          { num: 2, title: "Plynulé pohyby (Smooth Movements)", summary: "Plánování spojité trajektorie koncového efektoru bez trhání po přímce nebo kružnici podél zadaných os.", skills: "Kartézské trajektorie" },
          { num: 3, title: "Řízení pohledu (Gaze Control)", summary: "Implementace neblokujícího řízení krčních kloubů pro sledování míčku pohybujícího se po stole podle směrového vektoru pohledu.", skills: "Řízení krku & sledování" },
          { num: 4, title: "Řízení pomocí jakobiánu (RRMC)", summary: "Využití rychlostního řízení podle jakobiánu (RRMC) a shluků aktivovaných taxelů umělé kůže k uhnutí jedné i více končetin před padajícími míčky.", skills: "RRMC & hmatová kůže" },
          { num: 5, title: "Uchopení míčku (Grasp It!)", summary: "Detekce zeleného míčku v RGB obrazu z oční kamery, zpětná projekce jeho středu z hloubkové mapy do 3D a uchopení se zvednutím nad stůl.", skills: "Kamera, hloubka & úchop" },
        ],
      },
      downloads: {
        heading: "Stažení aplikace a instalace",
        subheading:
          "Přímé odkazy na nejnovější binárky pro Windows, macOS a Linux (bez instalace Pythonu) — nebo instalace přes pip či Docker.",
        tags: ["Windows .exe", "macOS .dmg", "Linux .AppImage / .deb", "Nejnovější vydání", "pip & Docker"],
        recommendedBadge: "Doporučeno · Bez Pythonu",
        installersTitle: "1. Samostatné desktopové binárky (Nejnovější vydání)",
        installersDesc:
          "Všechny odkazy směřují přímo na soubory z nejnovějšího vydání na GitHubu a obsahují Python, PyBullet, OpenGL, Blockly i českou/anglickou příručku:",
        macosGatekeeperNote: {
          title: "macOS (.dmg): Povolení aplikace v Nastavení systému",
          description:
            "Open-source balíčky .dmg jsou podepsány pouze lokálně (ad-hoc), takže je macOS Gatekeeper při prvním spuštění zablokuje. Po přetažení pyCub.app do Aplikací a pokusu o spuštění otevřete Nastavení systému -> Soukromí a zabezpečení, sjeďte do sekce Zabezpečení a klikněte na „Přesto otevřít“ (nebo v Terminálu spusťte: xattr -cr /Applications/pyCub.app).",
        },
        binariesIncludedNote:
          "Ani pro vlastní pokročilé skripty v Režimu 4 nepotřebujete systémový Python — v horní liště je okno Balíčky / Python, které umí doinstalovat další knihovny z PyPI přímo do ~/Documents/pyCub/extra_packages/.",
        handbookTitle: "2. Dvojjazyčné příručky a cvičení (HTML, PDF & .zip)",
        handbookDesc: "Offline kopie jsou přibaleny přímo v aplikaci; online verze a archiv cvičení najdete zde:",
        exercisesZipLabel: "Stáhnout kurzová cvičení (exercises.zip)",
        handbookLinks: [
          { label: "Česká HTML příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/index.html", badge: "HTML · CS" },
          { label: "English HTML Handbook", url: "https://rustlluk.github.io/pyCub/handbook/index.html", badge: "HTML · EN" },
          { label: "Česká PDF příručka", url: "https://rustlluk.github.io/pyCub/handbook/cs/pycub.pdf", badge: "PDF · CS" },
          { label: "English PDF Manual", url: "https://rustlluk.github.io/pyCub/handbook/pycub.pdf", badge: "PDF · EN" },
        ],
        pipSectionTitle: "3. Instalace přes pip (Python 3.10–3.12)",
        pipSectionDesc: "Spustitelné z libovolného terminálu — nainstaluje buď kompletní pycub-studio, nebo samostatné jádro icub_pybullet:",
        dockerTitle: "4. Docker (Nativní GUI i webové VNC)",
        dockerDesc: "Skript Docker/deploy.py spustí buď nativní X11/OpenGL okno na Linuxu, nebo prohlížečové noVNC (localhost:6080) na Windows/macOS/Linuxu.",
        buildFromSourceTitle: "5. Sestavení binárek ze zdrojových kódů",
        buildFromSourceDesc: "Vlastní .AppImage, .deb, .exe nebo .dmg vytvoříte pomocí skriptů ve složce packaging/:",
      },
      changelog: {
        heading: "Verze a milníky vývoje",
        subheading: "Přehled hlavních verzí frameworku pyCub a aplikace pyCub Studio.",
        tags: ["pyCub Studio", "Samostatné binárky", "Čeština & English"],
        milestones: [
          {
            version: "Studio Release",
            date: "2026",
            title: "pyCub Studio — Samostatné instalátory, 5režimová aplikace a PyPI",
            highlights: [
              "Automatické CI sestavení samostatných binárek a instalátorů pro Windows (.exe), macOS arm64/x86_64 (.dmg) a Linux (.AppImage/.deb)",
              "Vestavěný Qt + OpenGL 3.3 / 2.1 pohled v režimech 1–4 se zobrazením os a kamer l_eye / r_eye (RGB + Depth)",
              "Offline editor Google Blockly (Režim 2) s živým zvýrazňováním bloků a převodem do Pythonu",
              "Začátečnické kid_api s úplnou sadou českých i anglických funkcí (zamávej, dotkl_se_míčku aj.)",
              "Režim 5 Webový rozcestník pro výuku ve třídě s vyvažováním instancí a Wi-Fi QR kódem",
              "Rozdělení na desktopovou distribuci pycub-studio a samostatné jádro icub_pybullet",
            ],
          },
          {
            version: "v1.1.1",
            date: "2026",
            title: "Kompatibilita OS, dvojjazyčná příručka a cvičení 1–5",
            highlights: [
              "Přidán kompletní český překlad dokumentace a generování PDF příruček přes docs/build.py",
              "Přibaleno pět navazujících kurzových cvičení se šablonami a automatickým testováním řešení",
              "Publikován preprint na arXiv (2506.01756) a článek na konferenci RiE 2026",
            ],
          },
          {
            version: "v1.0.0",
            date: "2025",
            title: "Jádro simulátoru icub_pybullet a simulace hmatové kůže",
            highlights: [
              "Simulace kompletního URDF modelu iCub v PyBullet s konvexní dekompozicí kolizí VHACD",
              "Kloubové, rychlostní a kartézské řízení (Pose + IK) s konfigurací scén v YAML",
              "Vícevláknová simulace hmatové kůže pomocí paprsků a stereo oční kamery",
            ],
          },
        ],
      },
      about: {
        heading: "O projektu pyCub a týmu na ČVUT",
        subheading: "Open-source vzdělávací framework pro humanoidní robotiku vyvinutý ve skupině Humanoid and Cognitive Robotics Group na Katedře kybernetiky Fakulty elektrotechnické ČVUT v Praze.",
        tags: ["FEL ČVUT v Praze", "RiE 2026", "CC BY 4.0"],
        storyTitle: "Proč vzniklo pyCub Studio",
        storyParagraphs: [
          "Výuka humanoidní robotiky často naráží na technickou bariéru: skutečný robot iCub je vzácný a křehký, zatímco výzkumné simulátory vyžadují složitý Linux/ROS stack ještě předtím, než student pohne jediným kloubem.",
          "Projekt pyCub vznikl na ČVUT v Praze právě proto, aby tuto propast překlenul. Jeho jádro icub_pybullet zapouzdřuje PyBullet s přesnou kinematikou robota iCub, simulací hmatové kůže, stereo kamerami a reprodukovatelnými YAML scénami.",
          "pyCub Studio vše balí do samostatných spustitelných binárek pro Windows, macOS i Linux (i balíčků na PyPI) — začátečníci mohou skládat bloky Blockly česky nebo anglicky, pokročilejší zkoušet kid_api a odborné kurzy ladit kartézskou kinematiku se živým pohledem z očních kamer.",
        ],
        authorsTitle: "Autoři a pracoviště",
        authors: [
          {
            name: "Lukáš Rustler",
            role: "Hlavní autor a správce projektu",
            org: "Humanoid and Cognitive Robotics Group, Katedra kybernetiky, Fakulta elektrotechnická, České vysoké učení technické v Praze",
            url: "https://www.lukasrustler.cz",
          },
          {
            name: "Matěj Hoffmann",
            role: "Vedoucí výzkumné skupiny a spoluautor",
            org: "Humanoid and Cognitive Robotics Group, Katedra kybernetiky, Fakulta elektrotechnická, České vysoké učení technické v Praze",
            url: "https://sites.google.com/site/matejhof/",
          },
        ],
        highlightsTitle: "Hlavní principy",
        highlights: [
          { title: "Plynulý růst obtížnosti", desc: "Jeden konzistentní 3D robot od klikacích tlačítek a Blockly až po IK a hmatovou zpětnou vazbu." },
          { title: "Samostatné binárky i pip", desc: "Hotové soubory .exe, .dmg, .AppImage a .deb i instalace přes pip a webový QR server pro celou třídu." },
          { title: "Přirozeně dvojjazyčné", desc: "Plná podpora češtiny i angličtiny v GUI, blocích, Python kid_api i v dokumentaci." },
          { title: "Otevřený výukový materiál", desc: "Původní kód, příručka i cvičení jsou šířeny pod CC BY 4.0; modely robota a přibalené knihovny si zachovávají své vlastní otevřené licence." },
        ],
        licenseSection: {
          title: "Licence a komponenty třetích stran",
          subtitle:
            "Veškerý původní kód pyCub, dokumentace, kurzová cvičení i vlastní 3D sítě autorů jsou šířeny pod licencí Creative Commons Attribution 4.0 International (CC BY 4.0). Přibalené modely třetích stran a běhové knihovny podléhají svým vlastním otevřeným licencím:",
          items: [
            {
              component: "Původní pyCub, icub_pybullet & pyCub Studio",
              license: "CC BY 4.0",
              details: "Veškerý původní kód v Pythonu/JS, zdroje příruček, kurzová cvičení 1–5 a vlastní sítě objektů (icub_pybullet/other_meshes/).",
              url: "https://creativecommons.org/licenses/by/4.0/",
            },
            {
              component: "Modely robota iCub a data kůže (icub_pybullet/iCub/)",
              license: "CC-BY-SA-4.0",
              details: "URDF soubory, vizuální sítě, odvozené kolizní sítě VHACD a kalibrační data hmatové kůže z robotology/icub-models (v1.25.0).",
              url: "https://creativecommons.org/licenses/by-sa/4.0/",
            },
            {
              component: "Offline vizuální editor Blockly (studio/assets/blockly/)",
              license: "Apache-2.0",
              details: "Balíky Google / Raspberry Pi Foundation Blockly v10.4.3, překlady a média (vlastní pycub_blocks.js a index.html zůstávají pod CC BY 4.0).",
              url: "https://www.apache.org/licenses/LICENSE-2.0",
            },
            {
              component: "Desktopový runtime instalátorů (PySide6 / Qt 6, OpenCV)",
              license: "LGPLv3 / LGPLv2.1",
              details: "Dynamicky linkované PySide6 / Qt 6 / QtWebEngine v onedir balíčcích (LGPL-3.0, Chromium LGPL-2.1), opencv-python (Apache-2.0 / FFmpeg LGPL-2.1) a permisivní knihovny Pythonu (MIT, BSD, zlib).",
              url: "https://github.com/rustlluk/pyCub/blob/master/THIRD_PARTY_LICENSES.md",
            },
          ],
          primaryLinkLabel: "Licence CC BY 4.0",
          thirdPartyLinkLabel: "Kompletní přehled licencí (GitHub)",
        },
      },
    },
  },
};
