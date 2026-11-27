import AOS from "aos";

const stickyClassesContainer = [
  "shadow-[0px_1px_4px_0_rgba(25,33,61,0.06)]",
  "rounded-[20px]",
  "bg-[#ffffff]/80",
  "border-[#ffffff]/65",
  "dark:border-neutral-600/40",
  "dark:bg-neutral-900/75",
  "backdrop-blur-2xl",
  "backdrop-brightness-110",
];
const unstickyClassesContainer = [
  "shadow-none",
  "border-transparent",
  "rounded-none",
  "bg-transparent",
];
let headerElement = null;

function normalizePath(p) {
  if (!p || p === "/") return "/";
  return p.replace(/\/+$/, "");
}

function syncThemeState(isDark) {
  window.darkMode = isDark;
  const sunIcons = document.querySelectorAll("#sun, .mobile-sun");
  const moonIcons = document.querySelectorAll("#moon, .mobile-moon");
  const dayText = document.getElementById("dayText");
  const nightText = document.getElementById("nightText");
  const toggle = document.getElementById("darkToggle");
  const toggleMobile = document.getElementById("darkToggleMobile");

  if (isDark) {
    moonIcons.forEach((icon) => icon.classList.remove("hidden"));
    sunIcons.forEach((icon) => icon.classList.add("hidden"));
    dayText?.classList.add("hidden");
    nightText?.classList.remove("hidden");
  } else {
    sunIcons.forEach((icon) => icon.classList.remove("hidden"));
    moonIcons.forEach((icon) => icon.classList.add("hidden"));
    dayText?.classList.remove("hidden");
    nightText?.classList.add("hidden");
  }

  toggle?.setAttribute("aria-pressed", isDark ? "true" : "false");
  toggleMobile?.setAttribute("aria-pressed", isDark ? "true" : "false");
}

function setTheme(isDark, persist = true) {
  document.documentElement.classList.toggle("dark", isDark);
  if (persist) {
    try {
      localStorage.setItem("dark_mode", isDark ? "true" : "false");
    } catch {
      /* ignore storage restrictions */
    }
  }
  syncThemeState(isDark);
}

const RELEASE_API = "https://api.github.com/repos/rustlluk/pyCub/releases/latest";
const RELEASE_CACHE_KEY = "pycub_latest_release_v1";
const RELEASE_CACHE_TTL_MS = 10 * 60 * 1000;

const ASSET_MATCHERS = {
  "windows-exe": /Windows-x64\.exe$/i,
  "macos-arm64": /macOS-arm64\.dmg$/i,
  "macos-x64": /macOS-x86_64\.dmg$/i,
  "linux-x64-appimage": /Linux-x86_64\.AppImage$/i,
  "linux-arm64-appimage": /Linux-arm64\.AppImage$/i,
  "linux-amd64-deb": /_amd64\.deb$/i,
  "linux-arm64-deb": /_arm64\.deb$/i,
};

function formatMB(bytes) {
  if (!bytes || typeof bytes !== "number") return "";
  return `${Math.round(bytes / (1024 * 1024))} MB`;
}

function applyReleaseData(data) {
  if (!data || !Array.isArray(data.assets)) return;

  if (data.tag_name) {
    document.querySelectorAll(".release-latest-tag").forEach((el) => {
      el.textContent = data.tag_name;
    });
  }

  const byKey = {};
  for (const [key, regex] of Object.entries(ASSET_MATCHERS)) {
    const match = data.assets.find((a) => regex.test(a.name || ""));
    if (match) byKey[key] = match;
  }

  document.querySelectorAll("[data-release-asset]").forEach((el) => {
    const key = el.getAttribute("data-release-asset");
    const asset = key ? byKey[key] : null;
    if (!asset) return;

    if (el.tagName === "A" && asset.name) {
      el.setAttribute(
        "href",
        `https://github.com/rustlluk/pyCub/releases/latest/download/${encodeURIComponent(asset.name)}`
      );
      el.setAttribute("title", `${asset.name} (${formatMB(asset.size)})`);
    }
    const nameTarget = el.querySelector(".release-asset-name");
    if (nameTarget && asset.name) nameTarget.textContent = asset.name;
    const sizeTarget = el.querySelector(".release-asset-size");
    if (sizeTarget && asset.size) sizeTarget.textContent = formatMB(asset.size);
  });

  document.querySelectorAll("[data-release-name-for]").forEach((el) => {
    const key = el.getAttribute("data-release-name-for");
    const asset = key ? byKey[key] : null;
    if (asset && asset.name) el.textContent = asset.name;
  });
}

async function hydrateLatestReleaseLinks() {
  const hasTargets =
    document.querySelector("[data-release-asset]") ||
    document.querySelector(".release-latest-tag") ||
    document.querySelector("[data-release-name-for]");
  if (!hasTargets) return;

  try {
    const cachedRaw = sessionStorage.getItem(RELEASE_CACHE_KEY);
    if (cachedRaw) {
      const cached = JSON.parse(cachedRaw);
      if (cached && Date.now() - cached.ts < RELEASE_CACHE_TTL_MS && cached.data) {
        applyReleaseData(cached.data);
        return;
      }
    }
  } catch {
    /* ignore storage errors */
  }

  try {
    const res = await fetch(RELEASE_API, {
      headers: { Accept: "application/vnd.github+json" },
    });
    if (!res.ok) return;
    const data = await res.json();
    applyReleaseData(data);
    try {
      sessionStorage.setItem(RELEASE_CACHE_KEY, JSON.stringify({ ts: Date.now(), data }));
    } catch {
      /* ignore storage errors */
    }
  } catch {
    /* keep static /releases/latest/download/... fallback links */
  }
}

document.addEventListener("DOMContentLoaded", () => {
  headerElement = document.getElementById("header");

  const isDark = document.documentElement.classList.contains("dark");
  syncThemeState(isDark);

  stickyHeaderFuncionality();
  applyMenuItemClasses();
  evaluateHeaderPosition();
  hydrateLatestReleaseLinks();

  AOS.init({
    duration: 450,
    easing: "ease-out-cubic",
    once: true,
    offset: 20,
    delay: 0,
  });
});

window.stickyHeaderFuncionality = () => {
  window.addEventListener("scroll", () => {
    evaluateHeaderPosition();
  });
};

window.evaluateHeaderPosition = () => {
  if (!headerElement || !headerElement.firstElementChild) return;
  if (window.scrollY > 48) {
    headerElement.firstElementChild.classList.add(...stickyClassesContainer);
    headerElement.firstElementChild.classList.remove(...unstickyClassesContainer);
  } else {
    headerElement.firstElementChild.classList.remove(...stickyClassesContainer);
    headerElement.firstElementChild.classList.add(...unstickyClassesContainer);
  }
};

document.getElementById("darkToggle")?.addEventListener("click", () => {
  const nextIsDark = !document.documentElement.classList.contains("dark");
  setTheme(nextIsDark, true);
});

window.applyMenuItemClasses = () => {
  const current = normalizePath(window.location.pathname);
  const menuItems = document.querySelectorAll("#menu a");
  for (let i = 0; i < menuItems.length; i++) {
    const itemPath = normalizePath(menuItems[i].pathname);
    if (itemPath === current) {
      menuItems[i].classList.add("text-primary", "dark:text-primary-light", "font-semibold");
    }
  }
};
