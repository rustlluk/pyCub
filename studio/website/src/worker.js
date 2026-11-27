const LANG_COOKIE_NAME = "pycub_lang";
const CZECH_COUNTRIES = new Set(["CZ", "SK"]);
const BOT_USER_AGENT_RE =
  /bot|crawler|spider|slurp|mediapartners|facebookexternalhit|whatsapp|telegrambot|slackbot|discordbot|twitterbot|linkedinbot|embedly|preview|lighthouse|pagespeed/i;

function getCookieValue(cookieHeader, name) {
  if (!cookieHeader) return null;
  const parts = cookieHeader.split(";");
  for (const part of parts) {
    const eqIndex = part.indexOf("=");
    if (eqIndex === -1) continue;
    const key = part.slice(0, eqIndex).trim();
    if (key === name) {
      try {
        return decodeURIComponent(part.slice(eqIndex + 1).trim());
      } catch {
        return part.slice(eqIndex + 1).trim();
      }
    }
  }
  return null;
}

function isStaticOrNonHtmlPath(pathname) {
  if (
    pathname === "/cs" ||
    pathname.startsWith("/cs/") ||
    pathname === "/404" ||
    pathname === "/404.html" ||
    pathname.startsWith("/_astro/") ||
    pathname.startsWith("/assets/")
  ) {
    return true;
  }

  const lastSegment = pathname.split("/").pop() || "";
  if (lastSegment.includes(".") && !lastSegment.endsWith(".html")) {
    return true;
  }

  return false;
}

function toCzechCandidatePath(pathname) {
  let clean = pathname;
  if (clean.endsWith("/index.html")) {
    clean = clean.slice(0, -"/index.html".length) || "/";
  } else if (clean.endsWith(".html")) {
    clean = clean.slice(0, -".html".length) || "/";
  }
  if (clean !== "/") {
    clean = clean.replace(/\/+$/, "") || "/";
  }
  return clean === "/" ? "/cs/" : `/cs${clean}`;
}

export default {
  async fetch(request, env) {
    if (request.method !== "GET" && request.method !== "HEAD") {
      return env.ASSETS.fetch(request);
    }

    const url = new URL(request.url);
    if (isStaticOrNonHtmlPath(url.pathname)) {
      return env.ASSETS.fetch(request);
    }

    const userAgent = request.headers.get("User-Agent") || "";
    if (BOT_USER_AGENT_RE.test(userAgent)) {
      return env.ASSETS.fetch(request);
    }

    const savedLang = getCookieValue(request.headers.get("Cookie"), LANG_COOKIE_NAME);
    if (savedLang === "en") {
      return env.ASSETS.fetch(request);
    }

    const country =
      typeof request.cf?.country === "string" ? request.cf.country.toUpperCase() : "";
    const shouldRedirectToCs =
      savedLang === "cs" || (!savedLang && CZECH_COUNTRIES.has(country));

    if (!shouldRedirectToCs) {
      return env.ASSETS.fetch(request);
    }

    const candidateUrl = new URL(toCzechCandidatePath(url.pathname), url.origin);
    const probeRes = await env.ASSETS.fetch(
      new Request(candidateUrl.toString(), { method: "HEAD" }),
    );

    let redirectTarget = null;
    if (probeRes.status >= 200 && probeRes.status < 300) {
      redirectTarget = candidateUrl;
    } else if (probeRes.status === 307 || probeRes.status === 308) {
      const location = probeRes.headers.get("Location");
      redirectTarget = location ? new URL(location, url.origin) : candidateUrl;
    }

    if (!redirectTarget) {
      return env.ASSETS.fetch(request);
    }

    redirectTarget.search = url.search;

    return new Response(null, {
      status: 302,
      headers: {
        Location: redirectTarget.toString(),
        "Cache-Control": "private, no-store",
        Vary: "Cookie, CF-IPCountry",
      },
    });
  },
};
