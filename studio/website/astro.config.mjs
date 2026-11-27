import tailwindcss from "@tailwindcss/vite";
import { defineConfig } from "astro/config";
import mdx from "@astrojs/mdx";
import sitemap from "@astrojs/sitemap";
import { fileURLToPath } from "url";
import path from "path";
import { loadEnv } from "vite";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const env = loadEnv(process.env.NODE_ENV ?? "production", process.cwd(), "PUBLIC_");
const siteUrl =
  process.env.PUBLIC_SITE_URL || env.PUBLIC_SITE_URL || "https://pycub.lukasrustler.cz";

export default defineConfig({
  site: siteUrl,
  base: "/",
  envPrefix: "PUBLIC_",
  i18n: {
    defaultLocale: "en",
    locales: ["en", "cs"],
    routing: {
      prefixDefaultLocale: false,
    },
  },
  vite: {
    plugins: [tailwindcss()],
    resolve: {
      alias: {
        "@": path.resolve(__dirname, "./src"),
      },
    },
  },
  server: {
    port: 5200,
  },
  integrations: [
    mdx(),
    sitemap({
      i18n: {
        defaultLocale: "en",
        locales: {
          en: "en-US",
          cs: "cs-CZ",
        },
      },
    }),
  ],
});
