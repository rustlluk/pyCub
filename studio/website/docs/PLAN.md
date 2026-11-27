# RicoFast Project Overview

> Last updated: 2026-05-30

RicoFast is a static-first SaaS website template built with Astro, Tailwind CSS v4, MDX, and a reusable design system.

## Positioning

A polished SaaS template for products that need to look credible from day one.

## Target Users

- Indie makers
- Startup founders
- AI product teams
- Developer tool teams
- Open-source maintainers
- Designers and engineers building product launch sites

## Tech Stack

- Astro 6.4
- Vite 7.3
- Tailwind CSS v4
- MDX
- TypeScript
- AOS
- motion
- Lucide icons
- Astro RSS and sitemap integrations

## Implemented Pages

| Page | Route | Purpose |
| --- | --- | --- |
| Home | `/` | Full SaaS landing page |
| Features | `/features` | Feature overview |
| Pricing | `/pricing` | Demo pricing page |
| Blog | `/blog` | MDX blog index |
| Blog detail | `/blog/[slug]` | MDX article pages |
| Changelog | `/changelog` | Release notes |
| About | `/about` | Project story and values |
| Contact | `/contact` | Demo contact form |
| Elements | `/elements` | Component and design reference |
| Sign in | `/sign-in` | Auth template page |
| Sign up | `/sign-up` | Auth template page |
| RSS | `/rss.xml` | Blog RSS feed |
| 404 | `/404` | Not-found page |

The repository also currently includes `/signin` and `/signup` alternate auth pages.

## Main Customization Files

- `src/config/site.js` - site identity, metadata, social links.
- `src/styles/global.css` - design tokens, global styles, dark mode variables.
- `src/collections/menu.json` - navigation.
- `src/collections/social.json` - social data.
- `src/collections/stack.json` - tech stack data.
- `src/content/post/` - blog posts.
- `src/content/changelog/` - changelog entries.
- `public/` - logo, favicon, OG image, and static assets.

## Documentation Map

- `docs/PRD.md` - complete current product requirements.
- `docs/DESIGN.md` - design system reference.
- `docs/plan/` - page-level notes.
- `README.md` - English setup and usage guide.
- `README-zh.md` - Chinese setup and usage guide.
