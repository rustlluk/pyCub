# CLAUDE.md

This file gives AI coding assistants the working context and conventions needed when editing this repository.

## Project

RicoFast is a static-first SaaS website template built with Astro, Tailwind CSS v4, MDX, and a reusable component system.

It is a front-end template, not a full backend SaaS product. The project includes marketing pages, MDX blog/changelog content, demo pricing, demo auth screens, a demo contact form, SEO setup, dark mode, and a documented design system.

Primary docs:

- `docs/PRD.md` - current implemented product scope.
- `docs/PLAN.md` - concise project overview.
- `docs/DESIGN.md` - design tokens and UI rules.
- `docs/plan/` - page-level notes.
- `README.md` and `README-zh.md` - user-facing setup docs.

## Current Stack

| Layer | Choice |
| --- | --- |
| Framework | Astro 5.15 |
| Styling | Tailwind CSS v4 with `@theme` tokens |
| Content | MDX, `@astrojs/mdx`, Astro Content Layer |
| Icons | `@lucide/astro` |
| Motion | AOS, CSS animation, `motion` dependency |
| SEO | `@astrojs/sitemap`, `@astrojs/rss`, custom meta components |
| Type checking | TypeScript, `astro check` |
| Lint/format | Biome |
| Images | Static assets, `sharp` |

The dev server is configured for port `5200` in `astro.config.mjs`.

## Commands

```bash
pnpm install
pnpm dev
pnpm build
pnpm preview
pnpm check
pnpm astro
```

`pnpm build` runs `astro check && astro build`.

## Repository Layout

```text
src/
  assets/js/main.js        Header behavior, dark mode, mobile menu, AOS init
  collections/             JSON data for menu, social links, tech stack
  components/
    cards/                 BlogCard, TechStackCard
    elements/              PageHeader, SectionHeader, SeparatorLine
    home/                  HeroSection
    sections/              Header, Footer, Pricing, FAQ, BlogSection
    ui/                    Button, Badge, Logo, BrowserFrame, PricingToggle, etc.
    widgets/               Toc, Pagination, ToTop, TrackGa, OptimizedImage, etc.
  config/site.js           Site identity, metadata, social URLs, email
  content/
    post/                  Blog MDX entries
    changelog/             Changelog MDX entries
  layouts/                 Layout, PageLayout, PostLayout, Meta
  pages/                   Astro routes
  styles/                  Global tokens, article styles, AOS overrides
  content.config.js        Content Layer schemas
public/
  assets/                  Template images and icons
  rico/                    Rico/RicoUI support assets
  favicon.png
  og.jpg
  robots.txt
docs/
  PRD.md
  PLAN.md
  DESIGN.md
  plan/
```

## Routes

Current routes include:

- `/`
- `/features`
- `/pricing`
- `/blog`
- `/blog/[slug]`
- `/blog/page/[page]`
- `/changelog`
- `/about`
- `/contact`
- `/elements`
- `/sign-in`
- `/sign-up`
- `/signin`
- `/signup`
- `/rss.xml`
- `/404`

`/sign-in` and `/sign-up` are the preferred auth-template routes. `/signin` and `/signup` are also present in the repo, so check both before changing auth screens.

## Design System

Design tokens live in `src/styles/global.css` and are documented in `docs/DESIGN.md`.

Important tokens and rules:

- Primary color: `--color-primary` = `#2d6dc3`
- Accent color: `--color-accent` = `#fad13b`
- Light canvas: `--color-bg-primary` = `#fdfaf5`
- Dark canvas: `--color-bg-primary-dark` = `#0b1220`
- Display font: `--font-brand` = Instrument Serif
- UI/body font: `--font-sans` = Inter
- Main max width: `--max-screen` = `1200px`
- Inner max width: `--inner-screen` = `800px`
- Use `.site-container` for regular sections.
- Use `.inner-container` for narrow article/content layouts.
- Dark mode is class-based and stored in `localStorage` as `dark_mode`.

When adding UI:

- Use existing tokens before adding new values.
- Use existing components before creating new ones.
- Keep light and dark mode styles together.
- Keep motion subtle and respect `prefers-reduced-motion`.
- Prefer Lucide icons via `@lucide/astro`.
- Do not introduce a new dependency for simple UI behavior.

## Astro And Content Rules

- Use Astro Content Layer collections from `src/content.config.js`.
- Use `getCollection()` for blog/changelog content.
- Use `entry.id` for content routes.
- Use `render(entry)` rather than old Astro content APIs.
- Use `import.meta.env` for environment variables.
- Public client-exposed env vars must use the `PUBLIC_` prefix.
- `getStaticPaths()` params should be strings.

Blog posts live in `src/content/post/<slug>/index.mdx`.

Changelog entries live in `src/content/changelog/*.mdx`.

## Component Conventions

- Page-level sections go in `src/components/sections/`.
- Small reusable primitives go in `src/components/ui/`.
- Structural text/layout helpers go in `src/components/elements/`.
- Repeated cards go in `src/components/cards/`.
- Page utilities go in `src/components/widgets/`.
- Site identity should come from `src/config/site.js` when practical.
- Navigation comes from `src/collections/menu.json`.

## Styling Conventions

- Prefer Tailwind utilities plus project tokens.
- Avoid one-off hex values unless extending the token system deliberately.
- Avoid inline styles for colors and layout unless dynamic CSS variables are genuinely needed.
- Use `font-brand` only for display headings.
- Use the default sans font for body, forms, labels, navigation, and buttons.
- Major marketing sections commonly use `py-16 md:py-24`.
- Use dashed borders where the existing visual language does.
- Ensure text fits on mobile before finishing UI work.

## Motion Conventions

- AOS is initialized in `src/assets/js/main.js`.
- Custom AOS styles live in `src/styles/aos-custom.css`.
- Common reveal attributes:

```html
data-aos="fade-up-xs"
data-aos-once="true"
```

- Use staggered delays sparingly, usually `data-aos-delay={i * 100}`.
- Use CSS transitions/animations for small local effects.
- Use `motion` only when orchestration is worth the dependency.

## SEO And Analytics

Relevant files:

- `src/config/site.js`
- `src/layouts/Meta.astro`
- `src/components/widgets/Meta.astro`
- `src/components/widgets/TrackGa.astro`
- `src/pages/rss.xml.js`
- `public/og.jpg`
- `public/robots.txt`

Environment variables:

```env
PUBLIC_SITE_URL=https://your-domain.com
PUBLIC_GA4_ID=
PUBLIC_UMAMI_ID=
```

Analytics are optional. Do not require analytics IDs for local development.

## Before Editing

1. Read the relevant page/component first.
2. Check `docs/PRD.md` for current scope.
3. Check `docs/DESIGN.md` before changing visual patterns.
4. Check the matching `docs/plan/*.md` file when changing a page.
5. Preserve unrelated user changes in the working tree.

## After Editing

For code changes, run the narrowest useful verification:

- `pnpm build` for route/content/type changes.
- `pnpm check` for formatting/lint-sensitive edits.
- Manual browser review for visual/layout changes.

For docs-only changes, a text scan is usually enough.

Useful scans:

```bash
rg -n "TODO|FIXME|\\[ \\]" docs README.md README-zh.md CLAUDE.md
rg -n "template positioning|backend feature|real auth" docs README.md README-zh.md CLAUDE.md
```

## Do Not

- Do not reintroduce previous-template positioning.
- Do not recreate removed planning docs unless explicitly asked.
- Do not add new public claims that are not reflected in the current code.
- Do not make the static auth/contact pages sound like real backend features.
- Do not add real secrets or private credentials.
- Do not replace the established Astro + Tailwind + MDX stack without an explicit request.
