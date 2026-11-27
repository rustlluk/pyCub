# RicoFast PRD

> This PRD describes the current implemented RicoFast project.

## Product Summary

RicoFast is a design-led SaaS website template built with Astro, Tailwind CSS v4, MDX, and a reusable component system. It helps indie makers, AI tool teams, developer platform teams, and open-source maintainers launch a polished marketing site quickly.

The project is a static-first front-end template. It includes demo pricing, demo auth pages, demo contact behavior, MDX content, SEO setup, dark mode, and a documented design system.

## Positioning

| Field | Current value |
| --- | --- |
| Product name | RicoFast |
| Category | Astro SaaS website template |
| Tagline | A design-led SaaS template for Astro |
| Core promise | Ship a polished SaaS site in days, not weeks. |
| Audience | Indie makers, AI/dev-tool teams, OSS maintainers |
| Brand color | `#2d6dc3` |
| Accent color | `#fad13b` |
| Contact email | `hello@ricoui.com` |
| Repository | `https://github.com/ricocc/ricoui-saas-template` |

## Implemented Scope

- Static SaaS marketing site built with Astro 6.4.
- Tailwind CSS v4 design token system in `src/styles/global.css`.
- Light and dark modes with persisted user preference.
- MDX blog and changelog powered by Astro Content Layer.
- Reusable section, UI, card, layout, and widget components.
- SEO foundations: meta layout, Open Graph image, sitemap, RSS, robots.txt, canonical site URL support.
- Demo contact form with local UI behavior.
- Demo sign-in and sign-up pages.
- Responsive layouts for mobile, tablet, and desktop.

## Tech Stack

| Area | Implementation |
| --- | --- |
| Framework | Astro 6.4 |
| Build tooling | Vite 7.3 |
| Styling | Tailwind CSS v4, CSS custom properties, `@theme` tokens |
| Content | MDX, `@astrojs/mdx`, Astro Content Layer |
| Icons | `@lucide/astro` |
| Motion | AOS, CSS animation, `motion` dependency |
| SEO | `@astrojs/sitemap`, `@astrojs/rss`, custom meta components |
| Type safety | TypeScript, `astro check` |
| Code quality | Biome |
| Images | Static assets, `sharp` |

## Routes

| Route | Purpose | Source |
| --- | --- | --- |
| `/` | SaaS landing page | `src/pages/index.astro` |
| `/features` | Feature overview | `src/pages/features.astro` |
| `/pricing` | Demo pricing page | `src/pages/pricing.astro` |
| `/blog` | Blog index | `src/pages/blog/index.astro` |
| `/blog/[slug]` | Blog article pages | `src/pages/blog/[...slug].astro` |
| `/blog/page/[page]` | Paginated blog archive | `src/pages/blog/page/[page].astro` |
| `/changelog` | Release notes | `src/pages/changelog.astro` |
| `/about` | Product story and values | `src/pages/about.astro` |
| `/contact` | Demo contact page | `src/pages/contact.astro` |
| `/elements` | Component and design-system reference | `src/pages/elements.astro` |
| `/sign-in` | Sign-in template page | `src/pages/sign-in.astro` |
| `/sign-up` | Sign-up template page | `src/pages/sign-up.astro` |
| `/signin` | Alternate sign-in page currently present in repo | `src/pages/signin.astro` |
| `/signup` | Alternate sign-up page currently present in repo | `src/pages/signup.astro` |
| `/rss.xml` | RSS feed | `src/pages/rss.xml.js` |
| `/404` | Not-found page | `src/pages/404.astro` |

## Navigation

Navigation is defined in `src/collections/menu.json`.

Current top-level navigation:

- Home
- Pages dropdown
- Elements
- Contact

Current Pages dropdown:

- Features
- Pricing
- About
- Blog
- Changelog
- Sign in
- Sign up
- 404

Footer navigation is defined in `src/components/sections/Footer.astro`.

Current footer groups:

- Product: Features, Pricing, Changelog, Sign in, Sign up
- Resources: Blog, Elements, 404, Docs, GitHub
- Company: About, Contact
- Legal: License

## Page Requirements

### Home

Source: `src/pages/index.astro`

The home page presents RicoFast as a finished SaaS template. It includes:

- Hero section.
- Social proof/logo strip.
- Problem cards.
- Solution section.
- Feature grid.
- Product preview.
- Use cases.
- Tech stack.
- Pricing preview.
- FAQ.
- Latest articles.
- Final CTA.

### Features

Source: `src/pages/features.astro`

The features page expands the core template capabilities into detailed sections. It should stay aligned with the home page feature language while giving visitors more implementation context.

### Pricing

Sources:

- `src/pages/pricing.astro`
- `src/components/sections/Pricing.astro`

The pricing page is demo content for template users. It includes Free, Pro, and Enterprise sample tiers, a billing toggle, comparison content, FAQ, and CTA.

### Blog

Sources:

- `src/pages/blog/index.astro`
- `src/pages/blog/[...slug].astro`
- `src/pages/blog/page/[page].astro`
- `src/content/post/`

The blog renders MDX posts from the `post` collection, supports metadata and images, and uses `PostLayout.astro` for article pages.

### Changelog

Sources:

- `src/pages/changelog.astro`
- `src/content/changelog/`

The changelog renders versioned MDX entries from the `changelog` collection.

### About

Source: `src/pages/about.astro`

The about page explains the project story, values, and template philosophy.

### Contact

Source: `src/pages/contact.astro`

The contact page provides a polished static/demo form and supporting contact context.

### Elements

Source: `src/pages/elements.astro`

The elements page is the in-project reference for the component language and visual system.

### Auth Pages

Sources:

- `src/pages/sign-in.astro`
- `src/pages/sign-up.astro`
- `src/pages/signin.astro`
- `src/pages/signup.astro`

The auth pages are static template screens. They include local demo states but do not connect to a real authentication provider.

### 404

Source: `src/pages/404.astro`

The 404 page provides a branded not-found experience and routes visitors back into the site.

## Content Collections

Content schemas are defined in `src/content.config.js`.

### Blog Posts

Location: `src/content/post/`

Required frontmatter:

```yaml
title: string
description: string
publishDate: date
```

Optional frontmatter:

```yaml
read: number
tags: string[]
img: string
img_alt: string
featured: boolean
```

Current posts:

- `introducing-ricofast`
- `customize-your-saas-site`
- `design-system-behind-ricofast`
- `launching-v1-0`

### Changelog Entries

Location: `src/content/changelog/`

Required frontmatter:

```yaml
title: string
publishDate: date
```

Optional frontmatter:

```yaml
version: string
description: string
```

Current entries:

- `v1.0.0-stable-release.mdx`
- `v0.3.0-content-system.mdx`
- `v0.2.0-design-polish.mdx`
- `v0.1.0-initial-preview.mdx`

## Component System

Current component groups:

```text
src/components/
  cards/
  elements/
  home/
  sections/
  ui/
  widgets/
```

Core reusable components:

- `Header.astro`
- `Footer.astro`
- `HeroSection.astro`
- `Pricing.astro`
- `FAQ.astro`
- `BlogSection.astro`
- `Button.astro`
- `Badge.astro`
- `AccordionItem.astro`
- `PricingToggle.astro`
- `BrowserFrame.astro`
- `Logo.astro`
- `BlogCard.astro`
- `TechStackCard.astro`
- `PageHeader.astro`
- `SectionHeader.astro`
- `Pagination.astro`
- `Toc.astro`
- `ToTop.astro`
- `TrackGa.astro`
- `OptimizedImage.astro`

## Design System

Primary sources:

- `src/styles/global.css`
- `docs/DESIGN.md`
- `tailwind.config.mjs`

Current design rules:

- Light-first interface with class-based dark mode.
- Primary blue and accent gold are the key brand colors.
- `Instrument Serif` is used for display headings.
- `Inter` is used for body text and UI text.
- `--max-screen` is `1200px`.
- `--inner-screen` is `800px`.
- Motion should be subtle and respect `prefers-reduced-motion`.
- Reusable page containers use `.site-container` and `.inner-container`.

## SEO And Analytics

Current implementation:

- `src/layouts/Meta.astro`
- `src/components/widgets/Meta.astro`
- `src/components/widgets/TrackGa.astro`
- `src/pages/rss.xml.js`
- `@astrojs/sitemap`
- `public/robots.txt`
- `public/og.jpg`

Environment variables:

```env
PUBLIC_SITE_URL=https://your-domain.com
PUBLIC_GA4_ID=
PUBLIC_UMAMI_ID=
```

`PUBLIC_GA4_ID` and `PUBLIC_UMAMI_ID` are optional.

## Project Structure

```text
docs/
  DESIGN.md
  PLAN.md
  PRD.md
  plan/
public/
  assets/
  rico/
  favicon.png
  og.jpg
  robots.txt
src/
  assets/js/main.js
  collections/
  components/
  config/site.js
  content/
  content.config.js
  layouts/
  pages/
  styles/
astro.config.mjs
package.json
tailwind.config.mjs
```

## Build And QA

Supported commands:

```bash
pnpm dev
pnpm build
pnpm preview
pnpm check
pnpm astro
```

Release quality is verified by:

- `pnpm build`
- `pnpm check`
- Manual route review for all pages listed in this PRD.
- Manual light/dark mode review.
- Responsive review at mobile, tablet, and desktop widths.
- Sitemap and RSS output review after setting `PUBLIC_SITE_URL`.

## Related Docs

- `docs/DESIGN.md` - design system.
- `docs/PLAN.md` - concise project overview.
- `docs/plan/` - page-level implementation notes.
- `README.md` and `README-zh.md` - user-facing setup docs.
