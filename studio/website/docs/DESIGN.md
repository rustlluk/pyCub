# RicoFast Design System


This document describes the current visual system implemented in the RicoFast project.

## Design Direction

RicoFast uses a calm SaaS/product-site aesthetic:

- Warm light canvas.
- Blue primary accent.
- Gold highlight accent.
- Editorial display typography.
- Dashed separators and restrained card borders.
- Subtle motion, never heavy animation.
- Full support for class-based dark mode.

## Source Files

- `src/styles/global.css` - tokens, global styles, dark mode variables.
- `tailwind.config.mjs` - Tailwind content scanning and dark mode strategy.
- `src/layouts/Layout.astro` - global layout, CSS imports, dark-mode boot script.
- `src/assets/js/main.js` - dark-mode toggle, sticky header, mobile menu, AOS initialization.

## Color Tokens

Defined in `src/styles/global.css`.

### Brand

| Token | Value | Usage |
| --- | --- | --- |
| `--color-primary` | `#2d6dc3` | Brand color, links, CTAs, headings |
| `--color-primary-strong` | `#0066ff` | Hover and emphasis |
| `--color-primary-light` | `#8fb9ff` | Light accents |
| `--color-accent` | `#fad13b` | Badges, highlights |
| `--color-accent-light` | `#faeb75` | Softer accent states |

### Background

| Token | Value | Usage |
| --- | --- | --- |
| `--color-bg-primary` | `#fdfaf5` | Light page canvas |
| `--color-bg-secondary` | `#fff` | Cards and panels |
| `--color-bg-primary-light` | `#faf9f5` | Article surfaces |
| `--color-bg-primary-deep` | `#fefcf4` | Warm nested surfaces |
| `--color-bg-primary-dark` | `#0b1220` | Dark page canvas |
| `--color-bg-secondary-dark` | `#0f1b2d` | Dark cards and panels |

### Text

| Token | Value | Usage |
| --- | --- | --- |
| `--color-text-primary` | `#2d6dc3` | Light-mode headings |
| `--color-text-secondary` | `#3f4a5a` | Light-mode body text |
| `--color-text-tertiary` | `#7a6550` | Muted metadata |
| `--color-text-primary-dark` | `#3884eb` | Dark-mode headings |
| `--color-text-secondary-dark` | `#c5cedb` | Dark-mode body text |
| `--color-text-tertiary-dark` | `#9bb3d7` | Dark-mode muted metadata |

### Neutral Scale

The project defines `--color-neutral-50` through `--color-neutral-950` for borders, body text, muted labels, placeholders, and dark surfaces.

## Typography

Fonts are imported in `src/styles/global.css`.

| Token | Font | Usage |
| --- | --- | --- |
| `--font-brand` | Instrument Serif | Display headings, hero titles, large section headings |
| `--font-sans` | Inter | Body text, UI labels, navigation, buttons |
| `--font-body` | Inter | Body text |

Rules:

- Use `font-brand` only for display text.
- Use `font-sans` for UI and long-form content.
- Do not use negative letter spacing as a default style.
- Keep compact UI headings smaller than hero headings.

## Layout Tokens

| Token/Class | Value | Usage |
| --- | --- | --- |
| `--max-screen` | `1200px` | Main site width |
| `--inner-screen` | `800px` | Articles and narrow content |
| `.site-container` | max width + horizontal padding | Page sections |
| `.inner-container` | inner max width + padding | Narrow content |

## Dark Mode

Dark mode is class-based.

- Tailwind config: `darkMode: "class"`.
- Initial state is applied inline in `Layout.astro` before page paint.
- Preference is stored in `localStorage` under `dark_mode`.
- Toggle behavior lives in `src/assets/js/main.js`.
- Dark color overrides live in `html.dark` inside `global.css`.

## Motion

Current motion system:

- AOS is imported globally and initialized in `src/assets/js/main.js`.
- Custom AOS behavior lives in `src/styles/aos-custom.css`.
- Component-level CSS animations are used for product previews and pricing motion.
- Motion should respect `prefers-reduced-motion`.

Common attributes:

```html
data-aos="fade-up-xs"
data-aos-once="true"
```

## Core Components

| Component | Path | Role |
| --- | --- | --- |
| Header | `src/components/sections/Header.astro` | Main navigation |
| Footer | `src/components/sections/Footer.astro` | Footer navigation and social links |
| HeroSection | `src/components/home/HeroSection.astro` | Home hero |
| Pricing | `src/components/sections/Pricing.astro` | Pricing cards and billing toggle |
| FAQ | `src/components/sections/FAQ.astro` | Accordion FAQ section |
| BlogSection | `src/components/sections/BlogSection.astro` | Blog previews and lists |
| Button | `src/components/ui/Button.astro` | CTA and link buttons |
| Badge | `src/components/ui/Badge.astro` | Small labels |
| AccordionItem | `src/components/ui/AccordionItem.astro` | FAQ item |
| PricingToggle | `src/components/ui/PricingToggle.astro` | Monthly/yearly toggle |
| BrowserFrame | `src/components/ui/BrowserFrame.astro` | Browser-style product frame |
| Logo | `src/components/ui/Logo.astro` | Brand mark |
| PageHeader | `src/components/elements/PageHeader.astro` | Page-level header |
| SectionHeader | `src/components/elements/SectionHeader.astro` | Section title and description |
| BlogCard | `src/components/cards/BlogCard.astro` | Blog listing card |
| TechStackCard | `src/components/cards/TechStackCard.astro` | Technology grid |

## UI Rules

- Use existing components before creating new ones.
- Use Lucide icons via `@lucide/astro` when an icon is needed.
- Use `Button.astro` for primary and secondary CTAs.
- Use `BrowserFrame.astro` for browser or product-preview mockups.
- Keep cards restrained: subtle borders, low shadow, clean spacing.
- Maintain light and dark mode styles for every new surface.
- Prefer token values over one-off hex colors.
- Keep form controls accessible with labels, focus states, and clear success/error states.

## Page Design Patterns

### Marketing Sections

Use:

- `.site-container`
- `SectionHeader`
- responsive grids
- dashed separators when separating major areas

### Article Pages

Use:

- `PostLayout.astro`
- `src/styles/article.css`
- `src/styles/article-enhancements.css`
- optional `Toc.astro`

### Auth Pages

Use:

- centered form card
- brand mark
- clear demo notice
- provider buttons
- local success state
- dark-mode-safe input styling

## Assets

Current asset groups:

- `public/og.jpg`
- `public/favicon.png`
- `public/assets/logo.png`
- `public/assets/icon/*`
- `public/assets/folder/*`
- `public/assets/stack/*`
- `public/assets/blog/cover.jpg`
- `public/assets/article-cover.jpg`
- `public/rico/*`

Replace these when adapting RicoFast to a real product.
