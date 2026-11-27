# Component Map

Source directory: `src/components/`

## Current Groups

| Group | Purpose |
| --- | --- |
| `cards/` | Repeated card components such as blog and tech-stack cards |
| `elements/` | Generic structural elements such as page and section headers |
| `home/` | Home-page-specific hero component |
| `sections/` | Larger page sections such as header, footer, pricing, FAQ, blog section |
| `ui/` | Small reusable UI primitives |
| `widgets/` | Page utilities such as table of contents, pagination, analytics, and back-to-top |

## Core Components

- `cards/BlogCard.astro`
- `cards/TechStackCard.astro`
- `elements/PageHeader.astro`
- `elements/SectionHeader.astro`
- `elements/SeparatorLine.astro`
- `home/HeroSection.astro`
- `sections/Header.astro`
- `sections/Footer.astro`
- `sections/Pricing.astro`
- `sections/FAQ.astro`
- `sections/BlogSection.astro`
- `ui/AccordionItem.astro`
- `ui/AnimatedText.astro`
- `ui/Badge.astro`
- `ui/BrowserFrame.astro`
- `ui/Button.astro`
- `ui/Logo.astro`
- `ui/PricingToggle.astro`
- `ui/TopBg.astro`
- `widgets/ActionBar.astro`
- `widgets/Meta.astro`
- `widgets/OptimizedImage.astro`
- `widgets/Pagination.astro`
- `widgets/Toc.astro`
- `widgets/ToTop.astro`
- `widgets/TrackGa.astro`

## Rules

- Use existing components before adding new ones.
- Keep props simple and aligned with current usage.
- Preserve dark-mode styles.
- Use tokens from `global.css`.
- Use Lucide icons through `@lucide/astro` when available.
