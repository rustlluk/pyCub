# Home Page

Source: `src/pages/index.astro`

## Purpose

The home page introduces RicoFast as a complete SaaS website template and routes visitors toward the template repository, feature details, pricing examples, and content.

## Implemented Sections

1. Hero: `src/components/home/HeroSection.astro`
2. Social proof/logo strip
3. Problem cards
4. Solution statement
5. Feature grid
6. Browser-style product preview
7. Use cases
8. Tech stack
9. Pricing preview: `src/components/sections/Pricing.astro`
10. FAQ: `src/components/sections/FAQ.astro`
11. Latest articles: `src/components/sections/BlogSection.astro`
12. Final CTA

## Content Notes

- Main promise: ship a polished SaaS site in days, not weeks.
- Audience: indie makers, AI tools, developer tools, design tools, micro SaaS, open-source projects, analytics products.
- Pricing on the home page is demo content for users to replace.

## Key Components

- `Layout.astro`
- `HeroSection.astro`
- `SectionHeader.astro`
- `BrowserFrame.astro`
- `TechStackCard.astro`
- `Pricing.astro`
- `FAQ.astro`
- `BlogSection.astro`
- `Button.astro`

## QA

- Check hero and browser preview in light/dark mode.
- Check responsive stacking at 375px and 768px.
- Check pricing toggle interaction.
- Check FAQ accordion interaction.
- Check latest articles render from `src/content/post/`.
