# Navigation And Footer

Sources:

- `src/collections/menu.json`
- `src/components/sections/Header.astro`
- `src/components/sections/Footer.astro`
- `src/config/site.js`

## Header

The header reads navigation items from `src/collections/menu.json`.

Current structure:

- Home
- Pages dropdown
  - Features
  - Pricing
  - About
  - Blog
  - Changelog
  - Sign in
  - Sign up
  - 404
- Elements
- Contact

Header behavior is supported by `src/assets/js/main.js`:

- Sticky style after scroll.
- Mobile menu open/close.
- Active route styling.
- Dark mode toggle.

## Footer

Footer links are defined inside `src/components/sections/Footer.astro`.

Current groups:

- Product
- Resources
- Company
- Legal

Footer also includes:

- Logo.
- Short product description.
- Version badge.
- Built with Astro & Tailwind label.
- Social links from `src/config/site.js`.
- Back-to-top widget.

## QA

- Verify dropdown behavior on desktop.
- Verify mobile menu open/close.
- Verify active nav item styles.
- Verify footer external links open correctly.
- Verify social icons render in light and dark modes.
