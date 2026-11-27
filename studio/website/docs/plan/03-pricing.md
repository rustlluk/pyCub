# Pricing Page

Sources:

- `src/pages/pricing.astro`
- `src/components/sections/Pricing.astro`
- `src/components/ui/PricingToggle.astro`

## Purpose

The pricing page demonstrates a realistic SaaS pricing layout that template users can replace with their own plans.

## Implemented Content

- Page header.
- Demo pricing note.
- Free, Pro, and Enterprise cards.
- Monthly/yearly pricing toggle.
- Feature comparison content.
- Pricing FAQ.
- Bottom CTA.

## Behavior

`PricingToggle.astro` dispatches a custom event consumed by `Pricing.astro`. Prices, suffixes, and plan descriptions update locally without a backend.

## QA

- Verify monthly/yearly state changes update every plan.
- Verify animation respects reduced-motion settings.
- Verify plan cards stay readable on mobile.
- Verify dark-mode card borders and text contrast.
