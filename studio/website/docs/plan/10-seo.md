# SEO And Meta

Sources:

- `astro.config.mjs`
- `src/config/site.js`
- `src/layouts/Meta.astro`
- `src/components/widgets/Meta.astro`
- `src/components/widgets/TrackGa.astro`
- `src/pages/rss.xml.js`
- `public/robots.txt`
- `public/og.jpg`

## Current SEO Features

- Page-level title and description props through layouts.
- Site-level metadata in `src/config/site.js`.
- Open Graph image at `public/og.jpg`.
- RSS feed at `/rss.xml`.
- Sitemap integration through `@astrojs/sitemap`.
- Robots file in `public/robots.txt`.
- Optional analytics through public environment variables.

## Environment Variables

```env
PUBLIC_SITE_URL=https://your-domain.com
PUBLIC_GA4_ID=
PUBLIC_UMAMI_ID=
```

`PUBLIC_SITE_URL` should be set before deployment so canonical URLs, RSS, and sitemap output point to the production domain.

## QA

- Verify every page has a meaningful title.
- Verify every page has a meaningful description.
- Verify `/rss.xml` renders blog posts.
- Verify sitemap generation during production build.
- Verify `public/og.jpg` exists.
- Verify analytics scripts are omitted when IDs are empty.
