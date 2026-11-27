# Blog And Changelog

Sources:

- `src/content.config.js`
- `src/content/post/`
- `src/content/changelog/`
- `src/pages/blog/index.astro`
- `src/pages/blog/[...slug].astro`
- `src/pages/blog/page/[page].astro`
- `src/pages/changelog.astro`
- `src/pages/rss.xml.js`

## Blog

The blog uses the `post` collection. Posts are MDX files stored under `src/content/post/<slug>/index.mdx`.

Current posts:

- `introducing-ricofast`
- `customize-your-saas-site`
- `design-system-behind-ricofast`
- `launching-v1-0`

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

## Changelog

The changelog uses the `changelog` collection. Entries are MDX files stored under `src/content/changelog/`.

Current entries:

- `v1.0.0-stable-release.mdx`
- `v0.3.0-content-system.mdx`
- `v0.2.0-design-polish.mdx`
- `v0.1.0-initial-preview.mdx`

## QA

- Verify blog index renders all posts.
- Verify pagination route works.
- Verify article pages render MDX content and metadata.
- Verify changelog entries sort newest first.
- Verify `/rss.xml` includes blog posts.
