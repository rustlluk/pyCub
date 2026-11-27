# RicoFast — 一款设计感强的 Astro SaaS 模板

> [English](README.md)  | 中文文档



一个以设计驱动的开源 SaaS 网站模板，基于 Astro 6 + Tailwind CSS v4 构建。专为独立开发者、AI/开发工具团队、开源项目维护者打造，让你拥有有设计感的产品站点。

![preview](/docs/screenshot.jpeg)


![Astro](https://img.shields.io/badge/Astro-6.4.2-FF5D01?logo=astro&logoColor=white)
![Vite](https://img.shields.io/badge/Vite-7.3.3-646CFF?logo=vite&logoColor=white)
![Tailwind CSS](https://img.shields.io/badge/Tailwind-4.1.14-38B2AC?logo=tailwind-css&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue.svg)

**链接：** [在线演示](https://ricofast.pages.dev/) · [GitHub](https://github.com/ricocc/ricoui-saas-template) · [Changelog](https://ricofast.pages.dev/changelog) · [设计参考](https://ricofast.pages.dev/elements)

---

## 为什么是 RicoFast

大多数开源 SaaS 模板分两类：要么是骨架级 starter（剩下 90% 你得自己拼）；要么是大而全的模板包，它们主打的是功能全面，设计并不是这类模板的长项。 而 RicoFast 则是侧重设计和视觉，专注于有规范文档的设计，让 AI 准确识别设计系统，生产出视觉和设计优秀的页面。

## 特性

- **11 个生产就绪页面** — Home（12 个 section）、Features、Pricing、Blog、Changelog、About、Contact、Sign in、Sign up、Elements、404
- **15+ 可复用 section 组件** — Hero、FeatureGrid、FeatureDetail、Pricing cards、Comparison table、FAQ、Steps、Tech stack、Use cases、FinalCTA 等
- **Auth 页面模板** — 登录 / 注册各含两种布局变体（分屏和居中卡片），接入你的 auth provider 即可使用
- **MDX 驱动的 Blog & Changelog** — 基于 Astro 6 Content Layer API 和 `astro/loaders`
- **内置暗黑模式** — class-based、无切换闪烁、对比度审核过
- **Token 驱动的设计系统** — ~30 个 CSS 自定义属性，全部文档化于 `docs/DESIGN.md`
- **移动优先响应式** — 375 / 768 / 1024 / 1440 全断点测试
- **克制的动效** — AOS 滚动入场 + motion.js 编排，遵循 `prefers-reduced-motion`
- **Lighthouse 95+** — Performance、Accessibility、Best Practices、SEO 开箱即得
- **SEO 就绪** — Open Graph、Twitter cards、sitemap、RSS feed
- **TypeScript** — 每次 build 都跑 `astro check`
- **MIT 协议** — 个人/商业项目均可使用，无需注明出处

## 技术栈

- **框架：** [Astro 6.4.2](https://astro.build)（Content Layer API）
- **构建工具：** [Vite 7.3](https://vite.dev/)
- **样式：** [Tailwind CSS v4.1](https://tailwindcss.com)（`@theme` token 系统）
- **内容：** MDX 通过 `@astrojs/mdx`
- **动画：** [AOS](https://michalsnik.github.io/aos/) + [motion.js](https://motion.dev/)
- **图标：** [Lucide](https://lucide.dev/)，通过 `@lucide/astro` 使用
- **类型检查：** TypeScript + `astro check`
- **代码规范：** [Biome](https://biomejs.dev/)

## 环境要求

- Node.js `>=22.12.0`
- pnpm `>=9`（本项目以 `pnpm-lock.yaml` 作为依赖锁定文件）

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/ricocc/ricoui-saas-template.git my-saas
cd my-saas

# 2. 安装依赖
pnpm install

# 3. 启动开发服务器（http://localhost:5200）
pnpm dev
```

### 环境变量

复制 `.env.example` 为 `.env`，并设置你的域名：

```env
PUBLIC_SITE_URL=https://your-domain.com
PUBLIC_GA4_ID=your-google-analytics-id  # 可选
PUBLIC_UMAMI_ID=your-umami-id            # 可选
```

如果未设置 `PUBLIC_SITE_URL`，build 会使用默认值——上线前请设置真实域名，以确保 sitemap、RSS、canonical URL 正常生成。

## 项目结构

```
├── docs/
│   └── DESIGN.md           # 设计 token、字号、间距规范 — 首读
├── public/
│   ├── assets/             # 图片、og.jpg 等
│   ├── favicon.png
│   └── robots.txt
├── src/
│   ├── pages/              # 路由 — index、features、pricing、blog/ 等
│   ├── layouts/            # Layout.astro（根）、PageLayout、PostLayout、Meta
│   ├── components/
│   │   ├── home/           # HeroSection
│   │   ├── sections/       # Header、Footer、FAQ、BlogSection
│   │   ├── cards/          # BlogCard
│   │   ├── elements/       # SectionHeader、PageHeader、SeparatorLine
│   │   ├── ui/             # Button、Badge、AccordionItem、PricingToggle、BrowserFrame、Logo
│   │   └── widgets/        # Toc、Pagination、ToTop、Meta、OptimizedImage、TrackGa
│   ├── collections/        # menu.json、social.json
│   ├── content/
│   │   ├── post/           # 博客 MDX 条目
│   │   └── changelog/      # 更新日志 MDX 条目
│   ├── config/site.js      # 站点配置 — 首先修改这里
│   ├── styles/global.css   # 设计 token（Tailwind v4 @theme）
│   └── content.config.js   # Content Layer schema
├── CLAUDE.md               # AI 编程助手的上下文
├── astro.config.mjs
└── package.json
```

## 页面一览

| 路由 | 用途 | 编辑位置 |
|------|------|----------|
| `/` | 营销首页（12 个 section） | `src/pages/index.astro` |
| `/features` | 6 个功能详情模块 | `src/pages/features.astro` |
| `/pricing` | 三档定价 + 对比表 + FAQ | `src/pages/pricing.astro` |
| `/blog` | MDX 文章列表 | `src/content/post/` |
| `/changelog` | 版本更新日志 | `src/content/changelog/` |
| `/about` | 关于 + 价值观 + 时间线 | `src/pages/about.astro` |
| `/contact` | 演示联系表单 | `src/pages/contact.astro` |
| `/sign-in` | 登录页 — 分屏布局 | `src/pages/sign-in.astro` |
| `/sign-up` | 注册页 — 分屏布局 | `src/pages/sign-up.astro` |
| `/elements` | 内部设计参考 | `src/pages/elements.astro` |
| `/404` | 404 页面 | `src/pages/404.astro` |

Auth 页面还包含居中卡片变体，位于 `/signin` 和 `/signup`。

## 自定义指南

90% 的定制工作集中在三个地方。完整教程见 [`/blog/customize-your-saas-site`](https://ricofast.pages.dev/blog/customize-your-saas-site)。

### 1. 品牌身份 — `src/config/site.js`

```js
export const siteConfig = {
  title: "你的 SaaS",
  author: "你的团队",
  url: "https://your-domain.com",
  mail: "hello@your-domain.com",
  meta: {
    title: "你的 SaaS — 一句话价值主张",
    description: "你的产品清晰描述。",
    keywords: "你的, 目标, 关键词",
    image: "/og.jpg",
  },
  social: {
    twitter: "https://x.com/your-handle",
    github: "https://github.com/your-org/your-repo",
  },
};
```

### 2. 设计 Token — `src/styles/global.css`

```css
@theme {
  --color-primary: #2d6dc3;          /* 品牌主色 */
  --color-primary-strong: #0066ff;   /* hover/强调 */
  --color-accent: #fad13b;            /* 徽章、高亮 */
  --color-bg-primary: #fdfaf5;        /* 页面底色 */
  --font-brand: "Instrument Serif", serif;
  --font-sans: "Inter", sans-serif;
}
```

完整 token 列表见 [`docs/DESIGN.md`](docs/DESIGN.md)。

### 3. 内容 — `src/content/`

博客与 changelog 的 frontmatter schema 定义在 `src/content.config.js`。在 `src/content/post/<slug>/index.mdx` 放新文件，build 会自动识别。

## 常用命令

```bash
pnpm dev           # 开发服务器 http://localhost:5200
pnpm build         # astro check && astro build → dist/
pnpm preview       # 预览生产 build
pnpm check         # biome lint + format
```

## 部署

build 是纯静态的，任何静态主机都可以：

| 平台 | Build 命令 | 输出目录 |
|------|-----------|---------|
| Cloudflare Pages | `pnpm build` | `dist/` |
| Vercel | `pnpm build` | `dist/` |
| Netlify | `pnpm build` | `dist` |
| GitHub Pages | `pnpm build` + Pages action | `dist` |



## 文档

- [`docs/DESIGN.md`](docs/DESIGN.md) — 设计 token、字号、间距、动效规范
- [`docs/plan/`](docs/plan/) — 各页面详细规范
- [`CLAUDE.md`](CLAUDE.md) — AI 编程助手的工作上下文

## 贡献

欢迎贡献。Bug 报告、功能建议、PR 流程见 [`CONTRIBUTING.md`](CONTRIBUTING.md)。

## 协议

[MIT](LICENSE) — fork 它、用它发产品、卖你在它之上构建的东西，都无需注明出处。


## 其他模板

-  **Blog Template** - 开源 :  [https://github.com/ricocc/public-portfolio-site](https://github.com/ricocc/public-portfolio-site)



-  **Portfolio Template** - 开源 :  [https://github.com/ricocc/ricoui-portfolio](https://github.com/ricocc/ricoui-portfolio)


---

## 关于作者

我是Rico，网页/UI设计师，热衷于做些有趣和创意的作品。拥有 UI/UX 设计工作经验，目前专注于网页设计和视觉落地，以及开发项目探索。

可以添加我的微信，交个朋友

<img src="https://ricoui.com/assets/wechat.png" alt="ricocc-wechat" width="280" height="auto" style="display:inline-block;margin:12px;">


我平时在博客 <a href="https://ricoui.com/" target="_blank">Rico's Blog</a> 更新内容。也可以关注我的小红书 [@Rico的设计漫想](https://www.xiaohongshu.com/user/profile/5f2b6903000000000101f51f) 和 推特 [@ricouii](https://x.com/ricouii).

如果 RicoFast 帮你 ship 了什么，[给个 Star](https://github.com/ricocc/ricoui-saas-template) 就是最简单的反馈。



## 💜 支持作者

如果觉得有所帮助的话，一点点支持就可以大大激励创作者的热情，感谢！

<img src="public/rico/wechat-qr.jpg" alt="ricocc-wechat" width="280" height="auto" style="display:inline-block;margin:12px;">


<br/>

<a href="https://ko-fi.com/T6T817U4KZ" target="_blank"><img height="40" src="https://storage.ko-fi.com/cdn/kofi2.png?v=6" alt="Buy Me a Coffee at ko-fi.com" /></a>


---

⭐ 如果这个模板帮你节省了时间，请点一个 Star。
