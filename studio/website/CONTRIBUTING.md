# Contributing to RicoFast

Thanks for considering a contribution! All forms of help are welcome — bug reports, feature ideas, documentation improvements, or code.

## Reporting bugs

If you spot a bug:

1. Search [Issues](https://github.com/ricocc/ricoui-saas-template/issues) to confirm it isn't already filed
2. Open a new issue and include:
   - A clear description of the problem
   - Steps to reproduce
   - Expected vs. actual behavior
   - Screenshots (if relevant)
   - Environment info (browser, OS, Node version)

## Suggesting features

If you have an idea for improvement:

1. Search existing issues to avoid duplicates
2. Open a new Feature Request describing:
   - What the feature does
   - The use case it solves
   - A possible implementation sketch (optional)

## Submitting code

1. **Fork the repo**

   ```bash
   git clone https://github.com/your-username/ricoui-saas-template.git
   cd ricoui-saas-template
   ```

2. **Create a branch**

   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b fix/your-bug-fix
   ```

3. **Install dependencies**

   ```bash
   pnpm install
   ```

4. **Make your changes**
   - Follow the conventions described in [`CLAUDE.md`](CLAUDE.md) and [`docs/DESIGN.md`](docs/DESIGN.md)
   - Run `pnpm check` to verify Biome lint passes
   - Run `pnpm build` to verify the production build succeeds
   - Test in both light and dark mode

5. **Commit using Conventional Commits**

   ```bash
   git add .
   git commit -m "feat: add BrowserFrame component"
   ```

   Common prefixes:
   - `feat:` new feature
   - `fix:` bug fix
   - `docs:` documentation only
   - `style:` formatting / styling
   - `refactor:` code restructuring
   - `perf:` performance improvement
   - `chore:` tooling, build, dependencies

6. **Push and open a Pull Request**

   ```bash
   git push origin feature/your-feature-name
   ```

   Then create a PR against `main` and fill in the description template.

## Code conventions

- **Astro / Content** — Use Astro v5 Content Layer (`glob` loader, `entry.id`, `render(entry)`, `import.meta.env`). See `CLAUDE.md` for the full list.
- **Styling** — Use design tokens from `src/styles/global.css`. Don't introduce ad-hoc colors or magic spacing values.
- **Dark mode** — Every component must work in both light and dark. Test with the toggle in the Header.
- **Animation** — Use AOS for scroll reveals (`data-aos-once="true"`). Reserve motion.js for orchestrated entrances.
- **TypeScript** — `pnpm build` runs `astro check`. Fix type errors before opening a PR.
- **No new dependencies** — without strong justification in the PR description. The template stays light by design.

## Development requirements

- Node.js >= 18
- pnpm (recommended), npm, or yarn
- A modern browser (Chrome, Firefox, Safari, Edge)

## Before opening a PR

- [ ] `pnpm check` passes (Biome lint)
- [ ] `pnpm build` succeeds with no type errors
- [ ] Tested locally in dev (`pnpm dev`)
- [ ] Works in both light and dark mode
- [ ] Mobile (375px) and desktop (1440px) layouts intact
- [ ] No broken imports or stale references after cleanup

## Questions?

- Browse [Issues](https://github.com/ricocc/ricoui-saas-template/issues)
- Open a discussion or new issue
- Email: hello@ricoui.com

Thanks again for helping make this better.
