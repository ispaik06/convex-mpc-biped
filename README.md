# Convex MPC for Bipedal Humanoid Locomotion — portfolio article

A fully static, single-page technical article about the
[convex-mpc-biped](https://github.com/ispaik06/convex-mpc-biped) project.

## What's here

```
index.html          <- the finished page (self-contained; needs only assets/)
assets/
  katex/            <- KaTeX CSS + fonts (local, no CDN)
  media/            <- MP4 clips (converted from repo GIFs), robot renders, dashboard shots
  plots/            <- MPC debugging plots
src/
  sections/*.html   <- article source, one file per section, TeX math in $...$ / $$...$$
  template.html     <- page shell (head, TOC slot, theme toggle, scroll-spy JS)
  style.css         <- all styling (light/dark via CSS variables)
build.mjs           <- build script: KaTeX pre-render + TOC generation -> index.html
```

Math is **pre-rendered at build time** with KaTeX, so the published page needs no
JavaScript for math and never depends on a CDN — equations cannot break.

## Deploy

The deployable unit is `index.html` + `assets/`. Everything else is source.

- **Netlify**: drag-and-drop the folder (or `netlify deploy`); `node_modules/` and `src/`
  are harmless but can be excluded.
- **GitHub Pages**: copy `index.html` + `assets/` into any Pages branch/folder.

## Rebuild after editing

Edit the section files in `src/sections/`, then:

```bash
npm install   # first time only (installs katex)
node build.mjs
```

The build fails loudly on any KaTeX error and reports leftover unrendered `$$` markers.

## Porting to the Jekyll (Chirpy) blog later

The section sources are plain HTML with TeX math — convert to a post by concatenating
`src/sections/*.html` into a Markdown file and letting the blog's MathJax/KaTeX render
the `$...$` delimiters. Media under `assets/` can be copied to the blog's asset path as-is.
