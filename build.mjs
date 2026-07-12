// Build script: concatenates src/sections/*.html, pre-renders TeX math with KaTeX,
// auto-generates the table of contents, and emits a fully static index.html.
//
//   node build.mjs
//
// Math delimiters in section sources:
//   $$ ... $$   -> display math (katex displayMode)
//   $ ... $     -> inline math
// Rendered output requires no JavaScript for math; only assets/katex/katex.min.css.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import katex from "katex";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const SRC = path.join(ROOT, "src");
const SECTIONS_DIR = path.join(SRC, "sections");

const GUARD_OPEN = "";
const GUARD_CLOSE = "";

function renderMath(tex, displayMode) {
  try {
    return katex.renderToString(tex, {
      displayMode,
      throwOnError: true,
      strict: "ignore",
      trust: false,
    });
  } catch (err) {
    console.error(`\nKaTeX error (${displayMode ? "display" : "inline"}):\n  ${tex}\n  ${err.message}`);
    process.exitCode = 1;
    return `<code class="katex-error">${escapeHtml(tex)}</code>`;
  }
}

function escapeHtml(s) {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

// Replace math outside of <pre>/<code> blocks.
// Guard sentinels are Unicode private-use chars, so they cannot collide with prose.
function transformMath(html) {
  const guards = [];
  html = html.replace(/<pre[\s\S]*?<\/pre>|<code[\s\S]*?<\/code>/g, (m) => {
    guards.push(m);
    return GUARD_OPEN + (guards.length - 1) + GUARD_CLOSE;
  });

  // Display math first.
  html = html.replace(/\$\$([\s\S]+?)\$\$/g, (_, tex) => {
    return `<div class="eq">${renderMath(tex.trim(), true)}</div>`;
  });

  // Inline math. Dollar signs are reserved for math in the sources.
  html = html.replace(/\$([^$\n]+?)\$/g, (_, tex) => renderMath(tex.trim(), false));

  // Restore guarded segments.
  html = html.replace(new RegExp(`${GUARD_OPEN}(\\d+)${GUARD_CLOSE}`, "g"), (_, i) => guards[Number(i)]);
  return html;
}

// Collect h2 headings -> TOC. Headings carry explicit ids in the sources.
function buildToc(html) {
  const re = /<h2\s+id="([^"]+)"[^>]*>([\s\S]*?)<\/h2>/g;
  const items = [];
  let m;
  while ((m = re.exec(html)) !== null) {
    const text = m[2].replace(/<[^>]+>/g, " ").replace(/\s+/g, " ").trim();
    items.push({ id: m[1], text });
  }
  let toc = `<nav class="toc" id="toc" aria-label="Table of contents">\n<div class="toc-title">Contents</div>\n<ol>\n`;
  for (const it of items) {
    toc += `<li><a href="#${it.id}">${it.text}</a></li>\n`;
  }
  toc += "</ol>\n</nav>";
  return toc;
}

const sectionFiles = fs
  .readdirSync(SECTIONS_DIR)
  .filter((f) => f.endsWith(".html"))
  .sort();

console.log("Sections:", sectionFiles.join(", "));

let body = sectionFiles
  .map((f) => fs.readFileSync(path.join(SECTIONS_DIR, f), "utf8"))
  .join("\n\n");

body = transformMath(body);
const toc = buildToc(body);

const css = fs.readFileSync(path.join(SRC, "style.css"), "utf8");
const template = fs.readFileSync(path.join(SRC, "template.html"), "utf8");

const out = template
  .replace("<!--STYLE-->", () => `<style>\n${css}\n</style>`)
  .replace("<!--TOC-->", () => toc)
  .replace("<!--BODY-->", () => body);

fs.writeFileSync(path.join(ROOT, "index.html"), out);

const leftoverDisplay = (out.match(/\$\$/g) || []).length;
const leftoverGuards = out.includes(GUARD_OPEN) ? "YES" : "no";
console.log(`index.html written (${(out.length / 1024).toFixed(0)} KB). Leftover $$: ${leftoverDisplay}, leftover guards: ${leftoverGuards}`);
