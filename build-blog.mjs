// Builds the Jekyll (Chirpy) blog-post version of the article.
//
//   node build-blog.mjs
//
// Output: _posts/2026-07-13-Convex_MPC_Bipedal_Humanoid_Locomotion.md in the blog repo.
// Same pre-rendered KaTeX pipeline as build.mjs; body is emitted as raw HTML inside
// a {% raw %} block so Liquid/kramdown never touch it. Chirpy supplies typography,
// headings, and its own TOC (tocbot reads the h2/h3 ids).

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";
import { transformMath, autoLinkSectionRefs, GUARDS } from "./src/lib.mjs";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const SECTIONS_DIR = path.join(ROOT, "src", "sections");
const BLOG_REPO = path.join(os.homedir(), "Documents", "Blog Archive", "ispaik06.github.io");
const POST_NAME = "2026-07-13-Convex_MPC_Bipedal_Humanoid_Locomotion.md";
const ASSET_BASE = "/assets/img/posts/convex-mpc";

const sectionFiles = fs.readdirSync(SECTIONS_DIR).filter((f) => f.endsWith(".html")).sort();

let body = sectionFiles
  .map((f) => fs.readFileSync(path.join(SECTIONS_DIR, f), "utf8"))
  .join("\n\n");

// --- blog-specific source adjustments (before math rendering) ---------------

// Title/eyebrow/subtitle come from the post front matter; drop them from the body.
body = body.replace(/<div class="eyebrow">[\s\S]*?<\/div>\s*/, "");
body = body.replace(/<h1>[\s\S]*?<\/h1>\s*/, "");
body = body.replace(/<p class="subtitle">[\s\S]*?<\/p>\s*/, "");

// Rewrite asset paths to the blog's asset directory.
body = body.replaceAll('src="assets/', `src="${ASSET_BASE}/`);

// --- render ------------------------------------------------------------------

body = transformMath(body);
body = autoLinkSectionRefs(body);

const postCss = fs.readFileSync(path.join(ROOT, "src", "blog-post.css"), "utf8");

const frontMatter = `---
title: "Convex MPC for Bipedal Humanoid Locomotion"
description: "Extending the MIT Cheetah 3 convex MPC to CoP-constrained humanoid walking — SRB modeling, 6D contact-wrench optimization, swing-foot planning, solver engineering, and reproducible MPC debugging in MuJoCo."
date: 2026-07-13 00:30:00 +09:00
categories: [Robotics, Legged Locomotion]
tags: [MPC, Convex Optimization, MuJoCo, Humanoid, Legged Robotics, OSQP, C++]
math: false
image:
  path: ${ASSET_BASE}/preview.jpg
  alt: MIT Humanoid walking under the convex MPC in MuJoCo
---
`;

const post = `${frontMatter}
<link rel="stylesheet" href="${ASSET_BASE}/katex/katex.min.css">

<style>
${postCss}
</style>

{% raw %}
<div class="cmpc">

${body}

</div>
{% endraw %}
`;

const outPath = path.join(BLOG_REPO, "_posts", POST_NAME);
fs.writeFileSync(outPath, post);

const leftover = (post.match(/\$\$/g) || []).length;
const guards = post.includes(GUARDS.GUARD_OPEN) ? "YES" : "no";
console.log(`${outPath} written (${(post.length / 1024).toFixed(0)} KB). Leftover $$: ${leftover}, leftover guards: ${guards}`);
