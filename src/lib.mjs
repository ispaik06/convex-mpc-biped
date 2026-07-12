// Shared rendering helpers for the article builds (standalone page + blog port).
import katex from "katex";

// Explicit escapes — private-use-area chars that cannot appear in prose.
const GUARD_OPEN = "\uE000";
const GUARD_CLOSE = "\uE001";

export function escapeHtml(s) {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

export function renderMath(tex, displayMode) {
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

// Replace math outside of <pre>/<code> blocks.
// Guard sentinels are Unicode private-use chars, so they cannot collide with prose.
export function transformMath(html) {
  const guards = [];
  html = html.replace(/<pre[\s\S]*?<\/pre>|<code[\s\S]*?<\/code>/g, (m) => {
    guards.push(m);
    return GUARD_OPEN + (guards.length - 1) + GUARD_CLOSE;
  });

  html = html.replace(/\$\$([\s\S]+?)\$\$/g, (_, tex) => {
    return `<div class="eq">${renderMath(tex.trim(), true)}</div>`;
  });

  html = html.replace(/\$([^$\n]+?)\$/g, (_, tex) => renderMath(tex.trim(), false));

  html = html.replace(new RegExp(`${GUARD_OPEN}(\\d+)${GUARD_CLOSE}`, "g"), (_, i) => guards[Number(i)]);
  return html;
}

// ---------------------------------------------------------------------------
// Section-reference auto-linking: turns plain-text "§5" / "§4.3" into anchors.
// Already-linked references (inside <a>...</a>) are left untouched.
// ---------------------------------------------------------------------------

const SECTION_IDS = {
  1: "introduction",
  2: "architecture",
  3: "srb-model",
  4: "convex-mpc",
  5: "constraints",
  6: "reference-trajectory",
  7: "gait",
  8: "swing",
  9: "torque",
  10: "solver",
  11: "robot-independent",
  12: "debugging",
  13: "results",
  14: "limitations",
  15: "closing",
  16: "references",
};

const SUBSECTION_IDS = {
  "1.1": "from-quadruped-to-biped", "1.2": "scope",
  "2.1": "runtime-loop",
  "3.1": "state-input", "3.2": "frames", "3.3": "orientation-dynamics", "3.4": "continuous-dynamics",
  "4.1": "discretization", "4.2": "condensed-qp", "4.3": "cost-transform", "4.4": "yaw-policy",
  "5.1": "friction-pyramid", "5.2": "cop", "5.3": "torsional", "5.4": "foot-template",
  "5.5": "yaw-rotation", "5.6": "equality-constraints",
  "6.1": "seed-policy", "6.2": "yaw-gating", "6.3": "planar-propagation", "6.4": "lever-arms",
  "7.1": "phase", "7.2": "horizon-constraints", "7.3": "contact-estimation",
  "7.4": "early-contact", "7.5": "horizon-override", "7.6": "stance-yaw-hold",
  "8.1": "touchdown-eq", "8.2": "stopping", "8.3": "swing-traj",
  "9.1": "stance-map", "9.2": "swing-osc", "9.3": "realizability",
  "10.1": "problem-size", "10.2": "sparsity", "10.3": "direct-fill",
  "12.1": "dbg-srb", "12.2": "dbg-wrench", "12.3": "dbg-probe", "12.4": "dbg-rh", "12.5": "dashboard",
  "13.1": "tuning-summary",
};

export function autoLinkSectionRefs(html) {
  // Split on existing anchors so we never nest a link inside a link.
  const parts = html.split(/(<a\b[\s\S]*?<\/a>)/g);
  let linked = 0;
  const out = parts.map((part, i) => {
    if (i % 2 === 1) return part; // an existing <a>...</a> chunk
    return part.replace(/§(\d+)(?:\.(\d+))?/g, (m, sec, sub) => {
      const id = sub != null ? (SUBSECTION_IDS[`${sec}.${sub}`] || SECTION_IDS[Number(sec)])
                             : SECTION_IDS[Number(sec)];
      if (!id) return m;
      linked += 1;
      return `<a href="#${id}">${m}</a>`;
    });
  });
  console.log(`auto-linked ${linked} plain section reference(s)`);
  return out.join("");
}

// Collect h2 headings -> TOC. Headings carry explicit ids in the sources.
export function buildToc(html) {
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

export const GUARDS = { GUARD_OPEN, GUARD_CLOSE };
