#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import shared_memory


STATE_LABELS = (
    "roll",
    "pitch",
    "yaw",
    "pos_x",
    "pos_y",
    "pos_z",
    "omega_x",
    "omega_y",
    "omega_z",
    "vel_x",
    "vel_y",
    "vel_z",
)


def _chart(
    label: str,
    title: str,
    group: str,
    group_label: str,
    unit: str,
    color: str,
    *,
    scale: str = "auto",
    min_span: float = 1.0,
    precision: int = 3,
) -> dict[str, object]:
    return {
        "label": label,
        "title": title,
        "group": group,
        "group_label": group_label,
        "unit": unit,
        "color": color,
        "scale": scale,
        "min_span": min_span,
        "precision": precision,
    }


CHART_CONFIGS = (
    _chart("roll", "Roll", "pose", "Pose", "rad", "#7dd3fc", scale="symmetric", min_span=0.8),
    _chart("pitch", "Pitch", "pose", "Pose", "rad", "#4ade80", scale="symmetric", min_span=0.8),
    _chart("yaw", "Yaw", "pose", "Pose", "rad", "#f59e0b", scale="symmetric", min_span=1.0),
    _chart("pos_x", "Pos X", "pose", "Pose", "m", "#22c55e", scale="auto", min_span=0.5),
    _chart("pos_y", "Pos Y", "pose", "Pose", "m", "#f97316", scale="auto", min_span=0.5),
    _chart("pos_z", "Pos Z", "pose", "Pose", "m", "#60a5fa", scale="auto", min_span=0.4),
    _chart("omega_x", "Omega X", "motion", "Motion", "rad/s", "#f43f5e", scale="symmetric", min_span=0.8),
    _chart("omega_y", "Omega Y", "motion", "Motion", "rad/s", "#14b8a6", scale="symmetric", min_span=0.8),
    _chart("omega_z", "Omega Z", "motion", "Motion", "rad/s", "#eab308", scale="symmetric", min_span=0.8),
    _chart("vel_x", "Vel X", "motion", "Motion", "m/s", "#34d399", scale="symmetric", min_span=1.0),
    _chart("vel_y", "Vel Y", "motion", "Motion", "m/s", "#fb7185", scale="symmetric", min_span=1.0),
    _chart("vel_z", "Vel Z", "motion", "Motion", "m/s", "#38bdf8", scale="symmetric", min_span=1.0),
)

LAYOUT = struct.Struct("<QQd32s12dIIQQ")
LAYOUT_SIZE = LAYOUT.size
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8000
DEFAULT_SHM_NAME = os.environ.get("CONVEXMPC_SHM_NAME", "convexmpc_dashboard_state")
DEFAULT_WINDOW_SECONDS = 10.0
WINDOW_OPTIONS = (5, 10, 20, 30)

HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ConvexMPC Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #090d12;
      --bg-2: #0d1218;
      --panel: rgba(16, 21, 28, 0.94);
      --panel-2: rgba(21, 27, 35, 0.96);
      --border: rgba(148, 163, 184, 0.16);
      --border-strong: rgba(125, 211, 252, 0.34);
      --text: #edf4fb;
      --muted: #93a4b7;
      --accent: #7dd3fc;
      --good: #4ade80;
      --warn: #f59e0b;
      --bad: #ef4444;
      --shadow: 0 28px 60px rgba(0, 0, 0, 0.36);
    }
    * { box-sizing: border-box; }
    html, body { min-height: 100%; }
    body {
      margin: 0;
      color: var(--text);
      font-family: "Avenir Next", "SF Pro Text", "Segoe UI", sans-serif;
      background:
        radial-gradient(circle at 15% 12%, rgba(125, 211, 252, 0.15), transparent 26%),
        radial-gradient(circle at 82% 16%, rgba(74, 222, 128, 0.11), transparent 24%),
        radial-gradient(circle at 52% 100%, rgba(245, 158, 11, 0.10), transparent 34%),
        linear-gradient(180deg, #070b10, #0a0f14 44%, #090d12 100%);
    }
    body::before {
      content: "";
      position: fixed;
      inset: 0;
      pointer-events: none;
      background-image:
        linear-gradient(rgba(255, 255, 255, 0.025) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255, 255, 255, 0.025) 1px, transparent 1px);
      background-size: 72px 72px;
      mask-image: radial-gradient(circle at center, black 58%, transparent 100%);
      opacity: 0.34;
    }
    .shell {
      max-width: 1680px;
      margin: 0 auto;
      padding: 10px 14px 16px;
    }
    .hero {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      align-items: end;
      gap: 12px;
      margin-bottom: 8px;
    }
    .eyebrow {
      margin: 0 0 6px;
      color: var(--accent);
      text-transform: uppercase;
      letter-spacing: 0.18em;
      font-size: 11px;
      font-weight: 700;
    }
    h1 {
      margin: 0;
      font-size: clamp(22px, 2.45vw, 38px);
      line-height: 0.95;
      letter-spacing: -0.05em;
      font-weight: 780;
    }
    .subtitle {
      margin-top: 6px;
      color: var(--muted);
      line-height: 1.45;
      max-width: 78ch;
      font-size: 12px;
    }
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 9px 12px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.03);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: var(--warn);
      box-shadow: 0 0 0 0 rgba(245, 158, 11, 0.25);
    }
    .dot.live {
      background: var(--good);
      animation: pulse 1.8s ease-in-out infinite;
    }
    .dot.busy {
      background: var(--warn);
    }
    .dot.error {
      background: var(--bad);
      box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.25);
    }
    @keyframes pulse {
      0% { box-shadow: 0 0 0 0 rgba(74, 222, 128, 0.24); }
      70% { box-shadow: 0 0 0 11px rgba(74, 222, 128, 0); }
      100% { box-shadow: 0 0 0 0 rgba(74, 222, 128, 0); }
    }
    .toolbar {
      display: grid;
      grid-template-columns: minmax(0, 1fr);
      gap: 8px;
      align-items: center;
      margin-bottom: 8px;
    }
    .toolbar-left,
    .toolbar-right {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
    }
    .toolbar-right {
      display: none;
    }
    .segment {
      display: inline-flex;
      padding: 4px;
      border-radius: 16px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.03);
      backdrop-filter: blur(14px);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
    }
    .segment button,
    .nav-button {
      appearance: none;
      border: 0;
      cursor: pointer;
      color: var(--muted);
      background: transparent;
      font: inherit;
      font-size: 13px;
      font-weight: 650;
      padding: 10px 14px;
      border-radius: 12px;
      transition: transform 120ms ease, background 120ms ease, color 120ms ease, box-shadow 120ms ease;
    }
    .segment button:hover,
    .nav-button:hover {
      transform: translateY(-1px);
      color: var(--text);
    }
    .segment button.active {
      color: var(--text);
      background: linear-gradient(180deg, rgba(125, 211, 252, 0.18), rgba(125, 211, 252, 0.08));
      box-shadow: inset 0 0 0 1px rgba(125, 211, 252, 0.24);
    }
    .nav-button {
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.03);
      padding: 9px 12px;
    }
    .nav-button.active {
      color: var(--text);
      border-color: rgba(125, 211, 252, 0.26);
      background: rgba(125, 211, 252, 0.12);
    }
    .toolbar-note {
      padding: 9px 12px;
      border-radius: 14px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.03);
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
      max-width: 54ch;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(0, 1.48fr) minmax(260px, 0.52fr);
      gap: 12px;
      margin-bottom: 10px;
      align-items: start;
    }
    .panel {
      border: 1px solid var(--border);
      border-radius: 22px;
      background:
        linear-gradient(180deg, rgba(255, 255, 255, 0.035), transparent 18%),
        linear-gradient(180deg, rgba(21, 27, 35, 0.96), rgba(15, 20, 27, 0.96));
      box-shadow: var(--shadow);
      overflow: hidden;
    }
    .panel-head {
      padding: 8px 12px;
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      gap: 10px;
      border-bottom: 1px solid rgba(255, 255, 255, 0.06);
      background: linear-gradient(180deg, rgba(255, 255, 255, 0.04), transparent);
    }
    .panel-head-left,
    .panel-head-right {
      min-width: 0;
    }
    .panel-kicker {
      margin: 0 0 4px;
      color: var(--muted);
      font-size: 10px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.13em;
    }
    .panel-title {
      margin: 0;
      font-size: 17px;
      line-height: 1.05;
      letter-spacing: -0.03em;
      font-weight: 760;
    }
    .panel-sub {
      margin-top: 3px;
      color: var(--muted);
      font-size: 11px;
      line-height: 1.45;
    }
    .panel-badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 7px 10px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.03);
      border: 1px solid var(--border);
      color: var(--muted);
      font-size: 11px;
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }
    .panel-body {
      padding: 10px 12px 12px;
    }
    .focus-chart {
      height: 166px;
    }
    .focus-chart canvas,
    .chart-plot canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .focus-metrics {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin-top: 8px;
    }
    .metric,
    .side-card {
      border-radius: 16px;
      border: 1px solid rgba(255, 255, 255, 0.06);
      background: rgba(255, 255, 255, 0.03);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
    }
    .metric {
      padding: 8px 10px;
    }
    .metric-label,
    .side-label {
      color: var(--muted);
      font-size: 10px;
      text-transform: uppercase;
      letter-spacing: 0.11em;
      margin-bottom: 6px;
    }
    .metric-value,
    .side-value {
      font-size: 13px;
      font-variant-numeric: tabular-nums;
      line-height: 1.15;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .side-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 6px;
    }
    .side-card {
      padding: 8px 10px;
      min-height: 52px;
    }
    .grid-panel {
      margin-top: 8px;
    }
    .chart-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 8px;
    }
    .chart-grid[data-view="focus"] {
      grid-template-columns: minmax(0, 1fr);
    }
    .chart-card {
      position: relative;
      min-height: 156px;
      border-radius: 18px;
      border: 1px solid rgba(255, 255, 255, 0.06);
      background:
        radial-gradient(circle at top right, rgba(255, 255, 255, 0.04), transparent 35%),
        rgba(255, 255, 255, 0.03);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
      overflow: hidden;
      cursor: pointer;
      transition: transform 140ms ease, border-color 140ms ease, background 140ms ease, box-shadow 140ms ease;
    }
    .chart-grid[data-view="focus"] .chart-card {
      min-height: 210px;
    }
    .chart-card:hover {
      transform: translateY(-2px);
      border-color: rgba(125, 211, 252, 0.28);
      background:
        radial-gradient(circle at top right, rgba(125, 211, 252, 0.08), transparent 38%),
        rgba(255, 255, 255, 0.045);
    }
    .chart-card.active {
      border-color: rgba(125, 211, 252, 0.48);
      box-shadow:
        0 0 0 1px rgba(125, 211, 252, 0.16) inset,
        0 18px 34px rgba(0, 0, 0, 0.18);
    }
    .chart-card::after {
      content: "";
      position: absolute;
      inset: 0;
      pointer-events: none;
      background: linear-gradient(180deg, rgba(255, 255, 255, 0.03), transparent 34%);
    }
    .chart-head {
      position: relative;
      z-index: 1;
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      gap: 8px;
      padding: 8px 10px 4px;
    }
    .chart-title {
      font-size: 13px;
      font-weight: 760;
      letter-spacing: -0.02em;
    }
    .chart-meta {
      margin-top: 2px;
      color: var(--muted);
      font-size: 10px;
    }
    .chart-value {
      color: var(--accent);
      font-variant-numeric: tabular-nums;
      font-size: 12px;
      text-align: right;
    }
    .chart-plot {
      position: absolute;
      left: 0;
      right: 0;
      top: 38px;
      bottom: 20px;
      padding: 0 6px 0 0;
    }
    .chart-foot {
      position: absolute;
      left: 12px;
      right: 12px;
      bottom: 5px;
      display: flex;
      justify-content: space-between;
      gap: 8px;
      color: var(--muted);
      font-size: 9px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-variant-numeric: tabular-nums;
      z-index: 1;
    }
    .chart-foot span {
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    @media (max-width: 1240px) {
      .layout {
        grid-template-columns: 1fr;
      }
      .focus-metrics {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
      .side-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
    @media (max-width: 780px) {
      .hero,
      .toolbar {
        grid-template-columns: 1fr;
      }
      .toolbar-right {
        justify-content: flex-start;
      }
      .focus-chart {
        height: 220px;
      }
      .focus-metrics,
      .side-grid {
        grid-template-columns: 1fr;
      }
      .chart-grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header class="hero">
      <div>
        <p class="eyebrow">ConvexMPC real-time telemetry</p>
        <h1>Live state dashboard</h1>
        <div class="subtitle" id="hero-subtitle">
          Auto-scaled plots with stable grid ticks. Click any trace to promote it to the focus chart.
        </div>
      </div>
      <div class="status-badge">
        <span class="dot" id="status-dot"></span>
        <span id="status-text">connecting</span>
      </div>
    </header>

    <div class="toolbar">
      <div class="toolbar-left">
        <div class="segment" id="view-buttons" aria-label="view mode">
          <button type="button" class="active" data-view="all">All 12</button>
          <button type="button" data-view="pose">Pose</button>
          <button type="button" data-view="motion">Motion</button>
          <button type="button" data-view="focus">Focus</button>
        </div>
        <div class="segment" id="window-buttons" aria-label="window length">
          <button type="button" class="active" data-window="5">5s</button>
          <button type="button" data-window="10">10s</button>
          <button type="button" data-window="20">20s</button>
          <button type="button" data-window="30">30s</button>
        </div>
      </div>
      <div class="toolbar-right">
        <div class="toolbar-note" id="toolbar-note">
          Y-axis uses a nice-number grid. Symmetric traces stay zero-centered; pose traces expand and contract with hysteresis.
        </div>
      </div>
    </div>

    <section class="layout">
      <article class="panel">
        <div class="panel-head">
          <div class="panel-head-left">
            <div class="panel-kicker">Focus</div>
            <h2 class="panel-title" id="focus-title">Roll</h2>
            <div class="panel-sub" id="focus-subtitle">Pose • rad</div>
          </div>
          <div class="panel-head-right">
            <div class="panel-badge" id="focus-index">01 / 12</div>
            <div style="display:flex; gap:8px; margin-top:10px; justify-content:flex-end;">
              <button type="button" class="nav-button" id="focus-prev">Prev</button>
              <button type="button" class="nav-button" id="focus-next">Next</button>
            </div>
          </div>
        </div>
        <div class="panel-body">
          <div class="focus-chart">
            <canvas id="focus-canvas"></canvas>
          </div>
          <div class="focus-metrics">
            <div class="metric">
              <div class="metric-label">Current</div>
              <div class="metric-value" id="focus-current">--</div>
            </div>
            <div class="metric">
              <div class="metric-label">Span</div>
              <div class="metric-value" id="focus-span">--</div>
            </div>
            <div class="metric">
              <div class="metric-label">Grid Step</div>
              <div class="metric-value" id="focus-step">--</div>
            </div>
            <div class="metric">
              <div class="metric-label">Samples</div>
              <div class="metric-value" id="focus-samples">--</div>
            </div>
          </div>
        </div>
      </article>

      <aside class="panel">
        <div class="panel-head">
          <div class="panel-head-left">
            <div class="panel-kicker">Connection</div>
            <h2 class="panel-title">Telemetry</h2>
            <div class="panel-sub" id="connection-subtitle">
              Waiting for the controller to publish data.
            </div>
          </div>
          <div class="panel-head-right">
            <div class="panel-badge" id="sequence-text">seq --</div>
          </div>
        </div>
        <div class="panel-body">
          <div class="side-grid">
            <div class="side-card">
              <div class="side-label">Robot</div>
              <div class="side-value" id="robot-name">--</div>
            </div>
            <div class="side-card">
              <div class="side-label">Iteration</div>
              <div class="side-value" id="iteration">--</div>
            </div>
            <div class="side-card">
              <div class="side-label">Sim Time</div>
              <div class="side-value" id="sim-time">--</div>
            </div>
            <div class="side-card">
              <div class="side-label">Shared Memory</div>
              <div class="side-value" id="shm-name">--</div>
            </div>
            <div class="side-card">
              <div class="side-label">Layout Version</div>
              <div class="side-value" id="layout-version">--</div>
            </div>
            <div class="side-card">
              <div class="side-label">State Dim</div>
              <div class="side-value" id="state-dim">--</div>
            </div>
          </div>
        </div>
      </aside>
    </section>

    <section class="panel grid-panel">
      <div class="panel-head">
        <div class="panel-head-left">
          <div class="panel-kicker">Traces</div>
          <h2 class="panel-title" id="grid-title">All 12 channels</h2>
          <div class="panel-sub" id="grid-subtitle">
            Pose and motion states across a trailing 10 second window.
          </div>
        </div>
        <div class="panel-head-right">
          <div class="panel-badge" id="history-label">history --</div>
        </div>
      </div>
      <div class="panel-body">
        <div class="chart-grid" id="chart-grid" data-view="all"></div>
      </div>
    </section>
  </div>

  <script>
    const CHART_CONFIGS = %CHART_CONFIG%;
    const STATE_LABELS = %STATE_LABELS%;
    const CHART_ORDER = CHART_CONFIGS.map((config, index) => ({ config, index }));
    const DEFAULT_WINDOW_SECONDS = 10;
    const WINDOW_OPTIONS = [5, 10, 20, 30];
    const VIEW_LABELS = {
      all: "All 12 channels",
      pose: "Pose channels",
      motion: "Motion channels",
      focus: "Focus channel",
    };

    const appState = {
      view: "all",
      windowSeconds: DEFAULT_WINDOW_SECONDS,
      focusLabel: CHART_CONFIGS[0]?.label ?? "roll",
      latestSnapshot: null,
      lastSequence: null,
      history: [],
      chartRuntime: new Map(
        CHART_CONFIGS.map((config, index) => [
          config.label,
          {
            config,
            index,
            domain: null,
            card: null,
            canvas: null,
            valueNode: null,
            rangeNode: null,
            gridNode: null,
          },
        ])
      ),
      polling: false,
    };

    function escapeHtml(value) {
      return String(value)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;")
        .replaceAll("'", "&#39;");
    }

    function formatNumber(value, digits = 3) {
      if (value === null || value === undefined) {
        return "--";
      }
      if (!Number.isFinite(value)) {
        return String(value);
      }
      const rounded = Number(value).toFixed(digits);
      if (digits <= 0 || !rounded.includes(".")) {
        return rounded;
      }

      // Only trim fractional zeros. Keep integer zeros intact so 10 does not become 1.
      let text = rounded.replace(/(\.\d*?[1-9])0+$/, "$1");
      text = text.replace(/\.0+$/, "");
      if (text === "-0") {
        return "0";
      }
      return text;
    }

    function formatValue(value, unit, precision = 3) {
      if (value === null || value === undefined || !Number.isFinite(value)) {
        return "--";
      }
      return `${formatNumber(value, precision)} ${unit}`;
    }

    function hexToRgba(hex, alpha) {
      const clean = hex.replace("#", "");
      const r = parseInt(clean.slice(0, 2), 16);
      const g = parseInt(clean.slice(2, 4), 16);
      const b = parseInt(clean.slice(4, 6), 16);
      return `rgba(${r}, ${g}, ${b}, ${alpha})`;
    }

    function niceStep(range, targetTicks) {
      const safeRange = Math.max(Math.abs(range), 1e-9);
      const rough = safeRange / Math.max(targetTicks, 1);
      const exponent = Math.floor(Math.log10(rough));
      const fraction = rough / Math.pow(10, exponent);
      let niceFraction = 10;
      if (fraction <= 1) niceFraction = 1;
      else if (fraction <= 2) niceFraction = 2;
      else if (fraction <= 2.5) niceFraction = 2.5;
      else if (fraction <= 5) niceFraction = 5;
      return niceFraction * Math.pow(10, exponent);
    }

    function decimalsForStep(step) {
      if (!Number.isFinite(step) || step <= 0) {
        return 0;
      }
      if (step >= 1) {
        return 0;
      }
      return Math.min(6, Math.ceil(-Math.log10(step)) + 1);
    }

    function formatTick(value, step) {
      return formatNumber(value, decimalsForStep(step));
    }

    function buildTicks(min, max, step) {
      const ticks = [];
      if (!(step > 0)) {
        return ticks;
      }
      const start = Math.ceil((min - 1e-9) / step) * step;
      for (let value = start; value <= max + step * 0.5; value += step) {
        ticks.push(Number(value.toFixed(12)));
      }
      return ticks;
    }

    function visibleChartsForView() {
      if (appState.view === "pose") {
        return CHART_CONFIGS.filter((config) => config.group === "pose");
      }
      if (appState.view === "motion") {
        return CHART_CONFIGS.filter((config) => config.group === "motion");
      }
      if (appState.view === "focus") {
        return CHART_CONFIGS.filter((config) => config.label === appState.focusLabel);
      }
      return CHART_CONFIGS;
    }

    function getChartEntry(label) {
      return appState.chartRuntime.get(label) || null;
    }

    function ensureFocusVisible() {
      const visible = visibleChartsForView();
      if (!visible.some((config) => config.label === appState.focusLabel) && visible.length > 0) {
        appState.focusLabel = visible[0].label;
      }
    }

    function setView(view) {
      appState.view = view;
      ensureFocusVisible();
      updateButtonStates();
      render();
    }

    function setWindow(seconds) {
      appState.windowSeconds = seconds;
      updateButtonStates();
      pruneHistory();
      render();
    }

    function setFocus(label) {
      if (!CHART_CONFIGS.some((config) => config.label === label)) {
        return;
      }
      appState.focusLabel = label;
      ensureFocusVisible();
      updateButtonStates();
      render();
    }

    function stepFocus(delta) {
      const currentIndex = CHART_CONFIGS.findIndex((config) => config.label === appState.focusLabel);
      const nextIndex = (currentIndex + delta + CHART_CONFIGS.length) % CHART_CONFIGS.length;
      appState.focusLabel = CHART_CONFIGS[nextIndex].label;
      ensureFocusVisible();
      updateButtonStates();
      render();
    }

    function updateButtonStates() {
      document.querySelectorAll("#view-buttons button").forEach((button) => {
        button.classList.toggle("active", button.dataset.view === appState.view);
      });
      document.querySelectorAll("#window-buttons button").forEach((button) => {
        button.classList.toggle("active", Number(button.dataset.window) === appState.windowSeconds);
      });

      const visible = visibleChartsForView();
      document.getElementById("grid-title").textContent = VIEW_LABELS[appState.view] || "All 12 channels";
      document.getElementById("grid-subtitle").textContent =
        appState.view === "focus"
          ? "A single chart window with auto-scaled axes and stable tick spacing."
          : appState.view === "pose"
            ? "Orientation and position traces, grouped together for quick inspection."
            : appState.view === "motion"
              ? "Angular and linear velocities, centered around zero when appropriate."
              : "Pose and motion states across the trailing window.";
      document.getElementById("chart-grid").dataset.view = appState.view;
      document.getElementById("focus-index").textContent =
        `${String(CHART_CONFIGS.findIndex((config) => config.label === appState.focusLabel) + 1).padStart(2, "0")} / ${String(CHART_CONFIGS.length).padStart(2, "0")}`;
      document.getElementById("focus-prev").textContent = "Prev";
      document.getElementById("focus-next").textContent = "Next";

      visible.forEach((config) => {
        const entry = getChartEntry(config.label);
        if (entry?.card) {
          entry.card.style.display = "";
        }
      });
    }

    function buildChartCards() {
      const grid = document.getElementById("chart-grid");
      grid.innerHTML = CHART_CONFIGS.map((config, index) => `
        <article class="chart-card" role="button" tabindex="0"
                 data-index="${index}"
                 data-label="${escapeHtml(config.label)}"
                 data-scale="${escapeHtml(config.scale)}"
                 style="--accent:${escapeHtml(config.color)};">
          <div class="chart-head">
            <div>
              <div class="chart-title">${escapeHtml(config.title)}</div>
              <div class="chart-meta">${escapeHtml(config.group_label)} • ${escapeHtml(config.unit)}</div>
            </div>
            <div class="chart-value" data-role="value">--</div>
          </div>
          <div class="chart-plot">
            <canvas data-role="canvas"></canvas>
          </div>
          <div class="chart-foot">
            <span data-role="range">range --</span>
            <span data-role="grid">grid --</span>
          </div>
        </article>
      `).join("");

      grid.querySelectorAll(".chart-card").forEach((card) => {
        const label = card.dataset.label;
        const entry = getChartEntry(label);
        if (!entry) {
          return;
        }
        entry.card = card;
        entry.canvas = card.querySelector("[data-role='canvas']");
        entry.valueNode = card.querySelector("[data-role='value']");
        entry.rangeNode = card.querySelector("[data-role='range']");
        entry.gridNode = card.querySelector("[data-role='grid']");
        card.addEventListener("click", () => setFocus(label));
        card.addEventListener("keydown", (event) => {
          if (event.key === "Enter" || event.key === " ") {
            event.preventDefault();
            setFocus(label);
          }
        });
      });
    }

    function computeStats(samples, chartIndex) {
      let min = Infinity;
      let max = -Infinity;
      let sum = 0;
      let count = 0;
      let latest = NaN;
      for (const sample of samples) {
        const value = sample.values[chartIndex];
        if (!Number.isFinite(value)) {
          continue;
        }
        if (value < min) min = value;
        if (value > max) max = value;
        sum += value;
        latest = value;
        count += 1;
      }
      if (count === 0) {
        return null;
      }
      return {
        count,
        min,
        max,
        mean: sum / count,
        latest,
        span: max - min,
      };
    }

    function buildTargetDomain(stats, config) {
      if (!stats) {
        const half = Math.max(config.min_span / 2, 0.5);
        return { min: -half, max: half };
      }

      if (config.scale === "symmetric") {
        const baseBound = Math.max(Math.abs(stats.min), Math.abs(stats.max), config.min_span / 2);
        const padding = Math.max(baseBound * 0.12, config.min_span * 0.08);
        const bound = Math.max(baseBound + padding, config.min_span / 2);
        return { min: -bound, max: bound };
      }

      const span = Math.max(stats.max - stats.min, config.min_span);
      const padding = Math.max(span * 0.12, config.min_span * 0.08);
      return {
        min: stats.min - padding,
        max: stats.max + padding,
      };
    }

    function settleDomain(previous, target, config) {
      if (!previous) {
        return target;
      }

      const contractRate = 0.08;
      const next = {
        min: target.min < previous.min ? target.min : previous.min + (target.min - previous.min) * contractRate,
        max: target.max > previous.max ? target.max : previous.max + (target.max - previous.max) * contractRate,
      };

      const minSpan = Math.max(config.min_span, 1e-6);
      if (next.max - next.min < minSpan) {
        const center = (next.min + next.max) / 2;
        next.min = center - minSpan / 2;
        next.max = center + minSpan / 2;
      }
      return next;
    }

    function buildNiceDomain(domain, config, targetTicks) {
      if (config.scale === "symmetric") {
        const maxAbs = Math.max(Math.abs(domain.min), Math.abs(domain.max), config.min_span / 2);
        const step = niceStep(maxAbs * 2, targetTicks);
        const bound = Math.max(step, Math.ceil(maxAbs / step) * step);
        return { min: -bound, max: bound, step };
      }

      const step = niceStep(domain.max - domain.min, targetTicks);
      const min = Math.floor(domain.min / step) * step;
      const max = Math.ceil(domain.max / step) * step;
      return { min, max, step };
    }

    function resolveCanvas(canvas, state, focused) {
      if (canvas) {
        return canvas;
      }

      if (state?.card) {
        const fallbackCanvas = state.card.querySelector("[data-role='canvas']");
        if (fallbackCanvas) {
          state.canvas = fallbackCanvas;
          return fallbackCanvas;
        }
      }

      if (focused) {
        return document.getElementById("focus-canvas");
      }

      return null;
    }

    function drawChart(canvas, samples, config, state, focused) {
      const resolvedCanvas = resolveCanvas(canvas, state, focused);
      if (!resolvedCanvas) {
        return null;
      }

      const rect = resolvedCanvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.floor(rect.width));
      const height = Math.max(1, Math.floor(rect.height));
      const targetWidth = Math.max(1, Math.floor(width * dpr));
      const targetHeight = Math.max(1, Math.floor(height * dpr));
      if (resolvedCanvas.width !== targetWidth || resolvedCanvas.height !== targetHeight) {
        resolvedCanvas.width = targetWidth;
        resolvedCanvas.height = targetHeight;
      }

      const ctx = resolvedCanvas.getContext("2d");
      if (!ctx) {
        return null;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, width, height);

      const padding = focused
        ? { left: 62, right: 18, top: 16, bottom: 28 }
        : { left: 54, right: 14, top: 12, bottom: 24 };
      const plot = {
        x: padding.left,
        y: padding.top,
        w: Math.max(0, width - padding.left - padding.right),
        h: Math.max(0, height - padding.top - padding.bottom),
      };

      const bgGradient = ctx.createLinearGradient(0, 0, 0, height);
      bgGradient.addColorStop(0, "rgba(255, 255, 255, 0.02)");
      bgGradient.addColorStop(1, "rgba(0, 0, 0, 0.08)");
      ctx.fillStyle = bgGradient;
      ctx.fillRect(plot.x, plot.y, plot.w, plot.h);

      if (plot.w <= 8 || plot.h <= 8) {
        return;
      }

      const latestTime = samples.length > 0 ? samples[samples.length - 1].t : 0;
      const startTime = latestTime - appState.windowSeconds;
      const stats = computeStats(samples, state.index);
      const targetDomain = buildTargetDomain(stats, config);
      state.domain = settleDomain(state.domain, targetDomain, config);
      const niceDomain = buildNiceDomain(state.domain, config, focused ? 6 : 5);

      const xStep = niceStep(appState.windowSeconds, focused ? 6 : 4);
      const xTicks = buildTicks(-appState.windowSeconds, 0, xStep);
      const yTicks = buildTicks(niceDomain.min, niceDomain.max, niceDomain.step);
      const zeroVisible = niceDomain.min < 0 && niceDomain.max > 0;

      ctx.save();
      ctx.beginPath();
      ctx.rect(plot.x, plot.y, plot.w, plot.h);
      ctx.clip();

      ctx.strokeStyle = "rgba(255, 255, 255, 0.06)";
      ctx.lineWidth = 1;

      for (const tick of yTicks) {
        const y = plot.y + plot.h - ((tick - niceDomain.min) / (niceDomain.max - niceDomain.min)) * plot.h;
        ctx.beginPath();
        ctx.moveTo(plot.x, y);
        ctx.lineTo(plot.x + plot.w, y);
        ctx.stroke();
      }

      for (const tick of xTicks) {
        const x = plot.x + ((tick + appState.windowSeconds) / appState.windowSeconds) * plot.w;
        ctx.beginPath();
        ctx.moveTo(x, plot.y);
        ctx.lineTo(x, plot.y + plot.h);
        ctx.stroke();
      }

      if (zeroVisible) {
        const zeroY = plot.y + plot.h - ((0 - niceDomain.min) / (niceDomain.max - niceDomain.min)) * plot.h;
        ctx.strokeStyle = hexToRgba(config.color, 0.32);
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        ctx.moveTo(plot.x, zeroY);
        ctx.lineTo(plot.x + plot.w, zeroY);
        ctx.stroke();
      }

      ctx.restore();

      const points = [];
      for (const sample of samples) {
        const value = sample.values[state.index];
        if (!Number.isFinite(value)) {
          continue;
        }
        const x = plot.x + ((sample.t - startTime) / appState.windowSeconds) * plot.w;
        const y = plot.y + plot.h - ((value - niceDomain.min) / (niceDomain.max - niceDomain.min)) * plot.h;
        points.push({ x, y, value });
      }

      if (points.length === 0) {
        drawEmptyState(ctx, plot, config);
        drawAxes(ctx, plot, niceDomain, xTicks, yTicks, config, focused);
        return;
      }

      if (focused && points.length > 1) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i += 1) {
          ctx.lineTo(points[i].x, points[i].y);
        }
        ctx.lineTo(points[points.length - 1].x, plot.y + plot.h);
        ctx.lineTo(points[0].x, plot.y + plot.h);
        ctx.closePath();
        const fillGradient = ctx.createLinearGradient(0, plot.y, 0, plot.y + plot.h);
        fillGradient.addColorStop(0, hexToRgba(config.color, 0.28));
        fillGradient.addColorStop(1, hexToRgba(config.color, 0.03));
        ctx.fillStyle = fillGradient;
        ctx.fill();
        ctx.restore();
      }

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; i += 1) {
        ctx.lineTo(points[i].x, points[i].y);
      }
      ctx.strokeStyle = config.color;
      ctx.lineWidth = focused ? 2.8 : 2.0;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.shadowColor = hexToRgba(config.color, focused ? 0.32 : 0.20);
      ctx.shadowBlur = focused ? 18 : 10;
      ctx.stroke();
      ctx.restore();

      const lastPoint = points[points.length - 1];
      ctx.save();
      ctx.beginPath();
      ctx.arc(lastPoint.x, lastPoint.y, focused ? 4.2 : 3.4, 0, Math.PI * 2);
      ctx.fillStyle = config.color;
      ctx.shadowColor = hexToRgba(config.color, 0.40);
      ctx.shadowBlur = focused ? 18 : 10;
      ctx.fill();
      ctx.restore();

      drawAxes(ctx, plot, niceDomain, xTicks, yTicks, config, focused);
      return { stats, niceDomain };
    }

    function drawEmptyState(ctx, plot, config) {
      ctx.save();
      ctx.fillStyle = "rgba(255, 255, 255, 0.34)";
      ctx.font = "600 12px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(`Waiting for ${config.title.toLowerCase()} samples`, plot.x + plot.w / 2, plot.y + plot.h / 2);
      ctx.restore();
    }

    function drawAxes(ctx, plot, yDomain, xTicks, yTicks, config, focused) {
      ctx.save();
      ctx.fillStyle = "rgba(255, 255, 255, 0.72)";
      ctx.font = focused
        ? "12px Avenir Next, SF Pro Text, Segoe UI, sans-serif"
        : "11px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textBaseline = "middle";

      for (const tick of yTicks) {
        const y = plot.y + plot.h - ((tick - yDomain.min) / (yDomain.max - yDomain.min)) * plot.h;
        ctx.textAlign = "right";
        ctx.fillText(formatTick(tick, yDomain.step), plot.x - 8, y);
      }

      ctx.textBaseline = "top";
      for (const tick of xTicks) {
        const x = plot.x + ((tick + appState.windowSeconds) / appState.windowSeconds) * plot.w;
        ctx.textAlign = "center";
        ctx.fillText(`${formatTick(tick, niceStep(appState.windowSeconds, focused ? 6 : 4))}s`, x, plot.y + plot.h + 4);
      }

      ctx.fillStyle = "rgba(255, 255, 255, 0.42)";
      ctx.font = "11px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textAlign = "left";
      ctx.fillText(config.unit, plot.x, plot.y - 12);
      ctx.restore();
    }

    function updateChartCard(entry, stats, drawResult, visible) {
      const { config, card } = entry;
      if (!card) {
        return;
      }
      card.style.display = visible ? "" : "none";
      card.classList.toggle("active", config.label === appState.focusLabel);
      if (stats) {
        entry.valueNode.textContent = formatValue(stats.latest, config.unit, config.precision);
        entry.rangeNode.textContent = `range ${formatTick(stats.min, drawResult.niceDomain.step)} … ${formatTick(stats.max, drawResult.niceDomain.step)} ${config.unit}`;
        entry.gridNode.textContent = `grid ${formatTick(drawResult.niceDomain.step, drawResult.niceDomain.step)} ${config.unit}`;
      } else {
        entry.valueNode.textContent = "--";
        entry.rangeNode.textContent = "range --";
        entry.gridNode.textContent = "grid --";
      }
    }

    function updateFocusPanel(focusEntry, stats, drawResult, historyWindow) {
      const { config } = focusEntry;
      document.getElementById("focus-title").textContent = config.title;
      document.getElementById("focus-subtitle").textContent = `${config.group_label} • ${config.unit}`;
      document.getElementById("focus-index").textContent =
        `${String(focusEntry.index + 1).padStart(2, "0")} / ${String(CHART_CONFIGS.length).padStart(2, "0")}`;
      document.getElementById("focus-current").textContent =
        stats ? formatValue(stats.latest, config.unit, config.precision) : "--";
      document.getElementById("focus-span").textContent =
        stats ? `${formatNumber(stats.span, config.precision)} ${config.unit}` : "--";
      document.getElementById("focus-step").textContent =
        drawResult ? `${formatTick(drawResult.niceDomain.step, drawResult.niceDomain.step)} ${config.unit}` : "--";
      document.getElementById("focus-samples").textContent = String(historyWindow.length);
    }

    function updateConnectionPanel() {
      const data = appState.latestSnapshot;
      const connected = Boolean(data && data.connected);
      const status = connected ? (data.status || "connected") : (data?.status || "waiting");
      const dot = document.getElementById("status-dot");
      const statusText = document.getElementById("status-text");
      const heroSubtitle = document.getElementById("hero-subtitle");
      const connectionSubtitle = document.getElementById("connection-subtitle");

      dot.className = "dot";
      if (connected && status === "live") {
        dot.classList.add("live");
        statusText.textContent = "live";
        heroSubtitle.textContent = `Receiving live samples from ${data.robot_name || "the controller"} through ${data.shared_memory_name}.`;
        connectionSubtitle.textContent = data.message || "Live data is arriving.";
      } else if (connected && status === "busy") {
        dot.classList.add("busy");
        statusText.textContent = "busy";
        heroSubtitle.textContent = data.message || "Controller is updating the shared buffer.";
        connectionSubtitle.textContent = data.message || "Controller is updating the shared buffer.";
      } else if (connected) {
        dot.classList.add("live");
        statusText.textContent = status;
        heroSubtitle.textContent = data.message || "Connected to the shared memory segment.";
        connectionSubtitle.textContent = data.message || "Connected.";
      } else if (status === "error") {
        dot.classList.add("error");
        statusText.textContent = "error";
        heroSubtitle.textContent = data.message || "Unable to connect to the dashboard shared memory.";
        connectionSubtitle.textContent = data.message || "Unable to connect.";
      } else {
        dot.classList.add("busy");
        statusText.textContent = "waiting";
        heroSubtitle.textContent = data.message || "Waiting for the controller to publish samples.";
        connectionSubtitle.textContent = data.message || "Waiting for the controller to publish samples.";
      }

      document.getElementById("robot-name").textContent = connected ? (data.robot_name || "--") : "--";
      document.getElementById("iteration").textContent = connected ? String(data.iteration ?? "--") : "--";
      document.getElementById("sim-time").textContent = connected ? formatNumber(data.sim_time, 3) : "--";
      document.getElementById("shm-name").textContent = data.shared_memory_name || "--";
      document.getElementById("sequence-text").textContent = connected ? `seq ${data.sequence}` : "seq --";
      document.getElementById("layout-version").textContent = data.version ?? "--";
      document.getElementById("state-dim").textContent = data.state_dim ?? "--";
      document.getElementById("history-label").textContent = `history ${appState.history.length}`;
    }

    function pruneHistory() {
      if (appState.history.length === 0) {
        return;
      }
      const latestTime = appState.history[appState.history.length - 1].t;
      const keepSeconds = Math.max(appState.windowSeconds * 4, 60);
      const cutoff = latestTime - keepSeconds;
      while (appState.history.length > 0 && appState.history[0].t < cutoff) {
        appState.history.shift();
      }
    }

    function pushSnapshot(data) {
      if (!data || !data.connected || !Array.isArray(data.state) || data.state.length !== STATE_LABELS.length) {
        appState.latestSnapshot = data;
        return false;
      }

      if (appState.lastSequence !== null && data.sequence === appState.lastSequence) {
        appState.latestSnapshot = data;
        return false;
      }

      if (appState.history.length > 0) {
        const lastTime = appState.history[appState.history.length - 1].t;
        if (data.sim_time + 1e-9 < lastTime) {
          appState.history = [];
          appState.chartRuntime.forEach((entry) => {
            entry.domain = null;
          });
        }
      }

      appState.lastSequence = data.sequence;
      appState.latestSnapshot = data;
      appState.history.push({
        sequence: data.sequence,
        t: data.sim_time,
        values: data.state.slice(),
      });
      pruneHistory();
      return true;
    }

    function renderCharts() {
      const visibleConfigs = visibleChartsForView();
      const visibleLabels = new Set(visibleConfigs.map((config) => config.label));
      const historyWindow = appState.history.filter((sample) => sample.t >= (appState.history.length > 0 ? appState.history[appState.history.length - 1].t - appState.windowSeconds : 0));
      const focusEntry = getChartEntry(appState.focusLabel) || getChartEntry(CHART_CONFIGS[0].label);
      if (!focusEntry) {
        return;
      }

      document.getElementById("chart-grid").dataset.view = appState.view;

      const statsByIndex = CHART_ORDER.map((entry) => computeStats(historyWindow, entry.index));

      for (const orderEntry of CHART_ORDER) {
        const runtimeEntry = getChartEntry(orderEntry.config.label);
        if (!runtimeEntry || orderEntry.config.label === appState.focusLabel) {
          continue;
        }
        const stats = statsByIndex[orderEntry.index];
        const visible = visibleLabels.has(orderEntry.config.label);
        updateChartCard(
          runtimeEntry,
          stats,
          drawChart(runtimeEntry.canvas, historyWindow, orderEntry.config, runtimeEntry, false) || { niceDomain: { step: 1 } },
          visible
        );
      }

      const focusStats = statsByIndex[focusEntry.index];
      const focusDraw = drawChart(document.getElementById("focus-canvas"), historyWindow, focusEntry.config, focusEntry, true) || { niceDomain: { step: 1 } };
      updateFocusPanel(focusEntry, focusStats, focusDraw, historyWindow);
      updateChartCard(focusEntry, focusStats, focusDraw, visibleLabels.has(focusEntry.config.label));

      for (const orderEntry of CHART_ORDER) {
        const runtimeEntry = getChartEntry(orderEntry.config.label);
        const card = runtimeEntry?.card;
        if (!card) continue;
        card.classList.toggle("active", orderEntry.config.label === appState.focusLabel);
        if (appState.view === "focus") {
          card.style.display = orderEntry.config.label === appState.focusLabel ? "" : "none";
        } else {
          card.style.display = visibleLabels.has(orderEntry.config.label) ? "" : "none";
        }
      }
    }

    function render() {
      updateConnectionPanel();
      renderCharts();
    }

    function safeRender() {
      try {
        render();
      } catch (error) {
        console.error("[dashboard] render failed", error);
      }
    }

    function ingestSnapshot(data) {
      const changed = pushSnapshot(data);
      if (changed || data?.status === "busy" || data?.status === "error" || data?.status === "waiting") {
        safeRender();
      }
    }

    async function pollSnapshot() {
      if (appState.polling) {
        return;
      }
      appState.polling = true;
      try {
        const response = await fetch("/api/state", { cache: "no-store" });
        const data = await response.json();
        ingestSnapshot(data);
      } catch (error) {
        appState.latestSnapshot = {
          connected: false,
          status: "error",
          message: error.message || String(error),
          shared_memory_name: "--",
          robot_name: "",
          sequence: 0,
          iteration: 0,
          sim_time: 0.0,
          version: null,
          state_dim: null,
          state: [],
        };
        safeRender();
      } finally {
        appState.polling = false;
        window.setTimeout(pollSnapshot, 50);
      }
    }

    function bindControls() {
      document.querySelectorAll("#view-buttons button").forEach((button) => {
        button.addEventListener("click", () => setView(button.dataset.view));
      });
      document.querySelectorAll("#window-buttons button").forEach((button) => {
        button.addEventListener("click", () => setWindow(Number(button.dataset.window)));
      });
      document.getElementById("focus-prev").addEventListener("click", () => stepFocus(-1));
      document.getElementById("focus-next").addEventListener("click", () => stepFocus(1));
      window.addEventListener("resize", render);
    }

    buildChartCards();
    bindControls();
    updateButtonStates();
    safeRender();
    void pollSnapshot();
  </script>
</body>
</html>
"""


class DashboardSharedMemoryClient:
    def __init__(self, shared_memory_name: str):
        self.shared_memory_name = shared_memory_name.lstrip("/") or "convexmpc_dashboard_state"
        self._shm: shared_memory.SharedMemory | None = None
        self._last_error: str | None = None

    @property
    def is_open(self) -> bool:
        return self._shm is not None

    def close(self) -> None:
        if self._shm is not None:
            self._shm.close()
            self._shm = None

    def _open(self) -> bool:
        if self._shm is not None:
            return True
        try:
            self._shm = shared_memory.SharedMemory(
                name=self.shared_memory_name,
                create=False,
                track=False,
            )
        except TypeError:
            self._shm = shared_memory.SharedMemory(
                name=self.shared_memory_name,
                create=False,
            )
        except FileNotFoundError:
            self._last_error = "shared memory not available yet"
            return False
        except OSError as exc:
            self._last_error = str(exc)
            return False

        if self._shm.size < LAYOUT_SIZE:
            self._last_error = (
                f"shared memory segment is too small: {self._shm.size} < {LAYOUT_SIZE}"
            )
            self.close()
            return False

        self._last_error = None
        return True

    def snapshot(self) -> dict[str, object]:
        if not self._open():
            return {
                "connected": False,
                "status": "waiting",
                "message": self._last_error or "waiting for controller",
                "shared_memory_name": self.shared_memory_name,
                "robot_name": "",
                "sequence": 0,
                "iteration": 0,
                "sim_time": 0.0,
                "version": None,
                "state_dim": None,
                "state": [],
            }

        assert self._shm is not None
        buf = self._shm.buf
        seq1 = 0
        seq2 = 0
        fields = None
        for _ in range(8):
            seq1 = struct.unpack_from("<Q", buf, 0)[0]
            if seq1 % 2 == 1:
                time.sleep(0.001)
                continue
            fields = LAYOUT.unpack_from(buf, 0)
            seq2 = struct.unpack_from("<Q", buf, 0)[0]
            if seq1 == seq2 and seq2 % 2 == 0:
                break
            fields = None
        if fields is None:
            return {
                "connected": True,
                "status": "busy",
                "message": "controller is updating the shared buffer",
                "shared_memory_name": self.shared_memory_name,
                "robot_name": "",
                "sequence": seq2,
                "iteration": 0,
                "sim_time": 0.0,
                "version": None,
                "state_dim": None,
                "state": [],
            }

        sequence, iteration, sim_time, robot_raw = fields[:4]
        state = list(fields[4:16])
        version = fields[16]
        state_dim = fields[17]
        robot_name = robot_raw.split(b"\x00", 1)[0].decode("utf-8", "replace")
        status = "live" if sequence > 0 else "priming"
        message = "live data" if sequence > 0 else "waiting for first controller sample"
        if version != 1:
            status = "error"
            message = f"unexpected layout version {version}"
        elif state_dim != len(STATE_LABELS):
            status = "error"
            message = f"unexpected state dimension {state_dim}"

        return {
            "connected": True,
            "status": status,
            "message": message,
            "shared_memory_name": self.shared_memory_name,
            "robot_name": robot_name,
            "sequence": int(sequence),
            "iteration": int(iteration),
            "sim_time": float(sim_time),
            "version": int(version),
            "state_dim": int(state_dim),
            "state": state,
            "state_map": {label: state[idx] for idx, label in enumerate(STATE_LABELS)},
        }


class DashboardHTTPServer(ThreadingHTTPServer):
    def __init__(self, server_address, RequestHandlerClass, reader: DashboardSharedMemoryClient):
        super().__init__(server_address, RequestHandlerClass)
        self.reader = reader


class DashboardHandler(BaseHTTPRequestHandler):
    def _send_bytes(self, content: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _snapshot_payload(self) -> dict[str, object]:
        payload = self.server.reader.snapshot()
        payload["server_time"] = time.time()
        return payload

    def do_GET(self) -> None:  # noqa: N802
        if self.path in {"/", "/index.html"}:
            html = HTML_TEMPLATE.replace("%CHART_CONFIG%", json.dumps(CHART_CONFIGS, ensure_ascii=False)).replace(
                "%STATE_LABELS%",
                json.dumps(list(STATE_LABELS)),
            )
            self._send_bytes(html.encode("utf-8"), "text/html; charset=utf-8")
            return

        if self.path == "/api/state":
            payload = self._snapshot_payload()
            self._send_bytes(
                json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        if self.path == "/api/health":
            self._send_bytes(b"ok", "text/plain; charset=utf-8")
            return

        self.send_error(404, "Not Found")

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ConvexMPC dashboard")
    parser.add_argument("--host", default=os.environ.get("CONVEXMPC_DASHBOARD_HOST", DEFAULT_HOST))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("CONVEXMPC_DASHBOARD_PORT", DEFAULT_PORT)),
    )
    parser.add_argument(
        "--shared-memory",
        default=os.environ.get("CONVEXMPC_SHM_NAME", DEFAULT_SHM_NAME),
        dest="shared_memory",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open the browser automatically",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    reader = DashboardSharedMemoryClient(args.shared_memory)
    server = DashboardHTTPServer((args.host, args.port), DashboardHandler, reader)
    url = f"http://{args.host}:{args.port}"

    if not args.no_browser and os.environ.get("CONVEXMPC_DASHBOARD_OPEN_BROWSER", "0") not in {
        "0",
        "false",
        "False",
        "",
    }:
        webbrowser.open(url, new=2)

    print(f"[dashboard] listening on {url}")
    print(f"[dashboard] shared memory: {args.shared_memory}")

    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        reader.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
