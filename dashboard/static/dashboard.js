    const DASHBOARD_CONFIG = window.__DASHBOARD_CONFIG__ || {};
    const CHART_CONFIGS = DASHBOARD_CONFIG.chartConfigs || [];
    const STATE_LABELS = DASHBOARD_CONFIG.stateLabels || [];
    const CHART_ORDER = CHART_CONFIGS.map((config, index) => ({ config, index }));
    const BASE_CHART_COUNT = CHART_CONFIGS.length;
    const COMMAND_SERIES_BY_LABEL = {
      omega_z: { label: "cmd_psi_dot", baseLabel: "omega_z", index: 2, title: "Command psi_dot", valueLabel: "Command", color: "#f472b6" },
      vel_x: { label: "cmd_vel_x", baseLabel: "vel_x", index: 0, title: "Command vel_x", valueLabel: "Command", color: "#86efac" },
      vel_y: { label: "cmd_vel_y", baseLabel: "vel_y", index: 1, title: "Command vel_y", valueLabel: "Command", color: "#fca5a5" },
      roll: { label: "target_roll", baseLabel: "roll", index: 3, title: "Target roll", valueLabel: "Target", color: "#f9a8d4" },
      pitch: { label: "target_pitch", baseLabel: "pitch", index: 4, title: "Target pitch", valueLabel: "Target", color: "#c4b5fd" },
      pos_z: { label: "target_pos_z", baseLabel: "pos_z", index: 5, title: "Target pos_z", valueLabel: "Target", color: "#93c5fd" },
    };
    const COMMAND_STATE_INDEX = {
      xDot: STATE_LABELS.indexOf("cmd_vel_x"),
      yDot: STATE_LABELS.indexOf("cmd_vel_y"),
      psiDot: STATE_LABELS.indexOf("cmd_psi_dot"),
    };
    const DEFAULT_WINDOW_SECONDS = DASHBOARD_CONFIG.defaultWindowSeconds ?? 10;
    const WINDOW_OPTIONS = DASHBOARD_CONFIG.windowOptions || [5, 10, 20, 30];
    const MAX_MAIN_PANELS = DASHBOARD_CONFIG.maxMainPanels ?? 3;
    const MIN_MAIN_PANELS = DASHBOARD_CONFIG.minMainPanels ?? 1;
    const MAIN_LAYOUT_STORAGE_KEY = "convexmpc.mainPanelLayout.v2";
    const MAIN_PANEL_MODES = {
      raw: "Raw",
      mean: "Mean",
      ma: "MA",
    };
    const DEFAULT_MAIN_MA_SECONDS = 1.0;
    const MAIN_MA_MIN_SECONDS = 0.25;
    const MAIN_MA_MAX_SECONDS = 10.0;
    const MAIN_MA_STEP_SECONDS = 0.25;
    const VIEW_LABELS = {
      all: "All 12 channels",
      pose: "Pose channels",
      motion: "Motion channels",
    };

    const appState = {
      view: "all",
      windowSeconds: DEFAULT_WINDOW_SECONDS,
      angleUnit: "deg",
      mainLabels: ["vel_x", "vel_y", "omega_z"],
      mainPanelStates: [
        { mode: "raw", movingAverageSeconds: DEFAULT_MAIN_MA_SECONDS },
        { mode: "raw", movingAverageSeconds: DEFAULT_MAIN_MA_SECONDS },
        { mode: "raw", movingAverageSeconds: DEFAULT_MAIN_MA_SECONDS },
      ],
      mainLayouts: [],
      latestSnapshot: null,
      lastSequence: null,
      controllerLastSequence: null,
      controllerLastChangeAt: null,
      history: [],
      mainRuntime: [],
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

    function setTextContent(id, value) {
      const node = document.getElementById(id);
      if (node) {
        node.textContent = value;
      }
    }

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

    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }

    function isAngularUnit(unit) {
      return unit === "rad" || unit === "rad/s";
    }

    function displayScale(config) {
      if (appState.angleUnit === "deg" && isAngularUnit(config.unit)) {
        return 180 / Math.PI;
      }
      return 1;
    }

    function displayUnit(config) {
      if (appState.angleUnit === "deg") {
        if (config.unit === "rad") {
          return "deg";
        }
        if (config.unit === "rad/s") {
          return "deg/s";
        }
      }
      return config.unit;
    }

    function displayPrecision(config) {
      if (appState.angleUnit === "deg" && isAngularUnit(config.unit)) {
        return 1;
      }
      return config.precision;
    }

    function displayConfig(config) {
      const scale = displayScale(config);
      return {
        ...config,
        unit: displayUnit(config),
        min_span: config.min_span * scale,
        precision: displayPrecision(config),
        __scale: scale,
      };
    }

    function toDisplayValue(value, config) {
      if (!Number.isFinite(value)) {
        return value;
      }
      return value * displayScale(config);
    }

    function toDisplayStats(stats, config) {
      if (!stats) {
        return null;
      }
      const scale = displayScale(config);
      return {
        ...stats,
        min: stats.min * scale,
        max: stats.max * scale,
        mean: stats.mean * scale,
        latest: stats.latest * scale,
        span: stats.span * scale,
      };
    }

    function hexToRgba(hex, alpha) {
      const clean = hex.replace("#", "");
      const r = parseInt(clean.slice(0, 2), 16);
      const g = parseInt(clean.slice(2, 4), 16);
      const b = parseInt(clean.slice(4, 6), 16);
      return `rgba(${r}, ${g}, ${b}, ${alpha})`;
    }

    function prepareCanvas(canvas) {
      if (!canvas) {
        return null;
      }
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.floor(rect.width));
      const height = Math.max(1, Math.floor(rect.height));
      const targetWidth = Math.max(1, Math.floor(width * dpr));
      const targetHeight = Math.max(1, Math.floor(height * dpr));
      if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
        canvas.width = targetWidth;
        canvas.height = targetHeight;
      }
      const ctx = canvas.getContext("2d");
      if (!ctx) {
        return null;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, width, height);
      return { ctx, width, height };
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

    function formatDurationSeconds(seconds) {
      return `${formatNumber(seconds, seconds >= 1 ? 1 : 2)}s`;
    }

    function normalizeMainPanelState(state) {
      const mode = MAIN_PANEL_MODES[state?.mode] ? state.mode : "raw";
      const movingAverageSeconds = clamp(
        Number.isFinite(state?.movingAverageSeconds) ? state.movingAverageSeconds : DEFAULT_MAIN_MA_SECONDS,
        MAIN_MA_MIN_SECONDS,
        MAIN_MA_MAX_SECONDS
      );
      return { mode, movingAverageSeconds };
    }

    function defaultMainLayout(index) {
      return {
        span: 4,
        height: 300,
        order: index,
      };
    }

    function normalizeMainLayout(layout, index) {
      const base = defaultMainLayout(index);
      return {
        span: Math.round(clamp(Number.isFinite(layout?.span) ? layout.span : base.span, 4, 12)),
        height: Math.round(clamp(Number.isFinite(layout?.height) ? layout.height : base.height, 220, 620)),
        order: Math.round(clamp(Number.isFinite(layout?.order) ? layout.order : base.order, 0, MAX_MAIN_PANELS - 1)),
      };
    }

    function loadMainLayouts() {
      let saved = [];
      try {
        saved = JSON.parse(window.localStorage.getItem(MAIN_LAYOUT_STORAGE_KEY) || "[]");
      } catch {
        saved = [];
      }
      appState.mainLayouts = Array.from({ length: MAX_MAIN_PANELS }, (_, index) =>
        normalizeMainLayout(saved[index], index)
      );
    }

    function saveMainLayouts() {
      try {
        window.localStorage.setItem(MAIN_LAYOUT_STORAGE_KEY, JSON.stringify(appState.mainLayouts));
      } catch {
        // Layout persistence is best-effort only.
      }
    }

    function getMainLayout(index) {
      if (!appState.mainLayouts[index]) {
        appState.mainLayouts[index] = defaultMainLayout(index);
      }
      const normalized = normalizeMainLayout(appState.mainLayouts[index], index);
      appState.mainLayouts[index] = normalized;
      return normalized;
    }

    function setMainLayout(index, patch, { persist = true } = {}) {
      appState.mainLayouts[index] = normalizeMainLayout({ ...getMainLayout(index), ...patch }, index);
      applyMainPanelLayouts();
      if (persist) {
        saveMainLayouts();
      }
    }

    function applyMainPanelLayouts() {
      appState.mainRuntime.forEach((entry, index) => {
        if (!entry?.panel) {
          return;
        }
        const layout = getMainLayout(index);
        const visible = index < appState.mainLabels.length;
        entry.panel.classList.toggle("movable", true);
        entry.panel.style.setProperty("--main-span", String(layout.span));
        entry.panel.style.setProperty("--main-height", `${layout.height}px`);
        entry.panel.style.order = String(layout.order);
        entry.panel.style.display = visible ? "" : "none";
      });
    }

    function visibleMainPanelEntries() {
      return appState.mainRuntime
        .map((entry, index) => ({ entry, index, layout: getMainLayout(index) }))
        .filter((item) => item.index < appState.mainLabels.length && item.entry?.panel)
        .sort((a, b) => a.layout.order - b.layout.order || a.index - b.index);
    }

    function normalizeMainLayoutOrders() {
      visibleMainPanelEntries().forEach((item, order) => {
        appState.mainLayouts[item.index] = normalizeMainLayout({ ...item.layout, order }, item.index);
      });
      applyMainPanelLayouts();
    }

    function mainPanelRows() {
      const rows = [];
      let current = [];
      let used = 0;
      for (const item of visibleMainPanelEntries()) {
        const span = getMainLayout(item.index).span;
        if (current.length > 0 && used + span > 12) {
          rows.push(current);
          current = [];
          used = 0;
        }
        current.push(item);
        used += span;
      }
      if (current.length > 0) {
        rows.push(current);
      }
      return rows;
    }

    function distributeMainRow(row, primaryIndex, primarySpan) {
      if (row.length <= 1) {
        setMainLayout(primaryIndex, { span: primarySpan }, { persist: false });
        return;
      }
      const primary = Math.round(clamp(primarySpan, 4, 8));
      const remaining = 12 - primary;
      const others = row.filter((item) => item.index !== primaryIndex);
      const share = Math.max(4, Math.floor(remaining / Math.max(others.length, 1)));
      setMainLayout(primaryIndex, { span: primary }, { persist: false });
      others.forEach((item, offset) => {
        const span = offset === others.length - 1
          ? 12 - primary - share * (others.length - 1)
          : share;
        setMainLayout(item.index, { span: clamp(span, 4, 8) }, { persist: false });
      });
    }

    function commitMainPanelDrop(index, clientX, clientY) {
      const dragged = appState.mainRuntime[index];
      if (!dragged?.panel) {
        return;
      }
      const others = visibleMainPanelEntries()
        .filter((item) => item.index !== index)
        .map((item) => ({
          ...item,
          rect: item.entry.panel.getBoundingClientRect(),
        }));

      if (others.length === 0) {
        setMainLayout(index, { order: 0 }, { persist: false });
        saveMainLayouts();
        return;
      }

      const rows = [];
      for (const item of others.sort((a, b) => a.rect.top - b.rect.top || a.rect.left - b.rect.left)) {
        const previous = rows[rows.length - 1];
        const centerY = item.rect.top + item.rect.height / 2;
        if (!previous || centerY > previous.bottom + 24) {
          rows.push({
            top: item.rect.top,
            bottom: item.rect.bottom,
            items: [item],
          });
        } else {
          previous.top = Math.min(previous.top, item.rect.top);
          previous.bottom = Math.max(previous.bottom, item.rect.bottom);
          previous.items.push(item);
        }
      }
      rows.forEach((row) => {
        row.items.sort((a, b) => a.rect.left - b.rect.left);
        row.center = (row.top + row.bottom) / 2;
      });

      const nearestRow = rows.reduce((best, row) => {
        const distance = Math.abs(clientY - row.center);
        return !best || distance < best.distance ? { row, distance } : best;
      }, null)?.row;
      const rowHeight = nearestRow ? Math.max(1, nearestRow.bottom - nearestRow.top) : 1;
      const makeOwnRow = !nearestRow || Math.abs(clientY - nearestRow.center) > rowHeight * 0.46;

      const orderedRows = rows.map((row) => row.items.map((item) => item.index));
      if (makeOwnRow) {
        const insertAt = rows.findIndex((row) => clientY < row.center);
        const rowIndex = insertAt === -1 ? orderedRows.length : insertAt;
        orderedRows.splice(rowIndex, 0, [index]);
        setMainLayout(index, { span: 12 }, { persist: false });
      } else {
        const targetRowIndex = rows.indexOf(nearestRow);
        const targetItems = nearestRow.items;
        const insertAt = targetItems.findIndex((item) => clientX < item.rect.left + item.rect.width / 2);
        const itemIndex = insertAt === -1 ? targetItems.length : insertAt;
        orderedRows[targetRowIndex].splice(itemIndex, 0, index);
        const rowItems = orderedRows[targetRowIndex].map((rowIndexValue) => ({
          index: rowIndexValue,
          layout: getMainLayout(rowIndexValue),
        }));
        const span = rowItems.length >= 3 ? 4 : 6;
        rowItems.forEach((item) => setMainLayout(item.index, { span }, { persist: false }));
      }

      orderedRows.flat().forEach((panelIndex, order) => {
        setMainLayout(panelIndex, { order }, { persist: false });
      });
      normalizeMainLayoutOrders();
      saveMainLayouts();
    }

    function getMainPanelState(index) {
      const existing = appState.mainPanelStates[index];
      if (existing) {
        const normalized = normalizeMainPanelState(existing);
        appState.mainPanelStates[index] = normalized;
        return normalized;
      }
      const next = normalizeMainPanelState();
      appState.mainPanelStates[index] = next;
      return next;
    }

    function setMainPanelState(index, updater) {
      const current = getMainPanelState(index);
      const next = normalizeMainPanelState(updater({ ...current }));
      appState.mainPanelStates[index] = next;
      updateButtonStates();
      render();
    }

    function setMainPanelMode(index, mode) {
      if (!MAIN_PANEL_MODES[mode]) {
        return;
      }
      setMainPanelState(index, (state) => ({ ...state, mode }));
    }

    function adjustMainPanelMovingAverage(index, delta) {
      setMainPanelState(index, (state) => ({
        ...state,
        mode: "ma",
        movingAverageSeconds: clamp(
          state.movingAverageSeconds + delta,
          MAIN_MA_MIN_SECONDS,
          MAIN_MA_MAX_SECONDS
        ),
      }));
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
      return CHART_CONFIGS;
    }

    function getChartEntry(label) {
      return appState.chartRuntime.get(label) || null;
    }

    function getMainLabel(index) {
      return appState.mainLabels[index] || CHART_CONFIGS[0].label;
    }

    function syncPrimaryMainLabel() {
      appState.mainLabels = appState.mainLabels.slice(0, MAX_MAIN_PANELS);
      while (appState.mainLabels.length < MIN_MAIN_PANELS) {
        appState.mainLabels.push(CHART_CONFIGS[0].label);
      }
      appState.mainLabels[0] = appState.mainLabels[0] || CHART_CONFIGS[0].label;
    }

    function ensureMainLabelsVisible() {
      const visible = visibleChartsForView();
      if (visible.length === 0) {
        return;
      }
      appState.mainLabels = appState.mainLabels.map((label) =>
        visible.some((config) => config.label === label) ? label : visible[0].label
      );
      syncPrimaryMainLabel();
    }

    function advanceChartLabel(label, delta) {
      const index = CHART_CONFIGS.findIndex((config) => config.label === label);
      const nextIndex = (index + delta + CHART_CONFIGS.length) % CHART_CONFIGS.length;
      return CHART_CONFIGS[nextIndex].label;
    }

    function setMainPanelCount(nextCount) {
      const clamped = Math.max(MIN_MAIN_PANELS, Math.min(MAX_MAIN_PANELS, nextCount));
      while (appState.mainLabels.length < clamped) {
        const seed = appState.mainLabels[appState.mainLabels.length - 1] || CHART_CONFIGS[0].label;
        appState.mainLabels.push(advanceChartLabel(seed, 1));
        appState.mainPanelStates.push(normalizeMainPanelState());
      }
      appState.mainLabels = appState.mainLabels.slice(0, clamped);
      appState.mainPanelStates = appState.mainPanelStates.slice(0, clamped);
      while (appState.mainPanelStates.length < clamped) {
        appState.mainPanelStates.push(normalizeMainPanelState());
      }
      if (clamped >= 3) {
        setMainLayout(1, { span: 6, order: 1 }, { persist: false });
        setMainLayout(2, { span: 6, order: 2 }, { persist: false });
        saveMainLayouts();
      }
      syncPrimaryMainLabel();
      ensureMainLabelsVisible();
      updateButtonStates();
      applyMainPanelLayouts();
      render();
    }

    function setMainPanelLabel(index, label) {
      if (!CHART_CONFIGS.some((config) => config.label === label)) {
        return;
      }
      appState.mainLabels[index] = label;
      syncPrimaryMainLabel();
      ensureMainLabelsVisible();
      updateButtonStates();
      render();
    }

    function stepMainPanel(index, delta) {
      const currentLabel = getMainLabel(index);
      setMainPanelLabel(index, advanceChartLabel(currentLabel, delta));
    }

    function setView(view) {
      appState.view = view;
      ensureMainLabelsVisible();
      updateButtonStates();
      render();
    }

    function setWindow(seconds) {
      appState.windowSeconds = seconds;
      updateButtonStates();
      pruneHistory();
      render();
    }

    function setAngleUnit(unit) {
      if (unit !== "deg" && unit !== "rad") {
        return;
      }
      appState.angleUnit = unit;
      updateButtonStates();
      render();
    }

    function promoteToMainPanel(label) {
      setMainPanelLabel(0, label);
    }

    function updateButtonStates() {
      document.querySelectorAll("#view-buttons button").forEach((button) => {
        button.classList.toggle("active", button.dataset.view === appState.view);
      });
      document.querySelectorAll("#window-buttons button").forEach((button) => {
        button.classList.toggle("active", Number(button.dataset.window) === appState.windowSeconds);
      });
      document.querySelectorAll("#angle-buttons button").forEach((button) => {
        button.classList.toggle("active", button.dataset.angleUnit === appState.angleUnit);
      });

      const count = appState.mainLabels.length;
      const decButton = document.getElementById("main-count-dec");
      const incButton = document.getElementById("main-count-inc");
      const countLabel = document.getElementById("main-count-label");
      if (decButton) {
        decButton.disabled = count <= MIN_MAIN_PANELS;
      }
      if (incButton) {
        incButton.disabled = count >= MAX_MAIN_PANELS;
      }
      if (countLabel) {
        countLabel.textContent = `${count} / ${MAX_MAIN_PANELS}`;
      }

      const visible = visibleChartsForView();
      document.getElementById("grid-title").textContent = VIEW_LABELS[appState.view] || "All 12 channels";
      document.getElementById("grid-subtitle").textContent =
        appState.view === "pose"
          ? "Orientation and position traces, grouped together for quick inspection."
          : appState.view === "motion"
            ? "Angular and linear velocities, centered around zero when appropriate."
            : "Pose and motion states across the trailing window.";
      document.getElementById("chart-grid").dataset.view = appState.view;

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
        card.addEventListener("click", () => promoteToMainPanel(label));
        card.addEventListener("keydown", (event) => {
          if (event.key === "Enter" || event.key === " ") {
            event.preventDefault();
            promoteToMainPanel(label);
          }
        });
      });
    }

    function startMainPanelDrag(event, index) {
      if (event.button !== 0) {
        return;
      }
      const interactive = event.target.closest("button, .segment, .ma-stepper, .main-nav-buttons");
      if (interactive) {
        return;
      }

      const entry = appState.mainRuntime[index];
      const panel = entry?.panel;
      const container = document.getElementById("main-panels");
      if (!panel || !container) {
        return;
      }

      event.preventDefault();
      panel.setPointerCapture?.(event.pointerId);
      panel.classList.add("dragging");
      const startX = event.clientX;
      const startY = event.clientY;
      const startRect = panel.getBoundingClientRect();
      let lastX = startX;
      let lastY = startY;

      const onMove = (moveEvent) => {
        lastX = moveEvent.clientX;
        lastY = moveEvent.clientY;
        const dx = lastX - startX;
        const dy = lastY - startY;
        panel.style.width = `${startRect.width}px`;
        panel.style.transform = `translate(${dx}px, ${dy}px) scale(1.012)`;
      };

      const onUp = () => {
        panel.releasePointerCapture?.(event.pointerId);
        panel.classList.remove("dragging");
        panel.style.transform = "";
        panel.style.width = "";
        window.removeEventListener("pointermove", onMove);
        window.removeEventListener("pointerup", onUp);
        window.removeEventListener("pointercancel", onUp);
        commitMainPanelDrop(index, lastX, lastY);
        window.requestAnimationFrame(render);
      };

      window.addEventListener("pointermove", onMove);
      window.addEventListener("pointerup", onUp);
      window.addEventListener("pointercancel", onUp);
    }

    function startMainPanelResize(event, index) {
      if (event.button !== 0) {
        return;
      }
      const entry = appState.mainRuntime[index];
      const panel = entry?.panel;
      const container = document.getElementById("main-panels");
      if (!panel || !container) {
        return;
      }

      event.preventDefault();
      event.stopPropagation();
      const handle = event.currentTarget;
      handle.setPointerCapture?.(event.pointerId);
      panel.classList.add("resizing");
      const startX = event.clientX;
      const startY = event.clientY;
      const startLayout = getMainLayout(index);
      const columnWidth = Math.max(32, container.getBoundingClientRect().width / 12);
      let frameRequested = false;

      const applyResize = (moveEvent, persist = false) => {
        const dx = moveEvent.clientX - startX;
        const dy = moveEvent.clientY - startY;
        const row = mainPanelRows().find((items) => items.some((item) => item.index === index)) || [];
        const paired = row.length > 1;
        const maxSpan = row.length >= 3 ? 4 : paired ? 8 : 12;
        const span = clamp(Math.round(startLayout.span + dx / columnWidth), 4, maxSpan);
        const height = clamp(startLayout.height + dy, 220, 620);
        setMainLayout(index, { span, height }, { persist });
        if (paired) {
          const others = row.filter((item) => item.index !== index);
          const remaining = 12 - span;
          const share = Math.max(4, Math.floor(remaining / Math.max(others.length, 1)));
          others.forEach((item, offset) => {
            const nextSpan = offset === others.length - 1
              ? 12 - span - share * (others.length - 1)
              : share;
            setMainLayout(item.index, { span: clamp(nextSpan, 4, 8) }, { persist });
          });
        }
      };

      const onMove = (moveEvent) => {
        if (frameRequested) {
          return;
        }
        frameRequested = true;
        window.requestAnimationFrame(() => {
          frameRequested = false;
          applyResize(moveEvent);
          render();
        });
      };

      const onUp = (upEvent) => {
        handle.releasePointerCapture?.(event.pointerId);
        panel.classList.remove("resizing");
        window.removeEventListener("pointermove", onMove);
        window.removeEventListener("pointerup", onUp);
        window.removeEventListener("pointercancel", onUp);
        applyResize(upEvent, true);
        render();
      };

      window.addEventListener("pointermove", onMove);
      window.addEventListener("pointerup", onUp);
      window.addEventListener("pointercancel", onUp);
    }

    function buildMainPanels() {
      const container = document.getElementById("main-panels");
      container.innerHTML = Array.from({ length: MAX_MAIN_PANELS }, (_, index) => `
        <article class="panel main-panel" data-main-index="${index}">
          <div class="panel-head">
            <div class="panel-head-left">
              <div class="panel-kicker" data-role="main-kicker">Main ${index + 1}</div>
              <h2 class="panel-title" data-role="main-title">--</h2>
              <div class="main-panel-controls">
                <div class="segment main-mode-segment" data-role="main-mode-segment" aria-label="main display mode">
                  <button type="button" data-mode="raw">Raw</button>
                  <button type="button" data-mode="mean">Mean</button>
                  <button type="button" data-mode="ma">MA</button>
                </div>
                <div class="ma-stepper" data-role="main-ma-control" aria-label="moving average window">
                  <button type="button" data-action="ma-dec" aria-label="decrease moving average window">-</button>
                  <div class="ma-stepper-value" data-role="main-ma-value">1.0s</div>
                  <button type="button" data-action="ma-inc" aria-label="increase moving average window">+</button>
                </div>
              </div>
            </div>
            <div class="panel-head-right">
              <div class="panel-mode-badge" data-role="main-mode-badge">Raw</div>
              <div class="panel-current panel-overlay-current hidden" data-role="main-overlay-current">
                <span class="panel-current-label" data-role="main-overlay-label">Target</span>
                <span data-role="main-overlay-value">--</span>
              </div>
              <div class="panel-current" data-role="main-current">
                <span class="panel-current-label">Current</span>
                <span data-role="main-current-value">--</span>
              </div>
              <div class="main-nav-buttons">
                <button type="button" class="nav-button" data-action="prev">Prev</button>
                <button type="button" class="nav-button" data-action="next">Next</button>
              </div>
            </div>
          </div>
          <div class="panel-body">
            <div class="main-chart-layout">
              <div class="main-chart">
                <canvas data-role="main-canvas"></canvas>
              </div>
            </div>
          </div>
          <button type="button" class="panel-resize-handle" data-action="resize-panel" aria-label="resize main panel"></button>
        </article>
      `).join("");

      appState.mainRuntime = Array.from(container.querySelectorAll(".main-panel")).map((panel, index) => {
        const canvas = panel.querySelector("[data-role='main-canvas']");
        const titleNode = panel.querySelector("[data-role='main-title']");
        const currentNode = panel.querySelector("[data-role='main-current-value']");
        const overlayCurrentNode = panel.querySelector("[data-role='main-overlay-current']");
        const overlayLabelNode = panel.querySelector("[data-role='main-overlay-label']");
        const overlayValueNode = panel.querySelector("[data-role='main-overlay-value']");
        const modeBadgeNode = panel.querySelector("[data-role='main-mode-badge']");
        const modeButtons = Array.from(panel.querySelectorAll("[data-role='main-mode-segment'] button"));
        const maControl = panel.querySelector("[data-role='main-ma-control']");
        const maValueNode = panel.querySelector("[data-role='main-ma-value']");
        const panelHead = panel.querySelector(".panel-head");
        const resizeHandle = panel.querySelector("[data-action='resize-panel']");
        if (panelHead) {
          panelHead.addEventListener("pointerdown", (event) => startMainPanelDrag(event, index));
        }
        if (resizeHandle) {
          resizeHandle.addEventListener("pointerdown", (event) => startMainPanelResize(event, index));
        }
        panel.querySelector("[data-action='prev']").addEventListener("click", () => stepMainPanel(index, -1));
        panel.querySelector("[data-action='next']").addEventListener("click", () => stepMainPanel(index, 1));
        modeButtons.forEach((button) => {
          button.addEventListener("click", () => setMainPanelMode(index, button.dataset.mode));
        });
        panel.querySelector("[data-action='ma-dec']").addEventListener("click", () =>
          adjustMainPanelMovingAverage(index, -MAIN_MA_STEP_SECONDS)
        );
        panel.querySelector("[data-action='ma-inc']").addEventListener("click", () =>
          adjustMainPanelMovingAverage(index, MAIN_MA_STEP_SECONDS)
        );
        return {
          panel,
          canvas,
          titleNode,
          currentNode,
          overlayCurrentNode,
          overlayLabelNode,
          overlayValueNode,
          modeBadgeNode,
          modeButtons,
          maControl,
          maValueNode,
          domain: null,
          signature: null,
        };
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

    function computeSeriesStats(series) {
      if (!Array.isArray(series) || series.length === 0) {
        return null;
      }
      let min = Infinity;
      let max = -Infinity;
      let sum = 0;
      let count = 0;
      let latest = NaN;
      for (const sample of series) {
        if (!Number.isFinite(sample?.value)) {
          continue;
        }
        if (sample.value < min) min = sample.value;
        if (sample.value > max) max = sample.value;
        sum += sample.value;
        latest = sample.value;
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

    function buildSeries(samples, config, chartIndex, mode, movingAverageSeconds, sourceKey = "values") {
      const sourceValues = [];
      for (const sample of samples) {
        const rawValues = sample?.[sourceKey];
        const value = toDisplayValue(rawValues?.[chartIndex], config);
        if (!Number.isFinite(value)) {
          continue;
        }
        sourceValues.push({ t: sample.t, value });
      }

      if (sourceValues.length === 0) {
        return { series: [], stats: null };
      }

      if (mode === "mean") {
        const sum = sourceValues.reduce((acc, sample) => acc + sample.value, 0);
        const mean = sum / sourceValues.length;
        const series = sourceValues.map((sample) => ({ t: sample.t, value: mean }));
        return { series, stats: computeSeriesStats(series) };
      }

      if (mode === "ma") {
        const windowSeconds = clamp(
          Number.isFinite(movingAverageSeconds) ? movingAverageSeconds : DEFAULT_MAIN_MA_SECONDS,
          MAIN_MA_MIN_SECONDS,
          MAIN_MA_MAX_SECONDS
        );
        const series = [];
        let left = 0;
        let sum = 0;
        let count = 0;
        for (let right = 0; right < sourceValues.length; right += 1) {
          const sample = sourceValues[right];
          sum += sample.value;
          count += 1;
          while (sourceValues[right].t - sourceValues[left].t > windowSeconds) {
            sum -= sourceValues[left].value;
            count -= 1;
            left += 1;
          }
          series.push({ t: sample.t, value: sum / Math.max(count, 1) });
        }
        return { series, stats: computeSeriesStats(series) };
      }

      const series = sourceValues.map((sample) => ({ t: sample.t, value: sample.value }));
      return { series, stats: computeSeriesStats(series) };
    }

    function buildOverlaySeries(samples, overlay, mode, movingAverageSeconds) {
      if (!overlay) {
        return { series: [], stats: null };
      }
      const config = {
        label: overlay.label,
        title: overlay.title,
        unit: CHART_CONFIGS.find((entry) => entry.label === overlay.baseLabel)?.unit || "m/s",
        color: overlay.color,
        scale: "symmetric",
        min_span: 1.0,
        precision: 3,
      };
      return buildSeries(samples, config, overlay.index, mode, movingAverageSeconds, "overlays");
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

      return null;
    }

    function drawChart(
      canvas,
      samples,
      config,
      state,
      focused,
      mode = "raw",
      movingAverageSeconds = DEFAULT_MAIN_MA_SECONDS,
      overlay = null
    ) {
      const resolvedCanvas = resolveCanvas(canvas, state, focused);
      if (!resolvedCanvas) {
        return null;
      }

      const renderedConfig = displayConfig(config);

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

      const chartIndex = Number.isInteger(state?.index)
        ? state.index
        : CHART_CONFIGS.findIndex((entry) => entry.label === config.label);
      const latestTime = samples.length > 0 ? samples[samples.length - 1].t : 0;
      const startTime = latestTime - appState.windowSeconds;
      const seriesInfo = buildSeries(samples, config, chartIndex, mode, movingAverageSeconds);
      const overlayInfo = buildOverlaySeries(samples, overlay, mode, movingAverageSeconds);
      const combinedSeries = overlayInfo.series.length > 0 ? seriesInfo.series.concat(overlayInfo.series) : seriesInfo.series;
      const combinedStats = computeSeriesStats(combinedSeries);
      const targetDomain = buildTargetDomain(combinedStats, renderedConfig);
      state.domain = settleDomain(state.domain, targetDomain, renderedConfig);
      const niceDomain = buildNiceDomain(state.domain, renderedConfig, focused ? 6 : 5);

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

      const basePoints = [];
      for (const sample of seriesInfo.series) {
        const x = plot.x + ((sample.t - startTime) / appState.windowSeconds) * plot.w;
        const y = plot.y + plot.h - ((sample.value - niceDomain.min) / (niceDomain.max - niceDomain.min)) * plot.h;
        basePoints.push({ x, y, value: sample.value });
      }

      const overlayPoints = [];
      for (const sample of overlayInfo.series) {
        const x = plot.x + ((sample.t - startTime) / appState.windowSeconds) * plot.w;
        const y = plot.y + plot.h - ((sample.value - niceDomain.min) / (niceDomain.max - niceDomain.min)) * plot.h;
        overlayPoints.push({ x, y, value: sample.value });
      }

      if (basePoints.length === 0 && overlayPoints.length === 0) {
        drawEmptyState(ctx, plot, config);
        drawAxes(ctx, plot, niceDomain, xTicks, yTicks, config, focused);
        return;
      }

      if (focused && basePoints.length > 1) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(basePoints[0].x, basePoints[0].y);
        for (let i = 1; i < basePoints.length; i += 1) {
          ctx.lineTo(basePoints[i].x, basePoints[i].y);
        }
        ctx.lineTo(basePoints[basePoints.length - 1].x, plot.y + plot.h);
        ctx.lineTo(basePoints[0].x, plot.y + plot.h);
        ctx.closePath();
        const fillGradient = ctx.createLinearGradient(0, plot.y, 0, plot.y + plot.h);
        fillGradient.addColorStop(0, hexToRgba(config.color, 0.28));
        fillGradient.addColorStop(1, hexToRgba(config.color, 0.03));
        ctx.fillStyle = fillGradient;
        ctx.fill();
        ctx.restore();
      }

      if (basePoints.length > 0) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(basePoints[0].x, basePoints[0].y);
        for (let i = 1; i < basePoints.length; i += 1) {
          ctx.lineTo(basePoints[i].x, basePoints[i].y);
        }
        ctx.strokeStyle = config.color;
        ctx.lineWidth = focused ? 2.8 : 2.0;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.shadowColor = hexToRgba(config.color, focused ? 0.32 : 0.20);
        ctx.shadowBlur = focused ? 18 : 10;
        ctx.stroke();
        ctx.restore();

        const lastPoint = basePoints[basePoints.length - 1];
        ctx.save();
        ctx.beginPath();
        ctx.arc(lastPoint.x, lastPoint.y, focused ? 4.2 : 3.4, 0, Math.PI * 2);
        ctx.fillStyle = config.color;
        ctx.shadowColor = hexToRgba(config.color, 0.40);
        ctx.shadowBlur = focused ? 18 : 10;
        ctx.fill();
        ctx.restore();
      }

      if (overlayPoints.length > 0) {
        ctx.save();
        ctx.beginPath();
        ctx.setLineDash(focused ? [8, 5] : [6, 5]);
        ctx.moveTo(overlayPoints[0].x, overlayPoints[0].y);
        for (let i = 1; i < overlayPoints.length; i += 1) {
          ctx.lineTo(overlayPoints[i].x, overlayPoints[i].y);
        }
        ctx.strokeStyle = hexToRgba(overlay.color, focused ? 0.92 : 0.80);
        ctx.lineWidth = focused ? 2.2 : 1.7;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.shadowColor = hexToRgba(overlay.color, focused ? 0.24 : 0.14);
        ctx.shadowBlur = focused ? 12 : 8;
        ctx.stroke();
        ctx.restore();

        const lastOverlayPoint = overlayPoints[overlayPoints.length - 1];
        ctx.save();
        ctx.beginPath();
        ctx.arc(lastOverlayPoint.x, lastOverlayPoint.y, focused ? 3.1 : 2.5, 0, Math.PI * 2);
        ctx.fillStyle = hexToRgba(overlay.color, 0.90);
        ctx.shadowColor = hexToRgba(overlay.color, 0.22);
        ctx.shadowBlur = focused ? 12 : 8;
        ctx.fill();
        ctx.restore();
      }

      drawAxes(ctx, plot, niceDomain, xTicks, yTicks, config, focused);
      return {
        stats: seriesInfo.stats,
        overlayStats: overlayInfo.stats,
        combinedStats,
        niceDomain,
      };
    }

    function drawEmptyState(ctx, plot, config) {
      ctx.save();
      ctx.fillStyle = "rgba(255, 255, 255, 0.34)";
      ctx.font = "600 12px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      const renderedConfig = displayConfig(config);
      ctx.fillText(`Waiting for ${config.title.toLowerCase()} samples`, plot.x + plot.w / 2, plot.y + plot.h / 2);
      ctx.restore();
    }

    function latestCommandValues() {
      const state = appState.latestSnapshot?.state;
      if (!Array.isArray(state)) {
        return null;
      }
      const xDot = Number(state[COMMAND_STATE_INDEX.xDot]);
      const yDot = Number(state[COMMAND_STATE_INDEX.yDot]);
      const psiDot = Number(state[COMMAND_STATE_INDEX.psiDot]);
      if (![xDot, yDot, psiDot].every(Number.isFinite)) {
        return null;
      }
      return { xDot, yDot, psiDot };
    }

    function drawArrow(ctx, startX, startY, endX, endY, color) {
      const angle = Math.atan2(endY - startY, endX - startX);
      const head = 10;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 4;
      ctx.lineCap = "round";
      ctx.shadowColor = "rgba(125, 211, 252, 0.34)";
      ctx.shadowBlur = 14;
      ctx.beginPath();
      ctx.moveTo(startX, startY);
      ctx.lineTo(endX, endY);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(endX, endY);
      ctx.lineTo(endX - Math.cos(angle - Math.PI / 6) * head, endY - Math.sin(angle - Math.PI / 6) * head);
      ctx.lineTo(endX - Math.cos(angle + Math.PI / 6) * head, endY - Math.sin(angle + Math.PI / 6) * head);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    function drawYawArc(ctx, cx, cy, radius, psiDot, maxYaw) {
      const amount = clamp(Math.abs(psiDot) / maxYaw, 0, 1);
      const sweep = Math.max(0.12, amount * Math.PI * 1.45);
      const direction = psiDot >= 0 ? -1 : 1;
      const start = -Math.PI / 2;
      const end = start + direction * sweep;
      const color = psiDot >= 0 ? "rgba(244, 114, 182, 0.94)" : "rgba(251, 191, 36, 0.94)";

      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 5;
      ctx.lineCap = "round";
      ctx.shadowColor = color.replace("0.94", "0.24");
      ctx.shadowBlur = 12;
      ctx.beginPath();
      ctx.arc(cx, cy, radius, start, end, direction < 0);
      ctx.stroke();

      const tangent = end + direction * Math.PI / 2;
      const tipX = cx + Math.cos(end) * radius;
      const tipY = cy + Math.sin(end) * radius;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(tipX - Math.cos(tangent - Math.PI / 5) * 9, tipY - Math.sin(tangent - Math.PI / 5) * 9);
      ctx.lineTo(tipX - Math.cos(tangent + Math.PI / 5) * 9, tipY - Math.sin(tangent + Math.PI / 5) * 9);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    function drawCommandCompass() {
      const canvas = document.getElementById("command-compass-canvas");
      const prepared = prepareCanvas(canvas);
      if (!prepared) {
        return;
      }

      const { ctx, width, height } = prepared;
      const size = Math.min(width, height);
      const cx = width / 2;
      const cy = height / 2;
      const radius = Math.max(42, size * 0.34);
      const innerRadius = radius * 0.72;
      const values = latestCommandValues();
      const xDot = values?.xDot ?? 0;
      const yDot = values?.yDot ?? 0;
      const psiDot = values?.psiDot ?? 0;
      const speed = values ? Math.hypot(xDot, yDot) : NaN;
      const maxLinear = 0.7;
      const maxYaw = Math.max(1.5, Math.abs(psiDot) * 1.18);
      const vectorScale = clamp(Math.hypot(xDot, yDot) / maxLinear, 0, 1);
      const endX = cx + (yDot / maxLinear) * innerRadius;
      const endY = cy - (xDot / maxLinear) * innerRadius;

      const bg = ctx.createRadialGradient(cx, cy, radius * 0.1, cx, cy, radius * 1.22);
      bg.addColorStop(0, "rgba(125, 211, 252, 0.10)");
      bg.addColorStop(1, "rgba(255, 255, 255, 0.015)");
      ctx.fillStyle = bg;
      ctx.beginPath();
      ctx.arc(cx, cy, radius * 1.18, 0, Math.PI * 2);
      ctx.fill();

      ctx.save();
      ctx.strokeStyle = "rgba(255, 255, 255, 0.12)";
      ctx.lineWidth = 1;
      for (const scale of [0.38, 0.68, 1.0]) {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * scale, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(cx - radius, cy);
      ctx.lineTo(cx + radius, cy);
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx, cy + radius);
      ctx.stroke();
      ctx.restore();

      ctx.save();
      ctx.fillStyle = "rgba(255, 255, 255, 0.42)";
      ctx.font = "700 10px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("+x", cx, cy - radius - 11);
      ctx.fillText("-x", cx, cy + radius + 12);
      ctx.fillText("-y", cx + radius + 13, cy);
      ctx.fillText("+y", cx - radius - 13, cy);
      ctx.restore();

      drawYawArc(ctx, cx, cy, radius * 1.05, psiDot, maxYaw);

      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(237, 244, 251, 0.92)";
      ctx.fill();
      ctx.restore();

      if (values && vectorScale > 0.01) {
        drawArrow(ctx, cx, cy, endX, endY, "rgba(125, 211, 252, 0.96)");
      } else {
        ctx.save();
        ctx.strokeStyle = "rgba(125, 211, 252, 0.38)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(cx, cy, 11, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }

      ctx.save();
      ctx.fillStyle = values ? "rgba(237, 244, 251, 0.84)" : "rgba(237, 244, 251, 0.38)";
      ctx.font = "650 11px Avenir Next, SF Pro Text, Segoe UI, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(values ? `${formatNumber(speed, 2)} m/s` : "waiting", cx, cy + radius * 0.48);
      ctx.restore();

      setTextContent("command-speed", values ? `${formatNumber(speed, 2)} m/s` : "-- m/s");
      setTextContent("command-x-dot", values ? `${formatNumber(xDot, 3)} m/s` : "--");
      setTextContent("command-y-dot", values ? `${formatNumber(yDot, 3)} m/s` : "--");
      const psiDisplay = appState.angleUnit === "deg" ? psiDot * 180 / Math.PI : psiDot;
      setTextContent("command-psi-dot", values ? `${formatNumber(psiDisplay, appState.angleUnit === "deg" ? 1 : 3)} ${appState.angleUnit}/s` : "--");
    }

    function drawAxes(ctx, plot, yDomain, xTicks, yTicks, config, focused) {
      const renderedConfig = displayConfig(config);
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
      ctx.fillText(renderedConfig.unit, plot.x, plot.y - 12);
      ctx.restore();
    }

    function updateChartCard(entry, stats, drawResult, visible) {
      const { config, card } = entry;
      const renderedConfig = displayConfig(config);
      if (!card) {
        return;
      }
      card.style.display = visible ? "" : "none";
      card.classList.toggle("active", appState.mainLabels.includes(config.label));
      const displayStats = toDisplayStats(stats, config);
      if (displayStats) {
        entry.valueNode.textContent = formatValue(displayStats.latest, renderedConfig.unit, renderedConfig.precision);
        entry.rangeNode.textContent = `range ${formatTick(displayStats.min, drawResult.niceDomain.step)} … ${formatTick(displayStats.max, drawResult.niceDomain.step)} ${renderedConfig.unit}`;
        entry.gridNode.textContent = `grid ${formatTick(drawResult.niceDomain.step, drawResult.niceDomain.step)} ${renderedConfig.unit}`;
      } else {
        entry.valueNode.textContent = "--";
        entry.rangeNode.textContent = "range --";
        entry.gridNode.textContent = "grid --";
      }
    }

    function updateMainPanel(panelEntry, label, drawResult, panelState, panelIndex) {
      const config = CHART_CONFIGS.find((entry) => entry.label === label) || CHART_CONFIGS[0];
      const renderedConfig = displayConfig(config);
      const overlay = COMMAND_SERIES_BY_LABEL[label] || null;
      if (!panelEntry) {
        return;
      }

      const signature = `${label}:${panelState.mode}:${panelState.mode === "ma" ? panelState.movingAverageSeconds : ""}`;
      if (panelEntry.signature !== signature) {
        panelEntry.signature = signature;
        panelEntry.domain = null;
      }

      panelEntry.panel.style.display = "";
      panelEntry.titleNode.textContent = config.title;
      if (panelEntry.modeBadgeNode) {
        panelEntry.modeBadgeNode.textContent =
          panelState.mode === "ma"
            ? `${MAIN_PANEL_MODES[panelState.mode]} ${formatDurationSeconds(panelState.movingAverageSeconds)}`
            : MAIN_PANEL_MODES[panelState.mode];
      }
      panelEntry.modeButtons?.forEach((button) => {
        button.classList.toggle("active", button.dataset.mode === panelState.mode);
      });
      if (panelEntry.maControl) {
        panelEntry.maControl.classList.toggle("hidden", panelState.mode !== "ma");
      }
      if (panelEntry.maValueNode) {
        panelEntry.maValueNode.textContent = formatDurationSeconds(panelState.movingAverageSeconds);
      }
      if (panelEntry.currentNode) {
        panelEntry.currentNode.textContent = drawResult.stats
          ? formatValue(drawResult.stats.latest, renderedConfig.unit, renderedConfig.precision)
          : "--";
      }
      if (panelEntry.overlayCurrentNode) {
        const showOverlay = Boolean(overlay && drawResult.overlayStats);
        panelEntry.overlayCurrentNode.classList.toggle("hidden", !showOverlay);
        if (showOverlay) {
          if (panelEntry.overlayLabelNode) {
            panelEntry.overlayLabelNode.textContent = overlay.valueLabel || "Target";
          }
          if (panelEntry.overlayValueNode) {
            panelEntry.overlayValueNode.textContent = formatValue(
              drawResult.overlayStats.latest,
              renderedConfig.unit,
              renderedConfig.precision
            );
          }
        } else if (panelEntry.overlayValueNode) {
          panelEntry.overlayValueNode.textContent = "--";
        }
      }
    }

    function updateConnectionPanel() {
      const data = appState.latestSnapshot;
      const connected = Boolean(data && data.connected);
      const status = connected ? (data.status || "connected") : (data?.status || "waiting");
      const dot = document.getElementById("status-dot");
      const statusText = document.getElementById("status-text");
      const controllerBadge = document.getElementById("controller-status-badge");
      const controllerDot = document.getElementById("controller-status-dot");
      const controllerText = document.getElementById("controller-status-text");
      const heroSubtitle = document.getElementById("hero-subtitle");
      const nowSeconds = Number(data?.server_time ?? (performance.now() / 1000));

      dot.className = "dot";
      if (connected && status === "live") {
        dot.classList.add("live");
        statusText.textContent = "memory live";
        heroSubtitle.textContent = `Receiving live samples from ${data.robot_name || "the controller"} through ${data.shared_memory_name}.`;
      } else if (connected && status === "busy") {
        dot.classList.add("busy");
        statusText.textContent = "memory busy";
        heroSubtitle.textContent = data.message || "Controller is updating the shared buffer.";
      } else if (connected) {
        dot.classList.add("live");
        statusText.textContent = `memory ${status}`;
        heroSubtitle.textContent = data.message || "Connected to the shared memory segment.";
      } else if (status === "error") {
        dot.classList.add("error");
        statusText.textContent = "memory error";
        heroSubtitle.textContent = data.message || "Unable to connect to the dashboard shared memory.";
      } else {
        dot.classList.add("busy");
        statusText.textContent = "memory waiting";
        heroSubtitle.textContent = data?.message || "Waiting for the controller to publish samples.";
      }

      if (controllerBadge && controllerDot && controllerText) {
        const sequence = Number(data?.sequence ?? 0);
        const activeThresholdSeconds = 0.35;
        let controllerState = "waiting";
        let controllerLabel = "controller waiting";

        if (status === "error") {
          controllerState = "error";
          controllerLabel = "controller offline";
        } else if (connected && status === "busy") {
          controllerState = "busy";
          controllerLabel = "controller busy";
        } else if (!connected) {
          controllerState = "waiting";
          controllerLabel = "controller waiting";
        } else if (sequence <= 0) {
          appState.controllerLastSequence = 0;
          appState.controllerLastChangeAt = null;
          controllerState = "waiting";
          controllerLabel = "controller starting";
        } else {
          if (appState.controllerLastSequence !== sequence) {
            appState.controllerLastSequence = sequence;
            appState.controllerLastChangeAt = nowSeconds;
          }
          const lastChangeAt = appState.controllerLastChangeAt;
          const ageSeconds =
            Number.isFinite(lastChangeAt) ? Math.max(0, nowSeconds - lastChangeAt) : Number.POSITIVE_INFINITY;
          if (ageSeconds <= activeThresholdSeconds) {
            controllerState = "active";
            controllerLabel = "controller active";
          } else {
            controllerState = "stale";
            controllerLabel = "controller idle";
          }
        }

        controllerBadge.className = `status-badge controller ${controllerState}`;
        controllerDot.className = "dot";
        if (controllerState === "active") {
          controllerDot.classList.add("live");
        } else if (controllerState === "stale") {
          controllerDot.classList.add("busy");
        } else if (controllerState === "error") {
          controllerDot.classList.add("error");
        } else {
          controllerDot.classList.add("busy");
        }
        controllerText.textContent = controllerLabel;
      }

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
        values: data.state.slice(0, BASE_CHART_COUNT),
        overlays: data.state.slice(BASE_CHART_COUNT),
      });
      pruneHistory();
      return true;
    }

    function renderCharts() {
      applyMainPanelLayouts();
      const visibleConfigs = visibleChartsForView();
      const visibleLabels = new Set(visibleConfigs.map((config) => config.label));
      const historyWindow = appState.history.filter((sample) => sample.t >= (appState.history.length > 0 ? appState.history[appState.history.length - 1].t - appState.windowSeconds : 0));
      const mainLabels = appState.mainLabels.slice(0, MAX_MAIN_PANELS);

      document.getElementById("chart-grid").dataset.view = appState.view;

      const statsByIndex = CHART_ORDER.map((entry) => computeStats(historyWindow, entry.index));

      for (const orderEntry of CHART_ORDER) {
        const runtimeEntry = getChartEntry(orderEntry.config.label);
        if (!runtimeEntry) {
          continue;
        }
        const stats = statsByIndex[orderEntry.index];
        const visible = visibleLabels.has(orderEntry.config.label);
        const overlay = COMMAND_SERIES_BY_LABEL[orderEntry.config.label] || null;
        updateChartCard(
          runtimeEntry,
          stats,
          drawChart(runtimeEntry.canvas,
                    historyWindow,
                    orderEntry.config,
                    runtimeEntry,
                    false,
                    "raw",
                    DEFAULT_MAIN_MA_SECONDS,
                    overlay) || { niceDomain: { step: 1 } },
          visible
        );
      }

      appState.mainRuntime.forEach((panelEntry, index) => {
        const label = mainLabels[index];
        if (!panelEntry || !label) {
          if (panelEntry) {
            panelEntry.panel.style.display = "none";
          }
          return;
        }
        const config = CHART_CONFIGS.find((entry) => entry.label === label) || CHART_CONFIGS[0];
        const chartEntry = getChartEntry(label);
        const panelState = getMainPanelState(index);
        if (!chartEntry) {
          panelEntry.panel.style.display = "none";
          return;
        }
        const overlay = COMMAND_SERIES_BY_LABEL[label] || null;
        panelEntry.panel.style.display = "";
        panelEntry.titleNode.textContent = config.title;
        const mode = panelState.mode;
        const movingAverageSeconds = panelState.movingAverageSeconds;
        const cardDraw =
          drawChart(chartEntry.canvas,
                    historyWindow,
                    config,
                    chartEntry,
                    false,
                    "raw",
                    DEFAULT_MAIN_MA_SECONDS,
                    overlay) || { niceDomain: { step: 1 } };
        const mainDraw =
          drawChart(panelEntry.canvas,
                    historyWindow,
                    config,
                    panelEntry,
                    true,
                    mode,
                    movingAverageSeconds,
                    overlay) || cardDraw;
        updateMainPanel(panelEntry, label, mainDraw, panelState, index);
      });

      for (const orderEntry of CHART_ORDER) {
        const runtimeEntry = getChartEntry(orderEntry.config.label);
        const card = runtimeEntry?.card;
        if (!card) continue;
        card.classList.toggle("active", mainLabels.includes(orderEntry.config.label));
        card.style.display = visibleLabels.has(orderEntry.config.label) ? "" : "none";
      }
    }

    function render() {
      updateConnectionPanel();
      drawCommandCompass();
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
      updateConnectionPanel();
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
      document.querySelectorAll("#angle-buttons button").forEach((button) => {
        button.addEventListener("click", () => setAngleUnit(button.dataset.angleUnit));
      });
      const decButton = document.getElementById("main-count-dec");
      const incButton = document.getElementById("main-count-inc");
      if (decButton) {
        decButton.addEventListener("click", () => setMainPanelCount(appState.mainLabels.length - 1));
      }
      if (incButton) {
        incButton.addEventListener("click", () => setMainPanelCount(appState.mainLabels.length + 1));
      }
      window.addEventListener("resize", render);
    }

    buildMainPanels();
    loadMainLayouts();
    applyMainPanelLayouts();
    buildChartCards();
    bindControls();
    updateButtonStates();
    safeRender();
    window.requestAnimationFrame(() => safeRender());
    void pollSnapshot();
