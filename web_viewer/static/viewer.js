import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import loadMujoco from "/node_modules/mujoco-js/dist/mujoco_wasm.js";

const GEOM = {
  PLANE: 0,
  SPHERE: 2,
  CAPSULE: 3,
  CYLINDER: 5,
  BOX: 6,
  MESH: 7,
};

const canvas = document.getElementById("viewer");
const statusEl = document.getElementById("viewer-status") || document.getElementById("status");
const visualButton = document.getElementById("toggle-visual");
const collisionButton = document.getElementById("toggle-collision");
const params = new URLSearchParams(window.location.search);
const viewerState = {
  showVisual: params.get("visual") !== "0",
  showCollision: params.get("collision") === "1",
};
const showDebug = params.get("debug") === "1";

const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: true,
  alpha: false,
  powerPreference: "high-performance",
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x111318, 1);

const scene = new THREE.Scene();
scene.up.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(1.6, -2.2, 1.25);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.58);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.screenSpacePanning = false;

scene.add(new THREE.HemisphereLight(0xffffff, 0x303040, 1.8));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.4);
keyLight.position.set(2.5, -3.5, 4.0);
scene.add(keyLight);

const root = new THREE.Group();
scene.add(root);

let mujoco;
let model;
let data;
let sceneXmlText = "";
let latestQpos = null;
let latestInfo = null;
let latestSequence = -1;
let processedSequence = -1;
let geoms = [];
let groundTexture = null;
let lastFrame = performance.now();
let frameCount = 0;
let fps = 0;

function setStatus(text) {
  statusEl.textContent = text;
}

function value(vec, index, fallback = 0) {
  const result = typeof vec?.get === "function" ? vec.get(index) : vec?.[index];
  return Number.isFinite(result) ? result : fallback;
}

function setValue(vec, index, nextValue) {
  if (typeof vec?.set === "function" && !ArrayBuffer.isView(vec)) {
    vec.set(index, nextValue);
  } else if (vec) {
    vec[index] = nextValue;
  }
}

function rgba(model, geomId) {
  const base = 4 * geomId;
  return [
    value(model.geom_rgba, base + 0, 0.7),
    value(model.geom_rgba, base + 1, 0.7),
    value(model.geom_rgba, base + 2, 0.7),
    value(model.geom_rgba, base + 3, 1.0),
  ];
}

function geomSize(model, geomId, offset) {
  return value(model.geom_size, 3 * geomId + offset, 0);
}

function makeMaterial(model, geomId, group) {
  const type = value(model.geom_type, geomId, -1);
  if (type === GEOM.PLANE && group === 2) {
    return makeGroundMaterial(model, geomId);
  }
  const material = makeMuJoCoMaterial(model, geomId, { fallbackGround: false });
  if (material) {
    return material;
  }

  const color = rgba(model, geomId);
  const alpha = group === 3 ? Math.max(color[3], 0.55) : color[3];
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(color[0], color[1], color[2]),
    roughness: 0.55,
    metalness: 0.05,
    transparent: alpha < 0.999,
    opacity: alpha,
    depthWrite: alpha >= 0.5,
  });
}

function makeGroundMaterial(model, geomId) {
  const matId = value(model.geom_matid, geomId, -1);
  return new THREE.MeshStandardMaterial({
    map: checkerTextureFromSceneXml(model, matId),
    roughness: 0.62,
    metalness: 0.0,
  });
}

function makeMuJoCoMaterial(model, geomId, options) {
  const matId = value(model.geom_matid, geomId, -1);
  if (matId < 0) {
    return options.fallbackGround ? makeFallbackGroundMaterial() : null;
  }

  const texId = value(model.mat_texid, matId, -1);
  const rgbaBase = 4 * matId;
  const color = new THREE.Color(
    value(model.mat_rgba, rgbaBase + 0, 0.7),
    value(model.mat_rgba, rgbaBase + 1, 0.7),
    value(model.mat_rgba, rgbaBase + 2, 0.7),
  );
  const alpha = value(model.mat_rgba, rgbaBase + 3, 1.0);
  const roughness = clamp01(value(model.mat_roughness, matId, 0.55));
  const metallic = clamp01(value(model.mat_metallic, matId, 0.0));
  const reflectance = clamp01(value(model.mat_reflectance, matId, 0.0));
  const texture = texId >= 0 ? textureFromMuJoCo(model, texId, matId) : null;

  return new THREE.MeshStandardMaterial({
    color,
    map: texture,
    roughness: Math.max(0.08, roughness * (1.0 - 0.35 * reflectance)),
    metalness: metallic,
    transparent: alpha < 0.999,
    opacity: alpha,
    depthWrite: alpha >= 0.5,
  });
}

function clamp01(value) {
  return Math.max(0, Math.min(1, Number.isFinite(value) ? value : 0));
}

function textureFromMuJoCo(model, texId, matId) {
  if (groundTexture) {
    return groundTexture;
  }
  const width = value(model.tex_width, texId, 0);
  const height = value(model.tex_height, texId, 0);
  const channels = value(model.tex_nchannel, texId, 0);
  const adr = value(model.tex_adr, texId, -1);
  if (width <= 0 || height <= 0 || channels <= 0 || adr < 0) {
    return null;
  }

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  const image = ctx.createImageData(width, height);
  for (let i = 0; i < width * height; ++i) {
    const source = adr + i * channels;
    const dest = i * 4;
    if (channels === 1) {
      const luminance = value(model.tex_data, source, 255);
      image.data[dest + 0] = luminance;
      image.data[dest + 1] = luminance;
      image.data[dest + 2] = luminance;
      image.data[dest + 3] = 255;
    } else {
      image.data[dest + 0] = value(model.tex_data, source + 0, 255);
      image.data[dest + 1] = value(model.tex_data, source + 1, 255);
      image.data[dest + 2] = value(model.tex_data, source + 2, 255);
      image.data[dest + 3] = channels >= 4 ? value(model.tex_data, source + 3, 255) : 255;
    }
  }
  ctx.putImageData(image, 0, 0);

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(
    Math.max(value(model.mat_texrepeat, 2 * matId + 0, 1), 1),
    Math.max(value(model.mat_texrepeat, 2 * matId + 1, 1), 1),
  );
  texture.colorSpace = THREE.SRGBColorSpace;
  groundTexture = texture;
  return texture;
}

function makeFallbackGroundMaterial() {
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(0.2, 0.3, 0.4),
    roughness: 0.7,
    metalness: 0.0,
  });
}

function checkerTextureFromSceneXml(model, matId) {
  if (groundTexture) {
    return groundTexture;
  }

  const parser = new DOMParser();
  const xml = parser.parseFromString(sceneXmlText, "application/xml");
  const textureNode = xml.querySelector('texture[name="groundplane"]');
  const materialNode = xml.querySelector('material[name="groundplane"]');
  const rgb1 = parseRgb(textureNode?.getAttribute("rgb1"), [0.2, 0.3, 0.4]);
  const rgb2 = parseRgb(textureNode?.getAttribute("rgb2"), [0.1, 0.2, 0.3]);
  const markRgb = parseRgb(textureNode?.getAttribute("markrgb"), [0.8, 0.8, 0.8]);
  const repeat = parseRgb(materialNode?.getAttribute("texrepeat"), [
    Math.max(value(model.mat_texrepeat, 2 * matId + 0, 5), 1),
    Math.max(value(model.mat_texrepeat, 2 * matId + 1, 5), 1),
    1,
  ]);

  const size = 512;
  const cells = 10;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const cell = size / cells;
  for (let y = 0; y < cells; ++y) {
    for (let x = 0; x < cells; ++x) {
      ctx.fillStyle = colorString((x + y) % 2 === 0 ? rgb1 : rgb2);
      ctx.fillRect(x * cell, y * cell, cell, cell);
    }
  }
  ctx.strokeStyle = colorString(markRgb);
  ctx.lineWidth = 3;
  for (let i = 0; i <= cells; ++i) {
    ctx.beginPath();
    ctx.moveTo(i * cell, 0);
    ctx.lineTo(i * cell, size);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, i * cell);
    ctx.lineTo(size, i * cell);
    ctx.stroke();
  }

  groundTexture = new THREE.CanvasTexture(canvas);
  groundTexture.wrapS = THREE.RepeatWrapping;
  groundTexture.wrapT = THREE.RepeatWrapping;
  groundTexture.repeat.set(Math.max(repeat[0], 1), Math.max(repeat[1], 1));
  groundTexture.colorSpace = THREE.SRGBColorSpace;
  return groundTexture;
}

function parseRgb(value, fallback) {
  if (!value) {
    return fallback;
  }
  const parsed = value.trim().split(/\s+/).map(Number);
  return parsed.length >= 2 && parsed.every(Number.isFinite) ? parsed : fallback;
}

function colorString(rgb) {
  const channels = rgb.slice(0, 3).map((channel) =>
    Math.max(0, Math.min(255, Math.round(channel * 255))),
  );
  return `rgb(${channels[0]}, ${channels[1]}, ${channels[2]})`;
}

function shouldRenderGeom(model, geomId) {
  const group = value(model.geom_group, geomId, 0);
  if (group === 1 && !viewerState.showVisual) {
    return false;
  }
  if (group === 3 && !viewerState.showCollision) {
    return false;
  }
  if (group === 4 && !showDebug) {
    return false;
  }
  return group === 1 || group === 2 || group === 3 || group === 4;
}

function makeGeometry(model, geomId) {
  const type = value(model.geom_type, geomId, -1);
  if (type === GEOM.PLANE) {
    return new THREE.PlaneGeometry(20, 20, 1, 1);
  }
  if (type === GEOM.SPHERE) {
    const radius = Math.max(geomSize(model, geomId, 0), 0.001);
    return new THREE.SphereGeometry(radius, 24, 16);
  }
  if (type === GEOM.BOX) {
    const sx = Math.max(2 * geomSize(model, geomId, 0), 0.001);
    const sy = Math.max(2 * geomSize(model, geomId, 1), 0.001);
    const sz = Math.max(2 * geomSize(model, geomId, 2), 0.001);
    return new THREE.BoxGeometry(sx, sy, sz);
  }
  if (type === GEOM.CYLINDER) {
    const radius = Math.max(geomSize(model, geomId, 0), 0.001);
    const halfHeight = Math.max(geomSize(model, geomId, 1), 0.001);
    const geometry = new THREE.CylinderGeometry(radius, radius, 2 * halfHeight, 24, 1);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === GEOM.CAPSULE) {
    const radius = Math.max(geomSize(model, geomId, 0), 0.001);
    const halfHeight = Math.max(geomSize(model, geomId, 1), 0.001);
    const geometry = new THREE.CapsuleGeometry(radius, Math.max(2 * halfHeight, 0.001), 12, 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === GEOM.MESH) {
    return makeMeshGeometry(model, geomId);
  }
  return null;
}

function makeMeshGeometry(model, geomId) {
  const meshId = value(model.geom_dataid, geomId, -1);
  if (meshId < 0) {
    return null;
  }
  const vertAdr = value(model.mesh_vertadr, meshId, 0);
  const vertNum = value(model.mesh_vertnum, meshId, 0);
  const faceAdr = value(model.mesh_faceadr, meshId, 0);
  const faceNum = value(model.mesh_facenum, meshId, 0);
  if (vertNum <= 0 || faceNum <= 0) {
    return null;
  }

  const positions = new Float32Array(faceNum * 9);
  let out = 0;
  for (let face = 0; face < faceNum; ++face) {
    for (let corner = 0; corner < 3; ++corner) {
      const vertex = value(model.mesh_face, 3 * (faceAdr + face) + corner, 0);
      const source = 3 * (vertAdr + vertex);
      positions[out++] = value(model.mesh_vert, source + 0, 0);
      positions[out++] = value(model.mesh_vert, source + 1, 0);
      positions[out++] = value(model.mesh_vert, source + 2, 0);
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function buildScene() {
  geoms = [];
  root.clear();

  for (let geomId = 0; geomId < model.ngeom; ++geomId) {
    if (!shouldRenderGeom(model, geomId)) {
      continue;
    }
    const geometry = makeGeometry(model, geomId);
    if (!geometry) {
      continue;
    }
    const mesh = new THREE.Mesh(geometry, makeMaterial(model, geomId, value(model.geom_group, geomId, 0)));
    mesh.matrixAutoUpdate = false;
    root.add(mesh);
    geoms.push({ geomId, mesh });
  }
  updateToggleButtons();
}

async function loadModelFiles() {
  const manifest = await fetchJson("/api/model-files");
  mujoco.FS.mkdir("/working");
  mujoco.FS.mount(mujoco.MEMFS, { root: "." }, "/working");
  mujoco.FS.chdir("/working");

  for (const file of manifest.files) {
    const parts = file.path.split("/");
    let dir = "/working";
    for (const part of parts.slice(0, -1)) {
      dir += `/${part}`;
      try {
        mujoco.FS.mkdir(dir);
      } catch {
        // Directory already exists.
      }
    }
    const response = await fetch(file.url, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`failed to fetch ${file.path}: ${response.status}`);
    }
    const content = file.binary
      ? new Uint8Array(await response.arrayBuffer())
      : await response.text();
    if (file.path === "scene.xml") {
      sceneXmlText = content;
    }
    mujoco.FS.writeFile(`/working/${file.path}`, content);
  }
}

async function fetchJson(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`${url} returned ${response.status}`);
  }
  return response.json();
}

function applyQpos(qpos) {
  if (!data || !qpos?.length) {
    return;
  }
  const count = Math.min(model.nq, qpos.length);
  for (let i = 0; i < count; ++i) {
    setValue(data.qpos, i, qpos[i]);
  }
  mujoco.mj_forward(model, data);
}

function updateGeomTransforms() {
  const matrix = new THREE.Matrix4();
  for (const { geomId, mesh } of geoms) {
    const p = 3 * geomId;
    const r = 9 * geomId;
    const x = value(data.geom_xpos, p + 0, 0);
    const y = value(data.geom_xpos, p + 1, 0);
    const z = value(data.geom_xpos, p + 2, 0);
    matrix.set(
      value(data.geom_xmat, r + 0, 1), value(data.geom_xmat, r + 1, 0), value(data.geom_xmat, r + 2, 0), x,
      value(data.geom_xmat, r + 3, 0), value(data.geom_xmat, r + 4, 1), value(data.geom_xmat, r + 5, 0), y,
      value(data.geom_xmat, r + 6, 0), value(data.geom_xmat, r + 7, 0), value(data.geom_xmat, r + 8, 1), z,
      0, 0, 0, 1,
    );
    mesh.matrix.copy(matrix);
  }
}

function connectQposStream() {
  const events = new EventSource("/events/qpos");
  events.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.ok) {
      latestQpos = data.qpos;
      latestInfo = data;
      latestSequence = data.sequence;
    } else {
      latestInfo = data;
    }
  };
  events.onerror = () => {
    latestInfo = { ok: false, message: "qpos stream disconnected" };
  };
}

function resize() {
  const canvasBounds = canvas.getBoundingClientRect();
  const parentBounds = canvas.parentElement?.getBoundingClientRect();
  const bounds = canvasBounds.width > 0 && canvasBounds.height > 0 ? canvasBounds : parentBounds;
  const width = Math.max(1, Math.floor(bounds.width || window.innerWidth));
  const height = Math.max(1, Math.floor(bounds.height || window.innerHeight));
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(height, 1);
  camera.updateProjectionMatrix();
}

function updateToggleButtons() {
  visualButton.classList.toggle("active", viewerState.showVisual);
  visualButton.setAttribute("aria-pressed", String(viewerState.showVisual));
  collisionButton.classList.toggle("active", viewerState.showCollision);
  collisionButton.setAttribute("aria-pressed", String(viewerState.showCollision));
}

visualButton.addEventListener("click", () => {
  viewerState.showVisual = !viewerState.showVisual;
  buildScene();
  updateGeomTransforms();
});

collisionButton.addEventListener("click", () => {
  viewerState.showCollision = !viewerState.showCollision;
  buildScene();
  updateGeomTransforms();
});

function animate(now) {
  requestAnimationFrame(animate);
  frameCount += 1;
  if (now - lastFrame > 500) {
    fps = Math.round((1000 * frameCount) / (now - lastFrame));
    lastFrame = now;
    frameCount = 0;
  }

  if (latestQpos && latestSequence !== processedSequence) {
    applyQpos(latestQpos);
    updateGeomTransforms();
    processedSequence = latestSequence;
  }

  controls.update();
  renderer.render(scene, camera);

  if (latestInfo?.ok) {
    const simTime = Number.isFinite(latestInfo.sim_time) ? latestInfo.sim_time.toFixed(3) : "--";
    setStatus(
      `${latestInfo.robot_name || "MIT Humanoid"} | sim ${simTime}s | fps ${fps}`,
    );
  } else {
    setStatus(`${latestInfo?.message || "waiting for qpos"} | fps ${fps}`);
  }
}

async function main() {
  resize();
  window.addEventListener("resize", resize);

  setStatus("loading MuJoCo WASM");
  mujoco = await loadMujoco();

  setStatus("loading MIT humanoid assets");
  await loadModelFiles();
  model = mujoco.MjModel.loadFromXML("/working/scene.xml");
  if (!model) {
    throw new Error("MjModel.loadFromXML returned null");
  }
  data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);

  setStatus("building scene");
  buildScene();
  updateGeomTransforms();
  connectQposStream();
  requestAnimationFrame(animate);
}

main().catch((error) => {
  console.error(error);
  setStatus(`viewer error: ${error.message || error}`);
});
