// three.js view of a replay: drones as LED points over the launch field.
// ENU (x east, y north, z up) maps to three.js as (x, z, -y).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";
import { locate, positionAt } from "./sampling";
import type { ReplayData, V3 } from "./types";

const NIGHT = 0x0e1a2b;
const GRID_MAJOR = 0x2a3b55;
const GRID_MINOR = 0x1c2a40;
const PAD = 0x3d5273;
const AMBER = 0xf2a93b;
const RED = 0xff5a4e;
const LED_OFF = new THREE.Color(0x5a6a85); // drone body when its LEDs are dark

const toThree = (p: V3, out = new THREE.Vector3()) => out.set(p[0], p[2], -p[1]);

export class ReplayScene {
  private renderer: THREE.WebGLRenderer;
  private labels: CSS2DRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private cores: THREE.InstancedMesh;
  private halos: THREE.InstancedMesh;
  private markers: THREE.InstancedMesh | null = null;
  // Fixed pixel size: keeps every drone visible when a 50 m show is zoomed out to specks.
  private dots: THREE.Points;
  private pairLine: THREE.Line;
  private pairLabel: CSS2DObject;
  private resizeObserver: ResizeObserver;
  private raycaster = new THREE.Raycaster();
  private highlight: number[] = [];
  private time: number;
  private frameRequested = false;
  private tmp = new THREE.Object3D();
  private color = new THREE.Color();
  private p: V3 = [0, 0, 0];
  private rightInset = 0; // px covered by an overlay panel; the view centres on the rest

  constructor(
    private container: HTMLElement,
    private data: ReplayData,
    private onPick: (drone: number | null) => void,
  ) {
    const { header } = data;
    this.time = header.t0;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(NIGHT);
    container.appendChild(this.renderer.domElement);

    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = "replay-labels";
    container.appendChild(this.labels.domElement);

    this.scene.fog = new THREE.Fog(NIGHT, 150, 600);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 5000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.addEventListener("change", () => this.requestRender());

    this.addGround();
    this.addHoldingArea();

    const n = header.fleet_size;
    const coreMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.cores = new THREE.InstancedMesh(new THREE.SphereGeometry(0.28, 14, 10), coreMat, n);
    const haloMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.16,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
    });
    this.halos = new THREE.InstancedMesh(new THREE.SphereGeometry(0.5, 12, 8), haloMat, n);
    for (const mesh of [this.cores, this.halos]) {
      mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(n * 3), 3);
      mesh.frustumCulled = false;
      this.scene.add(mesh);
    }
    this.dots = new THREE.Points(
      new THREE.BufferGeometry()
        .setAttribute("position", new THREE.BufferAttribute(new Float32Array(n * 3), 3))
        .setAttribute("color", new THREE.BufferAttribute(new Float32Array(n * 3), 3)),
      new THREE.PointsMaterial({
        size: 8,
        sizeAttenuation: false,
        vertexColors: true,
        map: roundDot(),
        transparent: true,
        alphaTest: 0.3,
      }),
    );
    this.dots.frustumCulled = false;
    this.scene.add(this.dots);
    this.addViolationMarkers();

    this.pairLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: AMBER }),
    );
    this.pairLine.visible = false;
    this.scene.add(this.pairLine);
    const labelEl = document.createElement("div");
    labelEl.className = "replay-pair-label";
    this.pairLabel = new CSS2DObject(labelEl);
    this.pairLabel.visible = false;
    this.scene.add(this.pairLabel);

    this.frameAll();
    this.renderer.domElement.addEventListener("click", this.handleClick);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.update();
  }

  private addGround() {
    const { bounds_min: lo, bounds_max: hi, ground_z_m } = this.data.header;
    const extent = Math.max(hi[0] - lo[0], hi[1] - lo[1], 20) + 40;
    const size = Math.ceil(extent / 10) * 10;
    const grid = new THREE.GridHelper(size, size / 2, GRID_MAJOR, GRID_MINOR);
    grid.position.set((lo[0] + hi[0]) / 2, ground_z_m, -(lo[1] + hi[1]) / 2);
    this.scene.add(grid);
  }

  private addHoldingArea() {
    const ha = this.data.header.overlays.holding_area;
    if (!ha || ha.slots.length === 0) return;
    const pts = ha.slots.map((s) => toThree(s));
    const pads = new THREE.Points(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.PointsMaterial({ color: PAD, size: 0.35, sizeAttenuation: true }),
    );
    this.scene.add(pads);

    // Outline of the parked grid on its lowest layer.
    const xs = ha.slots.map((s) => s[0]);
    const ys = ha.slots.map((s) => s[1]);
    const z = Math.min(...ha.slots.map((s) => s[2]));
    const pad = ha.grid_spacing_m / 2;
    const [x0, x1, y0, y1] = [Math.min(...xs) - pad, Math.max(...xs) + pad, Math.min(...ys) - pad, Math.max(...ys) + pad];
    const outline = [
      [x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z], [x0, y0, z],
    ].map((p) => toThree(p as V3));
    this.scene.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(outline),
        new THREE.LineBasicMaterial({ color: PAD }),
      ),
    );
  }

  private addViolationMarkers() {
    const violations = this.data.header.overlays.failure?.violations ?? [];
    if (!violations.length) return;
    this.markers = new THREE.InstancedMesh(
      new THREE.OctahedronGeometry(0.22),
      new THREE.MeshBasicMaterial({ color: RED, transparent: true, opacity: 0.85 }),
      violations.length,
    );
    const mid = new THREE.Vector3();
    violations.forEach((v, i) => {
      toThree(
        [
          (v.position_a[0] + v.position_b[0]) / 2,
          (v.position_a[1] + v.position_b[1]) / 2,
          (v.position_a[2] + v.position_b[2]) / 2,
        ],
        mid,
      );
      this.tmp.position.copy(mid);
      this.tmp.scale.setScalar(1);
      this.tmp.updateMatrix();
      this.markers!.setMatrixAt(i, this.tmp.matrix);
    });
    this.scene.add(this.markers);
  }

  frameAll() {
    const { bounds_min: lo, bounds_max: hi } = this.data.header;
    const center = toThree([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2]);
    const radius = Math.max(Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / 2, 5);
    // Distance that fits the bounding sphere in the narrower of the two fields of view of the
    // uncovered area.
    const w = Math.max(this.container.clientWidth - this.rightInset, 1);
    const h = Math.max(this.container.clientHeight, 1);
    const vFov = THREE.MathUtils.degToRad(this.camera.fov);
    const hFov = 2 * Math.atan(Math.tan(vFov / 2) * (w / h));
    const distance = (radius / Math.sin(Math.min(vFov, hFov) / 2)) * 1.05;
    this.controls.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(0.55, 0.45, 0.7).normalize().multiplyScalar(distance));
    this.controls.update();
    this.requestRender();
  }

  /** Point the camera at a pair of drones at the current time. */
  focusOn(drones: number[]) {
    if (!drones.length) return;
    const c = new THREE.Vector3();
    for (const d of drones) c.add(toThree(positionAt(this.data, d, this.time, this.p)));
    c.divideScalar(drones.length);
    const offset = this.camera.position.clone().sub(this.controls.target).setLength(14);
    this.controls.target.copy(c);
    this.camera.position.copy(c).add(offset);
    this.controls.update();
    this.requestRender();
  }

  /** Keep the scene's centre in the part of the canvas an overlay on the right doesn't cover. */
  setRightInset(px: number) {
    const first = this.rightInset === 0 && px > 0;
    this.rightInset = Math.max(0, px);
    this.resize();
    if (first) this.frameAll();
  }

  setTime(t: number) {
    this.time = t;
    this.update();
  }

  setHighlight(drones: number[]) {
    this.highlight = drones;
    this.update();
  }

  private update() {
    const n = this.data.header.fleet_size;
    // Colors switch at frame boundaries (no blending), matching the 20 fps sampling.
    const { k } = locate(this.data.separation.times, this.time);
    const colors = this.data.colors;
    const pos = new THREE.Vector3();
    const dotPos = this.dots.geometry.getAttribute("position") as THREE.BufferAttribute;
    const dotCol = this.dots.geometry.getAttribute("color") as THREE.BufferAttribute;
    for (let i = 0; i < n; i++) {
      toThree(positionAt(this.data, i, this.time, this.p), pos);
      const lit = this.highlight.includes(i);
      this.tmp.position.copy(pos);
      this.tmp.scale.setScalar(lit ? 1.35 : 1); // colour marks the selection; size stays honest
      this.tmp.updateMatrix();
      this.cores.setMatrixAt(i, this.tmp.matrix);
      this.halos.setMatrixAt(i, this.tmp.matrix);
      const ci = (k * n + i) * 3;
      const r = colors[ci], g = colors[ci + 1], b = colors[ci + 2];
      if (r + g + b < 24) this.color.copy(LED_OFF);
      else this.color.setRGB(r / 255, g / 255, b / 255, THREE.SRGBColorSpace);
      if (lit) this.color.setHex(AMBER);
      this.cores.setColorAt(i, this.color);
      this.halos.setColorAt(i, this.color);
      dotPos.setXYZ(i, pos.x, pos.y, pos.z);
      dotCol.setXYZ(i, this.color.r, this.color.g, this.color.b);
    }
    dotPos.needsUpdate = true;
    dotCol.needsUpdate = true;
    this.cores.instanceMatrix.needsUpdate = true;
    this.halos.instanceMatrix.needsUpdate = true;
    this.cores.instanceColor!.needsUpdate = true;
    this.halos.instanceColor!.needsUpdate = true;
    this.updatePair();
    this.requestRender();
  }

  private updatePair() {
    const show = this.highlight.length === 2;
    this.pairLine.visible = show;
    this.pairLabel.visible = show;
    if (!show) return;
    const a = toThree(positionAt(this.data, this.highlight[0], this.time, this.p));
    const pa: V3 = [...this.p];
    const b = toThree(positionAt(this.data, this.highlight[1], this.time, this.p));
    this.pairLine.geometry.setFromPoints([a, b]);
    this.pairLabel.position.copy(a).add(b).multiplyScalar(0.5);
    const d = Math.hypot(pa[0] - this.p[0], pa[1] - this.p[1], pa[2] - this.p[2]);
    const floor = this.data.header.overlays.gatekeeper_floor_m ?? 0;
    const el = this.pairLabel.element;
    el.textContent = `${d.toFixed(2)} m`;
    el.dataset.state = d < floor ? "violation" : "ok";
  }

  private handleClick = (e: MouseEvent) => {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = this.raycaster.intersectObject(this.halos, false)[0];
    this.onPick(hit?.instanceId ?? null);
  };

  private resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = `${w}px`;
    this.renderer.domElement.style.height = `${h}px`;
    this.labels.setSize(w, h);
    // Render the left w px of a (w + inset)-wide virtual view: the projection centre lands in the
    // middle of the uncovered area.
    const inset = this.rightInset < w / 2 ? this.rightInset : 0;
    this.camera.aspect = (w + inset) / h;
    if (inset) this.camera.setViewOffset(w + inset, h, inset, 0, w, h);
    else this.camera.clearViewOffset();
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  private requestRender() {
    if (this.frameRequested) return;
    this.frameRequested = true;
    requestAnimationFrame(() => {
      this.frameRequested = false;
      this.renderer.render(this.scene, this.camera);
      this.labels.render(this.scene, this.camera);
    });
  }

  dispose() {
    this.resizeObserver.disconnect();
    this.renderer.domElement.removeEventListener("click", this.handleClick);
    this.controls.dispose();
    this.scene.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      mesh.geometry?.dispose();
      const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
      (Array.isArray(mat) ? mat : mat ? [mat] : []).forEach((m) => {
        (m as THREE.PointsMaterial).map?.dispose();
        m.dispose();
      });
    });
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.labels.domElement.remove();
  }
}

/** Soft round sprite so point dots read as lights, not squares. */
function roundDot(): THREE.Texture {
  const size = 32;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  const g = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  g.addColorStop(0, "rgba(255,255,255,1)");
  g.addColorStop(0.55, "rgba(255,255,255,1)");
  g.addColorStop(1, "rgba(255,255,255,0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}
