import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const deg = value => value * Math.PI / 180;
export function toolPosition(state) {
  const radius = 220 + state.h;
  const angle = deg(state.turret - 260);
  return new THREE.Vector3(Math.sin(angle) * radius, 200 - state.lift, -Math.cos(angle) * radius);
}

export class MechanismScene {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#f8fafb');
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setClearColor('#f8fafb');
    container.prepend(this.renderer.domElement);
    this.camera = new THREE.OrthographicCamera(-400, 400, 390, -390, 1, 3000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minZoom = 0.55;
    this.controls.maxZoom = 3;
    this.controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
    this.controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.03;
    this.scene.add(new THREE.HemisphereLight('#ffffff', '#879da8', 2.2));
    const sun = new THREE.DirectionalLight('#fff8eb', 3.2);
    sun.position.set(-250, 650, 200);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -500, right: 500, top: 500, bottom: -500, near: 1, far: 1600 });
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.6;
    sun.shadow.radius = 3;
    this.scene.add(sun);
    this.materials = {
      body: this.material('#344651'), edge: this.material('#22343e'), metal: this.material('#b8c9cc'),
      rail: this.material('#dce6e8'), mint: this.material('#258f7d'), dark: this.material('#253a42'),
      rubber: this.material('#39434a'), tray: this.material('#d8e5e1'), ground: this.material('#f7f9fa'),
      accents: ['#58ad97', '#e5b05f', '#d77f75'].map(color => this.material(color)),
    };
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(1500, 1500), new THREE.ShadowMaterial({ opacity: 0.12 }));
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);
    const grid = new THREE.GridHelper(1200, 24, '#e1e8eb', '#e8edef');
    grid.position.y = 0.2;
    grid.material.transparent = true;
    grid.material.opacity = 0.65;
    this.scene.add(grid);
    this.buildRobot();
    this.ringsRoot = new THREE.Group();
    this.scene.add(this.ringsRoot);
    this.sourceRoot = new THREE.Group();
    this.scene.add(this.sourceRoot);
    this.projected = new THREE.Mesh(new THREE.RingGeometry(8, 10, 32),
      new THREE.MeshBasicMaterial({ color: '#159b85', transparent: true, opacity: 0.9, depthTest: false }));
    this.projected.rotation.x = -Math.PI / 2;
    this.projected.renderOrder = 3;
    this.scene.add(this.projected);
    this.view('top');
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
  }

  material(color) { return new THREE.MeshStandardMaterial({ color, roughness: 0.68, metalness: 0.13 }); }
  box(parent, width, height, depth, material, x = 0, y = 0, z = 0) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
    mesh.position.set(x, y, z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    parent.add(mesh);
    return mesh;
  }
  cylinder(parent, radius, height, material, x = 0, y = 0, z = 0) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, 48), material);
    mesh.position.set(x, y, z);
    mesh.castShadow = mesh.receiveShadow = true;
    parent.add(mesh);
    return mesh;
  }
  label(text, color = '#6f858f', width = 80) {
    const canvas = document.createElement('canvas');
    canvas.width = 320;canvas.height = 100;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = color;
    ctx.font = '500 36px "Microsoft YaHei", sans-serif';
    ctx.textAlign = 'center';ctx.textBaseline = 'middle';
    ctx.fillText(text, 160, 50);
    const material = new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(width, width / 3.2, 1);
    sprite.renderOrder = 5;
    return sprite;
  }

  buildRobot() {
    const m = this.materials;
    this.robot = new THREE.Group();
    this.scene.add(this.robot);
    this.box(this.robot, 248, 21, 265, m.body, 0, 34, 36);
    this.box(this.robot, 234, 4, 251, m.edge, 0, 47, 36);
    this.box(this.robot, 208, 2, 220, m.body, 0, 50, 28);
    for (const x of [-139, 139]) for (const z of [-70, 145]) {
      this.box(this.robot, 32, 45, 66, m.rubber, x, 26, z);
      this.box(this.robot, 34, 3, 7, m.metal, x, 49, z - 20).rotation.y = deg(x > 0 ? 25 : -25);
      this.box(this.robot, 34, 3, 7, m.metal, x, 49, z).rotation.y = deg(x > 0 ? 25 : -25);
      this.box(this.robot, 34, 3, 7, m.metal, x, 49, z + 20).rotation.y = deg(x > 0 ? 25 : -25);
    }
    for (const x of [-103, 103]) for (const z of [-75, 144]) this.cylinder(this.robot, 4, 4, m.metal, x, 50, z);
    this.box(this.robot, 28, 4, 37, m.mint, 0, 52, -68);
    const arrow = new THREE.Mesh(new THREE.ConeGeometry(7, 16, 3), m.rail);
    arrow.rotation.x = -Math.PI / 2;arrow.position.set(0, 58, -68);this.robot.add(arrow);
    this.cylinder(this.robot, 37, 19, m.metal, 0, 65, 0);
    this.cylinder(this.robot, 29, 22, m.mint, 0, 83, 0);
    this.box(this.robot, 35, 114, 35, m.metal, 0, 146, 0);
    this.cylinder(this.robot, 25, 12, m.edge, 0, 205, 0);
    this.arm = new THREE.Group();
    this.robot.add(this.arm);
    this.box(this.arm, 24, 13, 330, m.metal, 0, 209, -143);
    this.box(this.arm, 7, 3, 323, m.mint, -8, 217, -143);
    this.box(this.arm, 7, 3, 323, m.rail, 8, 217, -143);
    this.cylinder(this.arm, 23, 14, m.dark, 0, 212, 0);
    this.carriage = new THREE.Group();this.arm.add(this.carriage);
    this.box(this.carriage, 43, 19, 44, m.mint, 0, 210, 0);
    this.liftRod = this.box(this.carriage, 10, 100, 10, m.metal, 0, 150, 0);
    this.claw = new THREE.Group();this.carriage.add(this.claw);
    this.box(this.claw, 40, 13, 30, m.dark);
    this.fingers = [-1, 1].map(sign => this.box(this.claw, 6, 24, 29, m.metal, sign * 12, -15, 0));
    this.held = this.box(this.claw, 44, 68, 44, m.accents[0], 0, -39, 0);
    this.tray = new THREE.Group();this.tray.position.set(-109, 0, 170);this.robot.add(this.tray);
    this.cylinder(this.tray, 60, 10, m.metal, 0, 104, 0);
    this.cylinder(this.tray, 58, 6, m.tray, 0, 112, 0);
    this.cylinder(this.tray, 13, 8, m.mint, 0, 119, 0);
    this.trayCubes = [];
    for (let i = 0; i < 3; i++) {
      const angle = i * Math.PI * 2 / 3;
      const x = Math.sin(angle) * 40, z = -Math.cos(angle) * 40;
      this.cylinder(this.tray, 22, 2, m.rail, x, 116, z);
      const cube = this.box(this.tray, 42, 68, 42, m.accents[i], x, 151, z);
      this.trayCubes.push(cube);
    }
    const caption = this.label('车头 ↑', '#7a8d96', 90);
    caption.position.set(0, 2, -130);this.scene.add(caption);
  }

  setData(data) {
    this.data = data;
    this.ringsRoot.clear();
    this.ringCubes = [];
    this.ringPoints = data.rings.map(pose => toolPosition({ ...pose, lift: 0 }));
    this.ringPoints.forEach((point, index) => {
      const disc = this.cylinder(this.ringsRoot, 33, 2, this.material('#f3f0e8'), point.x, 1.5, point.z);
      const ring = new THREE.Mesh(new THREE.TorusGeometry(31, 1.3, 8, 64), this.material('#c8bb9f'));
      ring.rotation.x = Math.PI / 2;ring.position.set(point.x, 3, point.z);this.ringsRoot.add(ring);
      const label = this.label(`${index + 1} 号圆环`, '#9e947f', 80);
      label.position.set(point.x, 5, point.z - 50);this.ringsRoot.add(label);
      const cube = this.box(this.ringsRoot, 44, 68, 44, this.materials.accents[index], point.x, 36, point.z);
      this.ringCubes.push(cube);
      disc.receiveShadow = true;
    });
  }

  setTimeline(timeline) {
    this.timeline = timeline;
    if (this.path) { this.scene.remove(this.path);this.path.geometry.dispose();this.path.material.dispose(); }
    const points = [];
    for (let i = 0; i <= 650; i++) {
      const p = toolPosition(timeline.stateAt(timeline.duration * i / 650));
      p.y = 5;points.push(p);
    }
    this.path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineDashedMaterial({ color: '#82a9bf', transparent: true, opacity: 0.7, dashSize: 5, gapSize: 5 }));
    this.path.computeLineDistances();this.scene.add(this.path);
    this.sourceRoot.clear();
    if (timeline.group.family === 'raw') {
      const target = timeline.group.actions.find(a => a.type === 'pose' && !a.approach && a.pose.lift > 0).pose;
      const position = toolPosition(target);
      this.sourceCube = this.box(this.sourceRoot, 44, 68, 44,
        this.materials.accents[timeline.group.slot - 1], position.x, position.y - 39, position.z);
      const base = this.box(this.sourceRoot, 58, 4, 58, this.materials.tray, position.x, position.y - 75, position.z);
      const caption = this.label('原料取料位', '#879b9e', 100);
      caption.position.set(position.x, base.position.y + 1, position.z - 45);this.sourceRoot.add(caption);
    }
  }

  inventory(time) {
    const timeline = this.timeline;
    const family = timeline.group.family;
    const raw = family === 'raw', recover = family === 'return' || family === 'return2';
    const tray = raw || recover ? [null, null, null] : [0, 1, 2];
    const rings = [null, null, null];
    if (recover) timeline.group.order.forEach((ring, index) => { rings[ring - 1] = index; });
    let slot = timeline.group.initial.slot - 1, held = null, picked = false;
    for (const event of timeline.events) {
      if (event.time > time) break;
      if (event.type === 'slot') { slot = event.slot - 1;continue; }
      const point = toolPosition(event.pose);
      let nearest = 0;
      this.ringPoints.forEach((p, i) => { if (p.distanceToSquared(point) < this.ringPoints[nearest].distanceToSquared(point)) nearest = i; });
      if (event.type === 'close') {
        if (raw) { held = timeline.group.slot - 1;picked = true; }
        else if (recover) { held = rings[nearest];rings[nearest] = null; }
        else { held = tray[slot];tray[slot] = null; }
      } else if (event.type === 'open' && held !== null) {
        if (Math.abs(event.pose.turret - 120) < 0.1) tray[slot] = held;
        else rings[nearest] = held;
        held = null;
      }
    }
    return { tray, rings, held, picked };
  }

  update(state, time, pathVisible = true) {
    const inventory = this.inventory(time);
    this.arm.rotation.y = -deg(state.turret - 260);
    this.carriage.position.z = -(220 + state.h);
    this.claw.position.y = 200 - state.lift;
    const rodHeight = Math.max(1, 210 - this.claw.position.y);
    this.liftRod.scale.y = rodHeight / 100;
    this.liftRod.position.y = (210 + this.claw.position.y) / 2;
    const open = THREE.MathUtils.clamp((state.gripper - this.timeline.group.servo.close) /
      Math.max(1, this.timeline.group.servo.open - this.timeline.group.servo.close), 0, 1);
    this.fingers.forEach((finger, index) => { finger.position.x = (index === 0 ? -1 : 1) * (13 + open * 13); });
    this.tray.rotation.y = deg(state.platform - this.timeline.group.servo.platformAngles[0]);
    this.trayCubes.forEach((cube, index) => { cube.visible = inventory.tray[index] !== null;if (cube.visible) cube.material = this.materials.accents[inventory.tray[index]]; });
    this.ringCubes.forEach((cube, index) => {
      cube.visible = inventory.rings[index] !== null;
      cube.position.y = this.timeline.group.layer === 2 ? 105 : 36;
      if (cube.visible) cube.material = this.materials.accents[inventory.rings[index]];
    });
    this.held.visible = inventory.held !== null;
    if (this.held.visible) this.held.material = this.materials.accents[inventory.held];
    if (this.sourceCube && this.sourceCube.parent === this.sourceRoot) this.sourceCube.visible = !inventory.picked;
    this.path.visible = pathVisible;
    const position = toolPosition(state);
    this.projected.position.set(position.x, 6, position.z);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    const projected = position.clone().project(this.camera);
    const label = document.getElementById('tool-label');
    label.style.left = `${(projected.x + 1) / 2 * this.container.clientWidth + 14}px`;
    label.style.top = `${(1 - projected.y) / 2 * this.container.clientHeight - 13}px`;
    label.style.opacity = Math.abs(projected.x) < 0.92 && Math.abs(projected.y) < 0.85 ? 1 : 0;
  }

  view(type) {
    this.controls.target.set(0, 70, -25);
    if (type === 'top') {
      this.camera.position.set(0, 1100, -25);
      this.camera.up.set(0, 0, -1);
      this.controls.enableRotate = false;
    } else {
      this.camera.position.set(530, 650, 570);
      this.camera.up.set(0, 1, 0);
      this.controls.enableRotate = true;
    }
    this.camera.zoom = 1;
    this.camera.lookAt(this.controls.target);this.camera.updateProjectionMatrix();this.controls.update();
  }

  resize() {
    const width = this.container.clientWidth, height = this.container.clientHeight;
    const halfHeight = 390, aspect = width / Math.max(1, height);
    this.camera.left = -halfHeight * aspect;this.camera.right = halfHeight * aspect;
    this.camera.top = halfHeight;this.camera.bottom = -halfHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
}
