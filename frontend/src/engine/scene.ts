// Three.js scene setup — renderer, camera, lighting, ground, controls

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export interface TwinScene {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  ground: THREE.Mesh;
  gridHelper: THREE.GridHelper;
  animate: (callback: (time: number) => void) => () => void;
  resize: () => void;
  dispose: () => void;
}

export function createScene(container: HTMLElement): TwinScene {
  const width = container.clientWidth;
  const height = container.clientHeight;

  // Renderer
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  container.appendChild(renderer.domElement);

  // Scene
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a2e);
  scene.fog = new THREE.Fog(0x1a1a2e, 20, 50);

  // Camera — low elevation so vertical mast poles look tall, not like dots from above
  const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 100);
  camera.position.set(1.5, 5.5, 2.2);
  camera.lookAt(0, 0, 0.2);

  // Controls
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 1;
  controls.maxDistance = 20;
  controls.maxPolarAngle = Math.PI / 1.9;
  controls.target.set(0, 0, 0.2);
  controls.update();

  // Lighting — bright enough to see all robot faces
  const ambient = new THREE.AmbientLight(0xffffff, 2.5);
  scene.add(ambient);

  const dir = new THREE.DirectionalLight(0xffffff, 3.0);
  dir.position.set(4, -2, 6);
  dir.castShadow = true;
  dir.shadow.mapSize.set(1024, 1024);
  dir.shadow.camera.near = 0.5;
  dir.shadow.camera.far = 30;
  dir.shadow.camera.left = -8;
  dir.shadow.camera.right = 8;
  dir.shadow.camera.top = 8;
  dir.shadow.camera.bottom = -8;
  scene.add(dir);

  // Fill light from the front so the robot face is always lit
  const fill = new THREE.DirectionalLight(0xaaccff, 2.0);
  fill.position.set(-3, 5, 3);
  scene.add(fill);

  const hemi = new THREE.HemisphereLight(0x8899cc, 0x445566, 1.0);
  scene.add(hemi);

  // Ground plane — 10x10m
  const groundGeo = new THREE.PlaneGeometry(10, 10);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x2a2a3e,
    roughness: 0.9,
    metalness: 0.0,
  });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.z = -0.001;
  ground.receiveShadow = true;
  scene.add(ground);

  // Grid — 1m spacing
  const gridHelper = new THREE.GridHelper(10, 10, 0x444466, 0x333355);
  gridHelper.position.z = 0.001;
  scene.add(gridHelper);

  // Animation loop
  let running = true;
  const callbacks: Array<(time: number) => void> = [];
  let rafId = 0;

  function loop(time: number) {
    if (!running) return;
    controls.update();
    callbacks.forEach((fn) => fn(time));
    renderer.render(scene, camera);
    rafId = requestAnimationFrame(loop);
  }
  rafId = requestAnimationFrame(loop);

  function animate(callback: (time: number) => void): () => void {
    callbacks.push(callback);
    return () => {
      const idx = callbacks.indexOf(callback);
      if (idx >= 0) callbacks.splice(idx, 1);
    };
  }

  function resize() {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function dispose() {
    running = false;
    cancelAnimationFrame(rafId);
    renderer.dispose();
    container.removeChild(renderer.domElement);
  }

  return { renderer, scene, camera, controls, ground, gridHelper, animate, resize, dispose };
}
