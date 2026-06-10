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

  // Camera — X-right, Y-forward, Z-up (ROS2 convention mapped to Three.js)
  const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 100);
  camera.position.set(4, -4, 5);
  camera.lookAt(0, 0, 0);

  // Controls
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 2;
  controls.maxDistance = 20;
  controls.maxPolarAngle = Math.PI / 2.1;
  controls.target.set(0, 0, 0);
  controls.update();

  // Lighting
  const ambient = new THREE.AmbientLight(0x404060, 0.6);
  scene.add(ambient);

  const dir = new THREE.DirectionalLight(0xffffff, 1.2);
  dir.position.set(5, -3, 8);
  dir.castShadow = true;
  dir.shadow.mapSize.set(1024, 1024);
  dir.shadow.camera.near = 0.5;
  dir.shadow.camera.far = 30;
  dir.shadow.camera.left = -8;
  dir.shadow.camera.right = 8;
  dir.shadow.camera.top = 8;
  dir.shadow.camera.bottom = -8;
  scene.add(dir);

  const hemi = new THREE.HemisphereLight(0x606080, 0x404040, 0.3);
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
