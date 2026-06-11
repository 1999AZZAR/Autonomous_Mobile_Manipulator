import * as THREE from 'three';

const CELL_SIZE = 0.1;
const GRID_OFFSET_Z = 0.005;
const OCCUPIED_COLOR = 0xff4444;
const FREE_COLOR = 0x44ff44;
const OCCUPIED_OPACITY = 0.4;
const FREE_OPACITY = 0.05;

export interface OccupancyGridRenderer {
  updateGrid(grid: Float32Array, width: number, height: number, originX: number, originY: number): void;
  setVisible(visible: boolean): void;
  dispose(): void;
}

export function createOccupancyGridRenderer(scene: THREE.Scene): OccupancyGridRenderer {
  const cellGroup = new THREE.Group();
  cellGroup.name = 'occupancy-grid';
  scene.add(cellGroup);

  let cellMeshes: THREE.Mesh[] = [];

  function clearCells() {
    cellMeshes.forEach((m) => {
      cellGroup.remove(m);
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    });
    cellMeshes = [];
  }

  function updateGrid(
    grid: Float32Array,
    width: number,
    height: number,
    originX: number,
    originY: number
  ) {
    const geoOccupied = new THREE.PlaneGeometry(CELL_SIZE * 0.9, CELL_SIZE * 0.9);
    const matOccupied = new THREE.MeshStandardMaterial({
      color: OCCUPIED_COLOR,
      transparent: true,
      opacity: OCCUPIED_OPACITY,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const geoFree = new THREE.PlaneGeometry(CELL_SIZE * 0.9, CELL_SIZE * 0.9);
    const matFree = new THREE.MeshStandardMaterial({
      color: FREE_COLOR,
      transparent: true,
      opacity: FREE_OPACITY,
      side: THREE.DoubleSide,
      depthWrite: false,
    });

    for (let gy = 0; gy < height; gy++) {
      for (let gx = 0; gx < width; gx++) {
        const idx = gy * width + gx;
        if (idx >= grid.length) continue;
        if (grid[idx] < 0.01) continue;

        const wx = (gx + originX) * CELL_SIZE;
        const wy = (gy + originY) * CELL_SIZE;
        const occupied = grid[idx] > 0.5;

        const mesh = new THREE.Mesh(occupied ? geoOccupied : geoFree, occupied ? matOccupied : matFree);
        mesh.position.set(wx, wy, GRID_OFFSET_Z);
        mesh.rotation.x = -Math.PI / 2;
        cellGroup.add(mesh);
        cellMeshes.push(mesh);
      }
    }
  }

  function setVisible(visible: boolean) {
    cellGroup.visible = visible;
  }

  function dispose() {
    clearCells();
    scene.remove(cellGroup);
  }

  return { updateGrid, setVisible, dispose };
}
