// RRT* (Rapidly-exploring Random Tree Star) path planner.
//
// All coordinates in mm.  Works with the PhysicsEngine obstacle set.
// Returns a collision-free path from start to goal, or null if no path
// found within the iteration budget.

import type { Obstacle } from '../types/twin';

// ── Config ────────────────────────────────────────────────────────────────

export interface RRTConfig {
  maxIterations: number;     // tree expansion attempts
  stepSize: number;          // mm — max extension per step
  goalBias: number;          // 0-1 probability of sampling goal directly
  goalTolerance: number;     // mm — considered "reached" within this radius
  robotRadius: number;       // mm — inflated collision check
  nearbyRadius: number;      // mm — rewiring neighbourhood (RRT* radius)
}

const DEFAULT_CONFIG: RRTConfig = {
  maxIterations: 2000,
  stepSize: 150,
  goalBias: 0.08,
  goalTolerance: 300,
  robotRadius: 230,
  nearbyRadius: 300,
};

// ── Public types ──────────────────────────────────────────────────────────

export interface Point { x: number; y: number; }

interface RRTNode {
  x: number;
  y: number;
  parent: number | null;
  cost: number;       // path cost from root
}

export interface RRTResult {
  path: Point[];          // collision-free waypoints (mm)
  tree: RRTNode[];        // full tree for visualisation
  reached: boolean;       // did we reach the goal?
  iterations: number;
}

// ── Main planner ──────────────────────────────────────────────────────────

export function planRRT(
  start: Point,
  goal: Point,
  obstacles: Obstacle[],
  cfg?: Partial<RRTConfig>,
): RRTResult {
  const c = { ...DEFAULT_CONFIG, ...cfg };

  const tree: RRTNode[] = [{ x: start.x, y: start.y, parent: null, cost: 0 }];

  // Spatial index — simple grid for fast nearest-neighbour (sufficient <5000 nodes)
  const cellSize = c.nearbyRadius * 2;
  const grid = new Map<string, number[]>();

  function cellKey(x: number, y: number): string {
    return `${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`;
  }

  function insertNode(idx: number, n: RRTNode) {
    const key = cellKey(n.x, n.y);
    const arr = grid.get(key);
    if (arr) arr.push(idx); else grid.set(key, [idx]);
  }

  function nearbyKeys(x: number, y: number): number[] {
    const cx = Math.floor(x / cellSize);
    const cy = Math.floor(y / cellSize);
    const ids: number[] = [];
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const arr = grid.get(`${cx + dx},${cy + dy}`);
        if (arr) ids.push(...arr);
      }
    }
    return ids;
  }

  insertNode(0, tree[0]);

  // Precompute inflated obstacle bounds (robot centre forbidden zone)
  const inflated = obstacles.map((o) => ({
    minX: o.x - o.width  / 2 - c.robotRadius,
    maxX: o.x + o.width  / 2 + c.robotRadius,
    minY: o.y - o.depth  / 2 - c.robotRadius,
    maxY: o.y + o.depth  / 2 + c.robotRadius,
  }));

  function collides(x: number, y: number): boolean {
    for (const o of inflated) {
      if (x > o.minX && x < o.maxX && y > o.minY && y < o.maxY) return true;
    }
    return false;
  }

  function segmentCollides(ax: number, ay: number, bx: number, by: number): boolean {
    const dx = bx - ax;
    const dy = by - ay;
    const len = Math.sqrt(dx * dx + dy * dy);
    const steps = Math.max(1, Math.ceil(len / (c.stepSize / 2)));
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      if (collides(ax + dx * t, ay + dy * t)) return true;
    }
    return false;
  }

  function dist(a: Point, b: Point): number {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  let goalIdx = -1;

  for (let iter = 0; iter < c.maxIterations; iter++) {
    // ── 1. Sample ──
    let sample: Point;
    if (Math.random() < c.goalBias) {
      sample = goal;
    } else {
      sample = {
        x: (Math.random() - 0.5) * 10000,
        y: (Math.random() - 0.5) * 10000,
      };
    }

    // ── 2. Nearest ──
    let nearestIdx = 0;
    let nearestDist = Infinity;
    const candidates = nearbyKeys(sample.x, sample.y);
    for (const idx of candidates.length > 0 ? candidates : Array.from({ length: tree.length }, (_, i) => i)) {
      const d = dist(tree[idx], sample);
      if (d < nearestDist) {
        nearestDist = d;
        nearestIdx = idx;
      }
    }

    // ── 3. Steer ──
    const nearest = tree[nearestIdx];
    let steerDist = nearestDist;
    let newX = sample.x;
    let newY = sample.y;
    if (steerDist > c.stepSize) {
      const ratio = c.stepSize / steerDist;
      newX = nearest.x + (sample.x - nearest.x) * ratio;
      newY = nearest.y + (sample.y - nearest.y) * ratio;
      steerDist = c.stepSize;
    }

    // ── 4. Collision check ──
    if (collides(newX, newY)) continue;
    if (segmentCollides(nearest.x, nearest.y, newX, newY)) continue;

    // ── 5. Choose best parent (RRT* rewiring) ──
    const nearby = nearbyKeys(newX, newY);
    let bestParent = nearestIdx;
    let bestCost = nearest.cost + steerDist;

    for (const idx of nearby) {
      const n = tree[idx];
      const d = dist(n, { x: newX, y: newY });
      const candidateCost = n.cost + d;
      if (candidateCost < bestCost && !segmentCollides(n.x, n.y, newX, newY)) {
        bestParent = idx;
        bestCost = candidateCost;
      }
    }

    const newIdx = tree.length;
    tree.push({ x: newX, y: newY, parent: bestParent, cost: bestCost });
    insertNode(newIdx, tree[newIdx]);

    // ── 6. Rewire nearby nodes through new node ──
    for (const idx of nearby) {
      if (idx === bestParent) continue;
      const n = tree[idx];
      const d = dist({ x: newX, y: newY }, n);
      const altCost = bestCost + d;
      if (altCost < n.cost && !segmentCollides(newX, newY, n.x, n.y)) {
        n.parent = newIdx;
        n.cost = altCost;
      }
    }

    // ── 7. Goal check ──
    if (dist({ x: newX, y: newY }, goal) <= c.goalTolerance) {
      goalIdx = newIdx;
      break;
    }
  }

  // ── Extract path ──
  if (goalIdx < 0) {
    // Try to find closest node to goal as a partial result
    let closestIdx = 0;
    let closestDist = Infinity;
    for (let i = 0; i < tree.length; i++) {
      const d = dist(tree[i], goal);
      if (d < closestDist) {
        closestDist = d;
        closestIdx = i;
      }
    }
    // Only return partial path if reasonably close
    if (closestDist < c.goalTolerance * 3) {
      goalIdx = closestIdx;
    } else {
      return { path: [], tree, reached: false, iterations: c.maxIterations };
    }
  }

  const path: Point[] = [];
  let idx: number | null = goalIdx;
  while (idx !== null) {
    const node: RRTNode = tree[idx];
    path.unshift({ x: node.x, y: node.y });
    idx = node.parent;
  }

  // Simplify: remove collinear waypoints that don't improve path
  const simplified = simplifyPath(path, c);

  return {
    path: simplified,
    tree,
    reached: dist(tree[goalIdx], goal) <= c.goalTolerance,
    iterations: c.maxIterations,
  };
}

// ── Path simplification ───────────────────────────────────────────────────

function simplifyPath(path: Point[], cfg: RRTConfig): Point[] {
  if (path.length <= 2) return path;

  const inflated = [] as Array<{ minX: number; maxX: number; minY: number; maxY: number }>;
  // We don't have obstacles here so just do simple collinear removal
  const result: Point[] = [path[0]];
  let prev = path[0];

  for (let i = 1; i < path.length - 1; i++) {
    const prevDx = prev.x - path[i].x;
    const prevDy = prev.y - path[i].y;
    const nextDx = path[i].x - path[i + 1].x;
    const nextDy = path[i].y - path[i + 1].y;

    // Keep corner points (direction change > 5°)
    const cross = Math.abs(prevDx * nextDy - prevDy * nextDx);
    const mag = Math.sqrt(prevDx * prevDx + prevDy * prevDy) * Math.sqrt(nextDx * nextDx + nextDy * nextDy);
    if (mag > 0 && cross / mag > 0.087) { // sin(5°)
      result.push(path[i]);
      prev = path[i];
    }
  }

  result.push(path[path.length - 1]);
  return result;
}

// ── Utility ───────────────────────────────────────────────────────────────

export function getPathLength(path: Point[]): number {
  let len = 0;
  for (let i = 1; i < path.length; i++) {
    const dx = path[i].x - path[i - 1].x;
    const dy = path[i].y - path[i - 1].y;
    len += Math.sqrt(dx * dx + dy * dy);
  }
  return len;
}
