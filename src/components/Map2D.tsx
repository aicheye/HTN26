import { useEffect, useRef } from "react";
import type { MapProps } from "./MapProps";
import {
  ARM_COLOR,
  CHASSIS,
  CHASSIS_EDGE,
  DANGER_M,
  FLOOR_EDGE,
  GAIT_RATE,
  GAIT_STRIDE,
  GRID,
  OBSTACLE,
  OBSTACLE_DANGER,
  SHADOW,
  TILE_M,
  distanceTo,
  isWalking,
} from "./mapShared";
import type { Obstacle, Point, Robot, WorldState } from "../types/world";

const PADDING = 20; // px around the arena

type View = { scale: number; offsetX: number; offsetY: number; height: number };

export function Map2D({
  state,
  showCameraLayer = false,
  compact = false,
  onPickGoal,
}: MapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const viewRef = useRef<View>({ scale: 1, offsetX: 0, offsetY: 0, height: 0 });

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;

    const draw = () => {
      const ctx = canvas.getContext("2d");
      if (!ctx) return;
      const dpr = window.devicePixelRatio || 1;
      const w = wrap.clientWidth;
      const h = wrap.clientHeight;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      viewRef.current = computeView(state, w, h, compact ? 8 : PADDING);
      render(ctx, w, h, state, viewRef.current, showCameraLayer, compact);
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [state, showCameraLayer, compact]);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onPickGoal) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const { scale, offsetX, offsetY, height } = viewRef.current;
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const x = (px - offsetX) / scale;
    const y = (height - py - offsetY) / scale;
    if (x < 0 || y < 0 || x > state.arena.width || y > state.arena.length) return;
    onPickGoal({ x, y });
  };

  return (
    <div ref={wrapRef} className="relative h-full w-full">
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        className="h-full w-full cursor-crosshair rounded-lg"
      />
    </div>
  );
}

function computeView(state: WorldState, w: number, h: number, pad = PADDING): View {
  const { width, length } = state.arena;
  const scale = Math.min((w - pad * 2) / width, (h - pad * 2) / length);
  return {
    scale,
    offsetX: (w - width * scale) / 2,
    offsetY: (h - length * scale) / 2,
    height: h,
  };
}

/** World meters -> canvas pixels (y flipped so +y points up the screen). */
function toPx(p: Point, v: View): [number, number] {
  return [v.offsetX + p.x * v.scale, v.height - (v.offsetY + p.y * v.scale)];
}

const FLOOR = "#e2e8f0";

function render(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  state: WorldState,
  v: View,
  showCameraLayer: boolean,
  compact: boolean,
) {
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0, 0, w, h);

  const tracked = state.robots.find((r) => r.tracking);
  drawFloor(ctx, state, v);
  if (showCameraLayer) drawCameraPlaceholder(ctx, state, v);
  drawCornerTags(ctx, state, v);
  state.obstacles.forEach((o) =>
    drawObstacle(ctx, o, v, tracked ? distanceTo(o, tracked) < DANGER_M : false),
  );
  drawPath(ctx, state.path, v);
  if (state.goal) drawGoal(ctx, state.goal, v);
  state.robots.forEach((r) => drawRobot(ctx, r, v));
  if (state.arm) drawArm(ctx, state, v);
  if (!compact) drawScaleBar(ctx, state, v);
}

function drawFloor(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const { width, length } = state.arena;
  const [x, y] = toPx({ x: 0, y: length }, v);
  const wpx = width * v.scale;
  const hpx = length * v.scale;

  ctx.save();
  ctx.fillStyle = FLOOR;
  ctx.fillRect(x, y, wpx, hpx);

  ctx.beginPath();
  ctx.rect(x, y, wpx, hpx);
  ctx.clip();
  ctx.strokeStyle = GRID;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let gx = TILE_M; gx < width; gx += TILE_M) {
    const [px] = toPx({ x: gx, y: 0 }, v);
    ctx.moveTo(px, y);
    ctx.lineTo(px, y + hpx);
  }
  for (let gy = TILE_M; gy < length; gy += TILE_M) {
    const [, py] = toPx({ x: 0, y: gy }, v);
    ctx.moveTo(x, py);
    ctx.lineTo(x + wpx, py);
  }
  ctx.stroke();
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = FLOOR_EDGE;
  ctx.lineWidth = 2;
  ctx.strokeRect(x, y, wpx, hpx);
  ctx.restore();
}

/** Fixed calibration markers, drawn as little AprilTag glyphs. */
function drawCornerTags(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  if (!state.arena.cornerTagIds?.length) return;
  const { width, length } = state.arena;
  const inset = 0.08;
  const size = 0.06 * v.scale;
  const corners: Point[] = [
    { x: inset, y: inset },
    { x: width - inset, y: inset },
    { x: width - inset, y: length - inset },
    { x: inset, y: length - inset },
  ];

  ctx.save();
  corners.forEach((c) => {
    const [px, py] = toPx(c, v);
    ctx.fillStyle = "#334155";
    ctx.fillRect(px - size / 2, py - size / 2, size, size);
    ctx.fillStyle = FLOOR;
    ctx.fillRect(px - size / 6, py - size / 6, size / 3, size / 3);
  });
  ctx.restore();
}

/** Scale reference, drawn inside the arena so it reads as part of the map. */
function drawScaleBar(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const len = v.scale; // one meter
  const [left, bottom] = toPx({ x: state.arena.width / 2, y: 0 }, v);
  const x = left - len / 2;
  const y = bottom - 18;

  ctx.save();
  ctx.strokeStyle = "rgba(15, 23, 42, 0.35)";
  ctx.lineWidth = 2;
  ctx.lineCap = "butt";
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + len, y);
  ctx.moveTo(x, y - 4);
  ctx.lineTo(x, y + 4);
  ctx.moveTo(x + len, y - 4);
  ctx.lineTo(x + len, y + 4);
  ctx.stroke();
  ctx.fillStyle = "rgba(15, 23, 42, 0.5)";
  ctx.font = "10px ui-monospace, monospace";
  ctx.textAlign = "center";
  ctx.fillText("1 m", x + len / 2, y - 8);
  ctx.restore();
}

function drawCameraPlaceholder(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const [x0, y0] = toPx({ x: 0, y: state.arena.length }, v);
  const wpx = state.arena.width * v.scale;
  const hpx = state.arena.length * v.scale;
  ctx.save();
  ctx.beginPath();
  ctx.rect(x0, y0, wpx, hpx);
  ctx.clip();
  ctx.fillStyle = "rgba(15, 23, 42, 0.06)";
  ctx.fillRect(x0, y0, wpx, hpx);
  ctx.fillStyle = "rgba(15, 23, 42, 0.45)";
  ctx.font = "12px ui-monospace, monospace";
  ctx.textAlign = "center";
  ctx.fillText("CAMERA FEED — NO STREAM", x0 + wpx / 2, y0 + hpx / 2);
  ctx.restore();
}

/**
 * Exact detected geometry in pixels. Polygons come straight from the contour the
 * vision side traced; rects and circles are the analytic shapes they reported.
 */
function outline(o: Obstacle, v: View): [number, number][] {
  if (o.shape === "polygon" && o.points?.length) {
    return o.points.map((p) => toPx(p, v));
  }

  const [cx, cy] = toPx({ x: o.x, y: o.y }, v);
  if (o.shape === "circle") {
    const r = (o.radius ?? 0) * v.scale;
    return Array.from({ length: 32 }, (_, i) => {
      const a = (i / 32) * Math.PI * 2;
      return [cx + Math.cos(a) * r, cy + Math.sin(a) * r] as [number, number];
    });
  }
  return rectCorners(o, v);
}

function rectCorners(o: Obstacle, v: View): [number, number][] {
  const hw = (o.width ?? 0) / 2;
  const hl = (o.length ?? 0) / 2;
  const c = Math.cos(o.yaw);
  const s = Math.sin(o.yaw);
  return (
    [
      [-hw, -hl],
      [hw, -hl],
      [hw, hl],
      [-hw, hl],
    ] as [number, number][]
  ).map(([lx, ly]) =>
    toPx({ x: o.x + lx * c - ly * s, y: o.y + lx * s + ly * c }, v),
  );
}

function polyPath(ctx: CanvasRenderingContext2D, pts: [number, number][]) {
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.closePath();
}

function drawObstacle(
  ctx: CanvasRenderingContext2D,
  o: Obstacle,
  v: View,
  danger: boolean,
) {
  const pts = outline(o, v);

  ctx.save();
  polyPath(ctx, pts);
  ctx.fillStyle = danger ? OBSTACLE_DANGER : OBSTACLE;
  ctx.fill();
  ctx.restore();
}

function drawPath(ctx: CanvasRenderingContext2D, path: Point[] | undefined, v: View) {
  if (!path || path.length < 2) return;
  ctx.save();
  ctx.setLineDash([8, 6]);
  ctx.strokeStyle = "rgba(15, 23, 42, 0.55)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  path.forEach((p, i) => {
    const [px, py] = toPx(p, v);
    if (i === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  });
  ctx.stroke();
  ctx.restore();
}

function drawGoal(ctx: CanvasRenderingContext2D, goal: Point, v: View) {
  const [x, y] = toPx(goal, v);
  const head = 9;
  const top = y - 26;

  ctx.save();
  ctx.fillStyle = "rgba(30, 41, 59, 0.18)";
  ctx.beginPath();
  ctx.ellipse(x, y, 6, 2.5, 0, 0, Math.PI * 2);
  ctx.fill();

  ctx.beginPath();
  ctx.arc(x, top, head, Math.PI * 0.82, Math.PI * 0.18, false);
  ctx.lineTo(x, y);
  ctx.closePath();
  ctx.fillStyle = "#ef4444";
  ctx.fill();
  ctx.strokeStyle = "#991b1b";
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(x, top, 3.4, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.restore();
}

/** Top-down view of the pick-and-place arm: a bracket at its table-edge mount. */
function drawArm(ctx: CanvasRenderingContext2D, state: WorldState, v: View) {
  const arm = state.arm;
  if (!arm) return;
  const [x, y] = toPx(arm.mount, v);
  const active = arm.mode !== "idle";
  const r = 10;

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-arm.mount.yaw);
  ctx.fillStyle = ARM_COLOR;
  ctx.strokeStyle = "#7c2d12";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(-r, -r * 0.8);
  ctx.lineTo(r * 0.6, -r * 0.8);
  ctx.lineTo(r * 1.3, 0);
  ctx.lineTo(r * 0.6, r * 0.8);
  ctx.lineTo(-r, r * 0.8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  if (active) {
    const target = state.robots.find((rob) => rob.id === arm.targetRobotId);
    if (target) {
      const [tx, ty] = toPx(target, v);
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = "rgba(249, 115, 22, 0.7)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.restore();
    }
    ctx.save();
    ctx.font = "600 9px ui-monospace, monospace";
    ctx.textAlign = "center";
    ctx.fillStyle = "#c2410c";
    ctx.fillText(arm.mode.toUpperCase(), x, y + (arm.mount.side === "north" ? 18 : -14));
    ctx.restore();
  }
}

/**
 * Top-down quadruped drawn from the reported footprint: body plus four legs at the
 * hips. Sesame is two MG90 servos per leg, so the legs sit outboard of the shell.
 */
function drawRobot(ctx: CanvasRenderingContext2D, r: Robot, v: View) {
  const [cx, cy] = toPx({ x: r.x, y: r.y }, v);
  const L = r.footprint.length * v.scale; // along heading
  const W = r.footprint.width * v.scale; // across
  const bodyL = L * 0.84;
  const bodyW = W * 0.58;
  const legL = L * 0.24;
  const hipX = bodyL * 0.3;
  const legInner = bodyW * 0.3;
  const legSpan = W / 2 - legInner;

  const body = r.tracking ? CHASSIS : "#94a3b8";
  const edge = r.tracking ? CHASSIS_EDGE : "#64748b";

  // diagonal-pair trot while a drive command is active
  const gait = isWalking(r.mode);
  const phase = gait ? (performance.now() / 1000) * GAIT_RATE : 0;
  const stride = gait ? legL * GAIT_STRIDE : 0;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-r.yaw);

  ctx.shadowColor = SHADOW;
  ctx.shadowBlur = 5;
  ctx.shadowOffsetY = 2;

  ctx.fillStyle = edge;
  (
    [
      [hipX, -W / 2, 0],
      [hipX, legInner, Math.PI],
      [-hipX, -W / 2, Math.PI],
      [-hipX, legInner, 0],
    ] as [number, number, number][]
  ).forEach(([x, y, offset]) => {
    const swing = Math.sin(phase + offset) * stride;
    roundRect(ctx, x + swing - legL / 2, y, legL, legSpan, legL * 0.35);
    ctx.fill();
  });

  roundRect(ctx, -bodyL / 2, -bodyW / 2, bodyL, bodyW, bodyW * 0.28);
  ctx.fillStyle = body;
  ctx.fill();
  ctx.shadowColor = "transparent";
  ctx.strokeStyle = edge;
  ctx.lineWidth = 1.25;
  if (!r.tracking) ctx.setLineDash([4, 3]);
  ctx.stroke();
  ctx.setLineDash([]);

  drawEyes(ctx, bodyL, bodyW, r.tracking);
  ctx.restore();

  ctx.save();
  ctx.font = "9px ui-monospace, monospace";
  ctx.textAlign = "center";
  ctx.fillStyle = r.tracking ? "rgba(15, 23, 42, 0.6)" : "#b45309";
  ctx.fillText(
    r.tracking ? r.id.toUpperCase() : `${r.id.toUpperCase()} · NO FIX`,
    cx,
    cy - Math.max(W, L) / 2 - 8,
  );
  ctx.restore();
}

/** The OLED face, facing +x so the robot always looks where it drives. */
function drawEyes(
  ctx: CanvasRenderingContext2D,
  bodyL: number,
  bodyW: number,
  tracking: boolean,
) {
  const r = Math.max(1.6, bodyW * 0.16);
  const ex = bodyL * 0.24;
  const ey = bodyW * 0.22;

  [-ey, ey].forEach((y) => {
    ctx.beginPath();
    ctx.arc(ex, y, r, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fill();

    ctx.beginPath();
    ctx.arc(ex + r * 0.3, y, r * 0.5, 0, Math.PI * 2);
    ctx.fillStyle = tracking ? "#0f172a" : "#94a3b8";
    ctx.fill();
  });
}

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  w: number,
  h: number,
  r: number,
) {
  const radius = Math.max(0, Math.min(r, w / 2, h / 2));
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + w, y, x + w, y + h, radius);
  ctx.arcTo(x + w, y + h, x, y + h, radius);
  ctx.arcTo(x, y + h, x, y, radius);
  ctx.arcTo(x, y, x + w, y, radius);
  ctx.closePath();
}

