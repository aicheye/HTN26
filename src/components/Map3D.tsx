import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useThree, type ThreeEvent } from "@react-three/fiber";
import { Line, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { MapProps } from "./MapProps";
import {
  CHASSIS,
  CHASSIS_EDGE,
  FLOOR,
  FLOOR_EDGE,
  GAIT_RATE,
  GAIT_STRIDE,
  GOAL,
  GRID,
  LEG_LAYOUT,
  OBSTACLE,
  OBSTACLE_DANGER,
  TILE_M,
  isDanger,
  isWalking,
  robotDims,
} from "./mapShared";
import type { Obstacle, Robot, WorldState } from "../types/world";

type Arena = WorldState["arena"];

const DRAG_PX = 4; // pointer travel that still counts as a click, not an orbit
const DRAG_MS = 300;

type ViewMode = "free" | "top" | "iso" | "follow";

/**
 * Three.js is Y-up but the schema is Z-up, so everything lives inside one group
 * rotated -90° about X. Inside it, position={[x, y, z]} and rotation-z={yaw} are
 * schema values verbatim. World -> local is (X, Y, Z) -> (X, -Z, Y).
 */
export function Map3D({ state, showCameraLayer = false, onPickGoal }: MapProps) {
  const { width, length } = state.arena;
  const [view, setView] = useState<ViewMode>("iso");
  const robot = state.robots.find((r) => r.tracking) ?? state.robots[0];

  return (
    <div className="relative h-full w-full">
      <Canvas
        shadows
        dpr={[1, 2]}
        camera={{
          position: [width / 2, length * 0.85, length * 1.0],
          fov: 45,
          near: 0.05,
          far: 60,
        }}
      >
        <color attach="background" args={["#f8fafc"]} />
        <hemisphereLight intensity={0.65} groundColor="#cbd5e1" />
        <directionalLight
          position={[width, 2.5, length]}
          intensity={1.6}
          castShadow
          shadow-mapSize={[1024, 1024]}
          shadow-camera-left={-3}
          shadow-camera-right={3}
          shadow-camera-top={3}
          shadow-camera-bottom={-3}
        />

        <group rotation={[-Math.PI / 2, 0, 0]}>
          <Ground arena={state.arena} dimmed={showCameraLayer} onPickGoal={onPickGoal} />
          <CornerTags arena={state.arena} />
          {state.obstacles.map((o) => (
            <ObstacleMesh key={o.id} obstacle={o} danger={isDanger(o, state.robots)} />
          ))}
          {state.path && state.path.length > 1 && (
            <Line
              points={state.path.map((p) => [p.x, p.y, 0.01] as [number, number, number])}
              color="#0f172a"
              lineWidth={2}
              dashed
              dashSize={0.06}
              gapSize={0.04}
            />
          )}
          {state.goal && <GoalPin x={state.goal.x} y={state.goal.y} />}
          {state.robots.map((r) => (
            <RobotModel key={r.id} robot={r} />
          ))}
        </group>

        <OrbitControls
          makeDefault
          enableDamping
          dampingFactor={0.1}
          rotateSpeed={0.55}
          panSpeed={0.7}
          zoomSpeed={0.8}
          zoomToCursor
          screenSpacePanning={false}
          target={[width / 2, 0, -length / 2]}
          minDistance={0.3}
          maxDistance={Math.max(width, length) * 2.2}
          maxPolarAngle={Math.PI / 2 - 0.06}
        />

        <CameraRig
          arena={state.arena}
          robot={robot}
          view={view}
          onUserTakeOver={() => setView((v) => (v === "follow" ? v : "free"))}
        />
      </Canvas>

      <div className="absolute left-3 top-3 flex gap-1 rounded-lg border border-zinc-200 bg-white/90 p-1 shadow-sm backdrop-blur">
        {(
          [
            ["iso", "Reset"],
            ["top", "Top"],
            ["follow", "Follow"],
          ] as [ViewMode, string][]
        ).map(([mode, label]) => (
          <button
            key={mode}
            type="button"
            onClick={() => setView(mode)}
            className={`rounded px-2 py-1 text-xs font-medium transition ${
              view === mode
                ? "bg-zinc-900 text-white"
                : "text-zinc-600 hover:bg-zinc-100"
            }`}
          >
            {label}
          </button>
        ))}
      </div>
    </div>
  );
}

/**
 * Keeps the camera usable: glides to view presets, follows the robot, and stops
 * the target and camera from sliding under the floor or off the arena.
 */
function CameraRig({
  arena,
  robot,
  view,
  onUserTakeOver,
}: {
  arena: Arena;
  robot: Robot | undefined;
  view: ViewMode;
  onUserTakeOver: () => void;
}) {
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as
    | { target: THREE.Vector3; addEventListener: Function; removeEventListener: Function }
    | null;
  const span = Math.max(arena.width, arena.length);
  const desired = useRef<{ pos: THREE.Vector3; target: THREE.Vector3 } | null>(null);
  const followOffset = useRef<THREE.Vector3 | null>(null);

  useEffect(() => {
    if (!controls) return;
    const onStart = () => {
      desired.current = null;
      onUserTakeOver();
    };
    controls.addEventListener("start", onStart);
    return () => controls.removeEventListener("start", onStart);
  }, [controls, onUserTakeOver]);

  useEffect(() => {
    const center = new THREE.Vector3(arena.width / 2, 0, -arena.length / 2);
    if (view === "top") {
      desired.current = {
        pos: new THREE.Vector3(center.x, span * 1.15, center.z + 0.001),
        target: center,
      };
    } else if (view === "iso") {
      desired.current = {
        pos: new THREE.Vector3(center.x, span * 0.75, center.z + span * 1.0),
        target: center,
      };
    } else {
      desired.current = null;
    }
    followOffset.current = null;
  }, [view, arena.width, arena.length, span]);

  useFrame((_, dt) => {
    if (!controls) return;
    const k = 1 - Math.exp(-6 * dt);

    if (view === "follow" && robot) {
      const target = new THREE.Vector3(robot.x, 0, -robot.y);
      if (!followOffset.current) {
        followOffset.current = camera.position.clone().sub(controls.target);
        if (followOffset.current.length() > span) {
          followOffset.current.setLength(Math.min(span * 0.6, 1.2));
        }
      }
      controls.target.lerp(target, k);
      camera.position.lerp(target.clone().add(followOffset.current), k);
    } else if (desired.current) {
      camera.position.lerp(desired.current.pos, k);
      controls.target.lerp(desired.current.target, k);
      if (camera.position.distanceTo(desired.current.pos) < 0.01) desired.current = null;
    }

    // keep the orbit point on the floor and inside the arena
    controls.target.x = clamp(controls.target.x, -0.2, arena.width + 0.2);
    controls.target.z = clamp(controls.target.z, -arena.length - 0.2, 0.2);
    controls.target.y = clamp(controls.target.y, 0, 0.25);
    camera.position.y = Math.max(camera.position.y, 0.05);
  });

  return null;
}

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

function Ground({
  arena,
  dimmed,
  onPickGoal,
}: {
  arena: Arena;
  dimmed: boolean;
  onPickGoal: MapProps["onPickGoal"];
}) {
  const texture = useTileTexture(arena);
  const down = useRef<{ x: number; y: number; t: number; moved: number } | null>(null);

  // OrbitControls captures the pointer, so drag distance has to come from the window
  useEffect(() => {
    const onMove = (e: PointerEvent) => {
      const d = down.current;
      if (!d) return;
      d.moved = Math.max(d.moved, Math.hypot(e.clientX - d.x, e.clientY - d.y));
    };
    window.addEventListener("pointermove", onMove);
    return () => window.removeEventListener("pointermove", onMove);
  }, []);

  const handleUp = (e: ThreeEvent<PointerEvent>) => {
    const start = down.current;
    down.current = null;
    if (!start || !onPickGoal) return;
    if (start.moved > DRAG_PX || performance.now() - start.t > DRAG_MS) return;
    onPickGoal({ x: e.point.x, y: -e.point.z });
  };

  return (
    <>
      <mesh
        position={[arena.width / 2, arena.length / 2, 0]}
        receiveShadow
        onPointerDown={(e) => {
          down.current = { x: e.clientX, y: e.clientY, t: performance.now(), moved: 0 };
        }}
        onPointerUp={handleUp}
      >
        <planeGeometry args={[arena.width, arena.length]} />
        <meshStandardMaterial
          map={texture}
          color={dimmed ? "#94a3b8" : "#ffffff"}
          roughness={0.95}
        />
      </mesh>
      <Line
        points={[
          [0, 0, 0.002],
          [arena.width, 0, 0.002],
          [arena.width, arena.length, 0.002],
          [0, arena.length, 0.002],
          [0, 0, 0.002],
        ]}
        color={FLOOR_EDGE}
        lineWidth={2}
      />
    </>
  );
}

/** One tile drawn once and repeated, so the 3D floor matches the 2D grid. */
function useTileTexture(arena: Arena) {
  return useMemo(() => {
    const size = 64;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d")!;
    ctx.fillStyle = FLOOR;
    ctx.fillRect(0, 0, size, size);
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 3;
    ctx.strokeRect(0, 0, size, size);

    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(arena.width / TILE_M, arena.length / TILE_M);
    texture.anisotropy = 4;
    return texture;
  }, [arena.width, arena.length]);
}

function CornerTags({ arena }: { arena: Arena }) {
  if (!arena.cornerTagIds?.length) return null;
  const inset = 0.08;
  const size = 0.06;
  const corners: [number, number][] = [
    [inset, inset],
    [arena.width - inset, inset],
    [arena.width - inset, arena.length - inset],
    [inset, arena.length - inset],
  ];
  return (
    <>
      {corners.map(([x, y], i) => (
        <mesh key={i} position={[x, y, 0.003]}>
          <planeGeometry args={[size, size]} />
          <meshBasicMaterial color="#334155" />
        </mesh>
      ))}
    </>
  );
}

function ObstacleMesh({ obstacle: o, danger }: { obstacle: Obstacle; danger: boolean }) {
  const height = o.height ?? 0.2;
  const color = danger ? OBSTACLE_DANGER : OBSTACLE;

  const extruded = useMemo(() => {
    if (o.shape !== "polygon" || !o.points?.length) return null;
    const pts = o.points.map((p) => new THREE.Vector2(p.x - o.x, p.y - o.y));
    // ExtrudeGeometry needs counter-clockwise winding or the normals invert
    if (THREE.ShapeUtils.isClockWise(pts)) pts.reverse();
    return new THREE.ExtrudeGeometry(new THREE.Shape(pts), {
      depth: height,
      bevelEnabled: false,
    });
  }, [o.points, o.x, o.y, height, o.shape]);

  useEffect(() => () => extruded?.dispose(), [extruded]);

  if (extruded) {
    return (
      <mesh geometry={extruded} position={[o.x, o.y, 0]} castShadow receiveShadow>
        <meshStandardMaterial color={color} roughness={0.8} />
      </mesh>
    );
  }

  if (o.shape === "circle") {
    return (
      <mesh
        position={[o.x, o.y, height / 2]}
        rotation={[Math.PI / 2, 0, 0]}
        castShadow
        receiveShadow
      >
        <cylinderGeometry args={[o.radius ?? 0.1, o.radius ?? 0.1, height, 28]} />
        <meshStandardMaterial color={color} roughness={0.8} />
      </mesh>
    );
  }

  return (
    <mesh
      position={[o.x, o.y, height / 2]}
      rotation={[0, 0, o.yaw]}
      castShadow
      receiveShadow
    >
      <boxGeometry args={[o.width ?? 0.2, o.length ?? 0.2, height]} />
      <meshStandardMaterial color={color} roughness={0.8} />
    </mesh>
  );
}

function GoalPin({ x, y }: { x: number; y: number }) {
  return (
    <group position={[x, y, 0]}>
      <mesh position={[0, 0, 0.09]} rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.035, 0.18, 20]} />
        <meshStandardMaterial color={GOAL} roughness={0.5} />
      </mesh>
      <mesh position={[0, 0, 0.001]} rotation={[0, 0, 0]}>
        <ringGeometry args={[0.05, 0.065, 24]} />
        <meshBasicMaterial color={GOAL} transparent opacity={0.8} />
      </mesh>
    </group>
  );
}

function RobotModel({ robot }: { robot: Robot }) {
  const group = useRef<THREE.Group>(null);
  const legs = useRef<(THREE.Mesh | null)[]>([]);
  const dims = robotDims(robot.footprint);
  const bodyH = 0.055;
  const legH = 0.045;

  // detections arrive ~20 Hz; damp toward them so the model glides
  const target = useRef({ x: robot.x, y: robot.y, yaw: robot.yaw });
  target.current = { x: robot.x, y: robot.y, yaw: robot.yaw };
  const walking = isWalking(robot.mode);

  useFrame((clockState, dt) => {
    const g = group.current;
    if (!g) return;
    const k = 1 - Math.exp(-12 * dt);
    g.position.x += (target.current.x - g.position.x) * k;
    g.position.y += (target.current.y - g.position.y) * k;
    let d = target.current.yaw - g.rotation.z;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    g.rotation.z += d * k;

    const phase = walking ? clockState.clock.elapsedTime * GAIT_RATE : 0;
    LEG_LAYOUT.forEach(([front, , offset], i) => {
      const leg = legs.current[i];
      if (!leg) return;
      const swing = walking ? Math.sin(phase + offset) * dims.legL * GAIT_STRIDE : 0;
      leg.position.x = front * dims.hipX + swing;
      leg.position.z = walking
        ? legH / 2 + Math.max(0, Math.cos(phase + offset)) * legH * 0.3
        : legH / 2;
    });
  });

  return (
    <group ref={group} position={[robot.x, robot.y, 0]} rotation={[0, 0, robot.yaw]}>
      {LEG_LAYOUT.map(([front, left], i) => (
        <mesh
          key={i}
          ref={(m) => {
            legs.current[i] = m;
          }}
          position={[front * dims.hipX, left * (dims.legInner + dims.legSpan / 2), legH / 2]}
          castShadow
        >
          <boxGeometry args={[dims.legL, dims.legSpan, legH]} />
          <meshStandardMaterial color={CHASSIS_EDGE} roughness={0.7} />
        </mesh>
      ))}

      <mesh position={[0, 0, legH + bodyH / 2]} castShadow>
        <boxGeometry args={[dims.bodyL, dims.bodyW, bodyH]} />
        <meshStandardMaterial
          color={robot.tracking ? CHASSIS : "#94a3b8"}
          roughness={0.55}
        />
      </mesh>

      <Eyes
        bodyL={dims.bodyL}
        bodyW={dims.bodyW}
        bodyH={bodyH}
        faceZ={legH + bodyH * 0.55}
        tracking={robot.tracking}
      />
    </group>
  );
}

/** The OLED face on the front of the shell, with two shallow eye domes. */
function Eyes({
  bodyL,
  bodyW,
  bodyH,
  faceZ,
  tracking,
}: {
  bodyL: number;
  bodyW: number;
  bodyH: number;
  faceZ: number;
  tracking: boolean;
}) {
  const r = bodyW * 0.17;
  const white = tracking ? "#e8eef7" : "#cbd5e1";
  const pupil = tracking ? "#0b1220" : "#94a3b8";

  return (
    <group position={[bodyL / 2, 0, faceZ]}>
      <mesh position={[0.0008, 0, 0]}>
        <boxGeometry args={[0.0016, bodyW * 0.8, bodyH * 0.68]} />
        <meshStandardMaterial color="#1c2432" roughness={0.35} metalness={0.1} />
      </mesh>

      {[-1, 1].map((side) => (
        <group key={side} position={[0.0018, side * bodyW * 0.2, 0]}>
          <mesh scale={[0.3, 1, 1]}>
            <sphereGeometry args={[r, 20, 20]} />
            <meshStandardMaterial color={white} roughness={0.3} />
          </mesh>
          <mesh position={[r * 0.1, 0, r * 0.08]} scale={[0.3, 1, 1]}>
            <sphereGeometry args={[r * 0.42, 16, 16]} />
            <meshStandardMaterial color={pupil} roughness={0.25} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
