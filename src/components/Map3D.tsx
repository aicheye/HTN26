import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useThree, type ThreeEvent } from "@react-three/fiber";
import { Line, OrbitControls, useTexture } from "@react-three/drei";
import * as THREE from "three";
import type { MapProps } from "./MapProps";
import { MetricArm as ArmModel, Solids } from "./MetricModels";
import { displayRobot, sesameGeometry } from "../robot/geometry";
import { cornerTags, markerUrls, tableBorder, woodCanvas } from "./sceneSurface";
import {
  FLOOR,
  FLOOR_EDGE,
  ROBOT_FITTING,
  ROBOT_FITTING_INSET,
  GOAL,
  GRID,
  hasContour,
  obstacleColor,
  obstacleHeight,
  OBSTACLE_DANGER,
  TILE_M,
  obstacleOutline,
  ARM_CARRY_LIFT_M,
  ARM_MAX_REACH,
  isCarried,
  isDanger,
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
export function Map3D({
  state,
  showCameraLayer = false,
  compact = false,
  onPickGoal,
}: MapProps) {
  const { width, length } = state.arena;
  const [view, setView] = useState<ViewMode>("iso");
  const robot = state.robots.find((r) => r.tracking) ?? state.robots[0];

  return (
    <div className="relative h-full w-full">
      <Canvas
        shadows
        dpr={[1, 2]}
        camera={{
          position: [width / 2, Math.max(width, length) * 1.6, length * 1.3],
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
          shadow-mapSize={[2048, 2048]}
          shadow-bias={-0.0001}
          shadow-normalBias={0.001}
          shadow-camera-left={-width}
          shadow-camera-right={width}
          shadow-camera-top={length}
          shadow-camera-bottom={-length}
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
            <RobotModel key={r.id} robot={r} carried={isCarried(state.arm, r.id)} />
          ))}
          {state.arm && <ArmModel arm={state.arm} />}
        </group>

        <OrbitControls
          makeDefault
          enabled={!compact}
          enableDamping
          dampingFactor={0.1}
          rotateSpeed={0.55}
          panSpeed={0.7}
          zoomSpeed={0.8}
          zoomToCursor
          screenSpacePanning={false}
          target={[width / 2, 0, -length / 2]}
          minDistance={0.3}
          maxDistance={Math.max(width, length) * 4}
          maxPolarAngle={Math.PI / 2 - 0.06}
        />

        <CameraRig
          arena={state.arena}
          robot={robot}
          view={view}
          compact={compact}
          onUserTakeOver={() => setView((v) => (v === "follow" ? v : "free"))}
        />
      </Canvas>

      {!compact && (
        <div className="absolute left-3 top-3 flex gap-1 rounded-lg border border-zinc-700 bg-zinc-900/85 p-1 shadow-lg backdrop-blur">
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
                  ? "bg-zinc-100 text-zinc-900"
                  : "text-zinc-300 hover:bg-zinc-800"
              }`}
            >
              {label}
            </button>
          ))}
        </div>
      )}
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
  compact,
  onUserTakeOver,
}: {
  arena: Arena;
  robot: Robot | undefined;
  view: ViewMode;
  compact: boolean;
  onUserTakeOver: () => void;
}) {
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as
    | { target: THREE.Vector3; addEventListener: Function; removeEventListener: Function }
    | null;
  // Include the tabletop border and arm in the presets, including portrait viewports.
  const size = useThree((s) => s.size);
  const span = Math.max(arena.width + 2 * tableBorder(arena), arena.length + 2 * tableBorder(arena), ARM_MAX_REACH * 1.3)
    * Math.max(1, size.height / Math.max(size.width, 1));
  const desired = useRef<{ pos: THREE.Vector3; target: THREE.Vector3 } | null>(null);

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
        pos: new THREE.Vector3(center.x, span * 1.55, center.z + 0.001),
        target: center,
      };
    } else if (view === "iso") {
      desired.current = {
        pos: new THREE.Vector3(center.x, span * 1.25, center.z + span * 1.35),
        target: center,
      };
    } else {
      desired.current = null;
    }
  }, [view, arena.width, arena.length, span]);

  useFrame((_, dt) => {
    const center = new THREE.Vector3(arena.width / 2, 0, -arena.length / 2);

    // the thumbnail has no controls to aim the camera, so frame it directly
    if (compact) {
      camera.position.set(center.x, span * 0.95, center.z + span * 1.05);
      camera.lookAt(center);
      return;
    }

    if (!controls) return;
    const k = 1 - Math.exp(-6 * dt);

    if (view === "follow" && robot) {
      const target = new THREE.Vector3(robot.x, 0, -robot.y);
      // read the offset fresh each frame so orbiting and zooming still work
      const offset = camera.position.clone().sub(controls.target);
      if (offset.length() > span * 1.5) offset.setLength(span * 0.6);
      controls.target.lerp(target, k);
      camera.position.copy(controls.target).add(offset);
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
    const x = e.point.x, y = -e.point.z;
    if (x >= 0 && y >= 0 && x <= arena.width && y <= arena.length) onPickGoal({ x, y });
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
        <planeGeometry args={[arena.width + tableBorder(arena) * 2, arena.length + tableBorder(arena) * 2]} />
        <meshStandardMaterial
          map={texture}
          color={dimmed ? "#94a3b8" : "#ffffff"}
          roughness={0.95}
        />
      </mesh>
      <mesh position={[arena.width / 2, arena.length / 2, -0.010]} receiveShadow castShadow>
        <boxGeometry args={[arena.width + tableBorder(arena) * 2, arena.length + tableBorder(arena) * 2, 0.018]} />
        <meshStandardMaterial color="#aa8757" roughness={0.8} />
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

/** Shared wood surface, or a repeated tile when the arena explicitly requests a grid. */
function useTileTexture(arena: Arena) {
  const texture = useMemo(() => {
    if (arena.surface !== "grid") {
      const wood = new THREE.CanvasTexture(woodCanvas());
      wood.colorSpace = THREE.SRGBColorSpace;
      wood.anisotropy = 8;
      return wood;
    }
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
  }, [arena.width, arena.length, arena.surface]);
  useEffect(() => () => texture.dispose(), [texture]);
  return texture;
}

function CornerTags({ arena }: { arena: Arena }) {
  return <>{cornerTags(arena).map(({ id, x, y }) => (
    <group key={id} position={[x, y, 0.003]}>
      <Marker id={id} size={arena.tagSize ?? 0.08} />
    </group>
  ))}</>;
}

function Marker({ id, size }: { id: number; size: number }) {
  return <>
    <mesh><planeGeometry args={[size * 1.25, size * 1.25]} /><meshBasicMaterial color="white" /></mesh>
    {markerUrls[id] && <MarkerInk url={markerUrls[id]} size={size} />}
  </>;
}

function MarkerInk({ url, size }: { url: string; size: number }) {
  const texture = useTexture(url);
  texture.magFilter = THREE.NearestFilter;
  return <mesh position={[0, 0, 0.0002]}><planeGeometry args={[size, size]} /><meshBasicMaterial map={texture} toneMapped={false} /></mesh>;
}

/** Loads an obstacle's photo without suspending the scene, and keeps showing the previous photo until the new one
 *  has loaded, so a rescan does not make the object blink. */
function useObstaclePhoto(url: string | undefined) {
  const [photo, setPhoto] = useState<THREE.Texture | null>(null);
  useEffect(() => {
    if (!url) { setPhoto(null); return; }
    let cancelled = false;
    new THREE.TextureLoader().load(url, (loaded) => {
      if (cancelled) { loaded.dispose(); return; }
      loaded.colorSpace = THREE.SRGBColorSpace;
      loaded.anisotropy = 4;
      setPhoto(loaded);
    });
    return () => { cancelled = true; };
  }, [url]);
  useEffect(() => () => photo?.dispose(), [photo]);
  return photo;
}

function ObstacleMesh({ obstacle: o, danger }: { obstacle: Obstacle; danger: boolean }) {
  const height = obstacleHeight(o);
  const color = obstacleColor(o);
  const outline = <Line points={obstacleOutline(o).map((p) => [p.x, p.y, height + 0.001] as [number, number, number])}
    color={danger ? OBSTACLE_DANGER : "#334155"} lineWidth={danger ? 2 : 1} dashed={o.height === undefined} dashSize={0.01} gapSize={0.006} />;

  // State arrives 10 times a second with a new points array each time. The geometry is rebuilt only when the
  // contour itself changes.
  const contour = hasContour(o) ? o.points!.map((p) => `${p.x},${p.y}`).join(" ") : "";
  const extruded = useMemo(() => {
    if (!contour) return null;
    const pts = o.points!.map((p) => new THREE.Vector2(p.x - o.x, p.y - o.y));
    // ExtrudeGeometry needs counter-clockwise winding or the normals invert
    if (THREE.ShapeUtils.isClockWise(pts)) pts.reverse();
    return new THREE.ExtrudeGeometry(new THREE.Shape(pts), {
      depth: height,
      bevelEnabled: false,
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [contour, o.x, o.y, height]);

  useEffect(() => () => extruded?.dispose(), [extruded]);
  const photo = useObstaclePhoto(o.textureUrl);
  // The photo lies on the top face, covering the oriented box. It is transparent outside the object's contour.
  const decal = photo && (
    <mesh position={[o.x, o.y, height + 0.0006]} rotation={[0, 0, o.yaw]}>
      <planeGeometry args={[o.width ?? 0.2, o.length ?? 0.2]} />
      <meshBasicMaterial map={photo} transparent alphaTest={0.5} toneMapped={false} />
    </mesh>
  );

  if (extruded) {
    return (
      <>{outline}{decal}<mesh geometry={extruded} position={[o.x, o.y, 0]} castShadow receiveShadow>
        <meshStandardMaterial color={color} roughness={0.8} />
      </mesh></>
    );
  }

  if (o.shape === "circle") {
    return (
      <>{outline}<mesh
        position={[o.x, o.y, height / 2]}
        rotation={[Math.PI / 2, 0, 0]}
        castShadow
        receiveShadow
      >
        <cylinderGeometry args={[o.radius ?? 0.1, o.radius ?? 0.1, height, 28]} />
        <meshStandardMaterial color={color} roughness={0.8} />
      </mesh></>
    );
  }

  return (
    <>{outline}{decal}<mesh
      position={[o.x, o.y, height / 2]}
      rotation={[0, 0, o.yaw]}
      castShadow
      receiveShadow
    >
      <boxGeometry args={[o.width ?? 0.2, o.length ?? 0.2, height]} />
      <meshStandardMaterial color={color} roughness={0.8} />
    </mesh></>
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

function RobotModel({ robot, carried = false }: { robot: Robot; carried?: boolean }) {
  const group = useRef<THREE.Group>(null);
  const model = sesameGeometry(robot);

  // detections arrive ~20 Hz; damp toward them so the model glides
  const target = useRef({ x: robot.x, y: robot.y, yaw: robot.yaw });
  target.current = { x: robot.x, y: robot.y, yaw: robot.yaw };

  useFrame((_, dt) => {
    const g = group.current;
    if (!g) return;
    const k = carried || (robot.z ?? 0) > 0 ? 1 : 1 - Math.exp(-12 * dt);
    g.position.x += (target.current.x - g.position.x) * k;
    g.position.y += (target.current.y - g.position.y) * k;
    const d = target.current.yaw - g.rotation.z;
    g.rotation.z += Math.atan2(Math.sin(d), Math.cos(d)) * k;
  });

  return (
    <group ref={group} position={[robot.x, robot.y, robot.z ?? (carried ? ARM_CARRY_LIFT_M : 0)]} rotation={[0, 0, robot.yaw]}>
      <Solids solids={model.solids} animate={() => sesameGeometry(displayRobot(robot, performance.now() / 1000, carried)).solids} />
      <group position={[0.001, 0.004, model.baseHeight + 0.047]} rotation={[0, 0, -Math.PI / 2]}>
        <Marker id={robot.tagId} size={0.036} />
      </group>
      <Eyes bodyL={0.086} bodyW={0.068} bodyH={0.062} faceZ={model.baseHeight + 0.002} tracking={robot.tracking} />
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
  const r = bodyW * 0.075;
  const white = tracking ? ROBOT_FITTING : "#d5dde5";
  const pupil = tracking ? ROBOT_FITTING_INSET : "#aab6c1";

  return (
    <group position={[bodyL / 2, 0.004, faceZ]}>
      <mesh position={[0.0008, 0, 0]}>
        <boxGeometry args={[0.0016, bodyW * 0.82, bodyH * 0.4]} />
        <meshStandardMaterial color="#1c1e22" roughness={0.8} />
      </mesh>

      {[-1, 1].map((side) => (
        <group key={side} position={[0.0018, side * bodyW * 0.2, 0]}>
          <mesh scale={[0.45, 1, 1]}>
            <sphereGeometry args={[r, 20, 20]} />
            <meshStandardMaterial color={white} roughness={0.4} metalness={0.25} />
          </mesh>
          {/* pushed past the dome surface so the pupil reads from the front and above */}
          <mesh position={[r * 0.3, 0, r * 0.06]} scale={[0.45, 1, 1]}>
            <sphereGeometry args={[r * 0.5, 16, 16]} />
            <meshStandardMaterial color={pupil} roughness={0.2} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
