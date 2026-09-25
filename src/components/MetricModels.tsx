import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { ExtrudeGeometry, Shape, Vector2, type Mesh } from "three";
import type { ArmState } from "../types/world";
import { armGeometry, PRISM_BEVEL, type Solid } from "../robot/geometry";

export function Solids({ solids, animate }: { solids: Solid[]; animate?: () => Solid[] }) {
  const meshes = useRef<(Mesh | null)[]>([]);
  useFrame(() => {
    if (!animate) return;
    animate().forEach((s, i) => {
      const mesh = meshes.current[i];
      if (!mesh) return;
      mesh.matrix.copy(s.matrix);
      mesh.matrixWorldNeedsUpdate = true;
    });
  });
  return <>{solids.map((s, i) => (
    <mesh key={s.name ?? i} ref={(m) => { meshes.current[i] = m; }} matrix={s.matrix} matrixAutoUpdate={false} castShadow receiveShadow>
      {s.kind === "prism" ? <FingerGeometry outline={s.outline} thickness={s.size[2]} />
        : s.kind === "box" ? <boxGeometry args={s.size} />
        : s.kind === "sphere" ? <sphereGeometry args={[s.size[0], 16, 12]} />
        : s.kind === "cylinder" ? <cylinderGeometry args={[s.size[0], s.size[0], s.size[1], 24]} />
        : <capsuleGeometry args={[s.size[0], s.size[1], 4, 16]} />}
      <meshStandardMaterial color={s.color} roughness={s.name === "hinge-pin" ? 0.3 : 0.65} metalness={s.name === "hinge-pin" ? 0.7 : 0} />
    </mesh>
  ))}</>;
}

function FingerGeometry({ outline, thickness }: { outline: [number, number][]; thickness: number }) {
  const geometry = useMemo(() => {
    const shape = new Shape(outline.map(([x, y]) => new Vector2(x, y)));
    const result = new ExtrudeGeometry(shape, {
      depth: thickness, bevelEnabled: true, bevelSize: PRISM_BEVEL, bevelThickness: PRISM_BEVEL, bevelSegments: 1, steps: 1,
    });
    result.translate(0, 0, -thickness / 2);
    return result;
  }, [outline, thickness]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return <primitive object={geometry} attach="geometry" />;
}

export function MetricArm({ arm }: { arm: ArmState }) {
  const solids = useMemo(() => armGeometry(arm.joints), [arm.joints.waist, arm.joints.shoulder, arm.joints.elbow,
    arm.joints.wristPitch, arm.joints.wristRoll, arm.joints.gripper]);
  return <group position={[arm.mount.x, arm.mount.y, 0]} rotation={[0, 0, arm.mount.yaw]}>
    <Solids solids={solids} />
  </group>;
}
