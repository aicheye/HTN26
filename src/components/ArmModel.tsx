import {
  ARM_COLOR,
  ARM_JOINT_COLOR,
  ARM_LINK,
} from "./mapShared";
import type { ArmState } from "../types/world";

/** Vertical gap so the forearm reads as a separate beam above the upper arm when folded flat. */
const FOREARM_LIFT = 0.026;

/** Simple rescue arm model using the same Z-up coordinates as the arena. */
export function ArmModel({ arm }: { arm: ArmState }) {
  const { waist, shoulder, elbow, wristPitch, wristRoll, gripper } = arm.joints;
  const L = ARM_LINK;

  return (
    <group position={[arm.mount.x, arm.mount.y, 0]} rotation={[0, 0, arm.mount.yaw]}>
      {/* clamped to the table edge: the plate straddles the boundary, half outside the arena */}
      <mesh position={[-0.025, 0, 0.017]} castShadow receiveShadow>
        <boxGeometry args={[0.085, 0.08, 0.034]} />
        <meshStandardMaterial color={ARM_COLOR} roughness={0.6} />
      </mesh>

      <mesh position={[0, 0, L.pedestal / 2 + 0.034]} castShadow>
        <boxGeometry args={[0.034, 0.034, L.pedestal]} />
        <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.6} metalness={0.2} />
      </mesh>

      <group position={[0, 0, L.pedestal + 0.034]} rotation={[0, 0, waist]}>
        <Servo />
        <mesh position={[0, 0, L.shoulderRise / 2]} castShadow>
          <boxGeometry args={[0.0155, 0.0166, L.shoulderRise]} />
          <meshStandardMaterial color={ARM_COLOR} roughness={0.55} />
        </mesh>

        <group position={[0, 0, L.shoulderRise]}>
          <Servo />
          <group rotation={[0, shoulder, 0]}>
            <ParallelUpperArm length={L.upperArm} />
            <group position={[L.upperArm, 0, 0]} rotation={[0, elbow, 0]}>
              <Servo />
              <mesh position={[0, 0, FOREARM_LIFT / 2]} castShadow>
                <boxGeometry args={[0.028, 0.028, FOREARM_LIFT + 0.028]} />
                <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.45} metalness={0.25} />
              </mesh>
              <group position={[0, 0, FOREARM_LIFT]}>
                <Link length={L.lowerArm} width={0.025} />
                <group position={[L.lowerArm, 0, 0]} rotation={[0, wristPitch, 0]}>
                  <Servo small />
                  <Link length={L.wrist} width={0.02} />
                  <group position={[L.wrist, 0, 0]} rotation={[wristRoll, 0, 0]}>
                    <Servo small />
                    <Gripper length={L.gripper} jaw={L.jaw} openAngle={gripper} />
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

/** Simplified parallel upper link. */
function ParallelUpperArm({ length }: { length: number }) {
  const gap = 0.026;
  return (
    <group>
      <group position={[0, -gap / 2, 0]}>
        <Link length={length} width={0.016} />
      </group>
      <group position={[0, gap / 2, 0]}>
        <Link length={length} width={0.016} />
      </group>
      {[0, length].map((x) => (
        <mesh key={x} position={[x, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
          <cylinderGeometry args={[0.0074, 0.0074, gap, 10]} />
          <meshStandardMaterial color={ARM_COLOR} roughness={0.55} />
        </mesh>
      ))}
    </group>
  );
}

function Servo({ small = false }: { small?: boolean }) {
  const r = small ? 0.0172 : 0.0235;
  const axle = small ? 0.0384 : 0.0482;
  return (
    <group>
      <mesh castShadow>
        <sphereGeometry args={[r, 16, 12]} />
        <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.45} metalness={0.25} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[r * 0.45, r * 0.45, axle, 14]} />
        <meshStandardMaterial color="#52525b" roughness={0.4} metalness={0.35} />
      </mesh>
    </group>
  );
}

function Link({ length, width }: { length: number; width: number }) {
  return (
    <mesh position={[length / 2, 0, 0]} rotation={[0, 0, -Math.PI / 2]} castShadow>
      <cylinderGeometry args={[width / 2, width / 2, length, 14]} />
      <meshStandardMaterial color={ARM_COLOR} roughness={0.55} />
    </mesh>
  );
}

function Gripper({
  length,
  jaw,
  openAngle,
}: {
  length: number;
  jaw: number;
  openAngle: number;
}) {
  const spread = 0.0049 + Math.max(0, openAngle) * 0.0217;
  return (
    <group>
      <mesh position={[length / 2, 0, 0]} rotation={[0, 0, -Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.0105, 0.0105, length, 12]} />
        <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.5} metalness={0.2} />
      </mesh>
      {[-1, 1].map((side) => (
        <mesh key={side} position={[length + jaw / 2, side * spread, 0]} castShadow>
          <boxGeometry args={[jaw, 0.0063, 0.0148]} />
          <meshStandardMaterial color={ARM_COLOR} roughness={0.55} />
        </mesh>
      ))}
    </group>
  );
}
