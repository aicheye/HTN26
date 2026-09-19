import {
  ARM_COLOR,
  ARM_JOINT_COLOR,
  ARM_LINK,
} from "./mapShared";
import type { ArmState } from "../types/world";

/**
 * SO-101 arm rendered as a nested joint chain matching the URDF: waist (yaw) ->
 * shoulder (pitch) -> elbow (pitch) -> wrist pitch -> wrist roll -> gripper.
 * Lives inside the same Z-up group as the floor and robot, so schema x/y/yaw
 * and this component's local "up" (+z) line up without extra conversion.
 */
export function ArmModel({ arm }: { arm: ArmState }) {
  const { waist, shoulder, elbow, wristPitch, wristRoll, gripper } = arm.joints;
  const L = ARM_LINK;

  return (
    <group position={[arm.mount.x, arm.mount.y, 0]} rotation={[0, 0, arm.mount.yaw]}>
      {/* square mounting plate clamped to the table edge, plus the motor stack on top */}
      <mesh position={[0, 0, L.plate / 2]} castShadow receiveShadow>
        <boxGeometry args={[0.11, 0.11, L.plate]} />
        <meshStandardMaterial color={ARM_COLOR} roughness={0.6} />
      </mesh>
      <mesh position={[0, 0, L.pedestal / 2 + L.plate]} castShadow>
        <boxGeometry args={[0.065, 0.065, L.pedestal]} />
        <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.6} metalness={0.2} />
      </mesh>

      <group position={[0, 0, L.pedestal + L.plate]} rotation={[0, 0, waist]}>
        <JointHub />
        <group rotation={[0, shoulder, 0]}>
          <Link length={L.upperArm} />
          <group position={[L.upperArm, 0, 0]} rotation={[0, elbow, 0]}>
            <JointHub small />
            <Link length={L.lowerArm} />
            <group position={[L.lowerArm, 0, 0]} rotation={[0, wristPitch, 0]}>
              <JointHub small />
              <Link length={L.wrist} thin />
              <group position={[L.wrist, 0, 0]} rotation={[wristRoll, 0, 0]}>
                <JointHub small />
                <Gripper length={L.gripper} jaw={L.jaw} openAngle={gripper} />
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

function JointHub({ small = false }: { small?: boolean }) {
  const r = small ? 0.032 : 0.042;
  return (
    <mesh rotation={[Math.PI / 2, 0, 0]} castShadow>
      <cylinderGeometry args={[r, r, r * 1.6, 16]} />
      <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.5} metalness={0.3} />
    </mesh>
  );
}

/** A link runs along local +x from the joint behind it to the one ahead. */
function Link({ length, thin = false }: { length: number; thin?: boolean }) {
  const h = thin ? 0.042 : 0.055;
  return (
    <mesh position={[length / 2, 0, 0]} castShadow>
      <boxGeometry args={[length, h, h]} />
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
  const spread = 0.009 + Math.max(0, openAngle) * 0.03;
  return (
    <group>
      <mesh position={[length / 2, 0, 0]} castShadow>
        <boxGeometry args={[length, 0.042, 0.042]} />
        <meshStandardMaterial color={ARM_JOINT_COLOR} roughness={0.5} metalness={0.2} />
      </mesh>
      {[-1, 1].map((side) => (
        <mesh
          key={side}
          position={[length + jaw / 2, side * spread, 0]}
          castShadow
        >
          <boxGeometry args={[jaw, 0.014, 0.03]} />
          <meshStandardMaterial color={ARM_COLOR} roughness={0.55} />
        </mesh>
      ))}
    </group>
  );
}
