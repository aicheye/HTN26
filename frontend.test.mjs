import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { registerHooks } from "node:module";
import test from "node:test";
import ts from "typescript";

registerHooks({
  resolve(specifier, context, next) {
    if (specifier.startsWith(".") && !/\.[a-z]+$/i.test(specifier)) return next(`${specifier}.ts`, context);
    return next(specifier, context);
  },
  load(url, context, next) {
    if (url.endsWith("/src/data/sampleWorldState.json")) return { format: "module", shortCircuit: true, source: `export default ${readFileSync(new URL(url), "utf8")};` };
    if (url.endsWith(".ts")) return { format: "module", shortCircuit: true,
      source: ts.transpileModule(readFileSync(new URL(url), "utf8"), {
        compilerOptions: { target: ts.ScriptTarget.ES2020, module: ts.ModuleKind.ESNext },
      }).outputText };
    return next(url, context);
  },
});

const { Vector3 } = await import("three");
const { ARM_HOME, ARM_URDF_LIMITS, armFrames, armGeometry, origin, projectedOutline, sesameGeometry, urdfArmTip, solveUrdfArmIK } = await import("./src/robot/geometry.ts");
const { obstacleColor, obstacleHeight, obstacleOutline, distanceTo, OBSTACLE, ARM_REST_POSE, ARM_LIMITS } = await import("./src/components/mapShared.ts");
const near = (actual, expected, tolerance = 1e-7) => assert.ok(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);
const { MockSource } = await import("./src/sources/MockSource.ts");

test("Mock barrier test carries Sesame with the gripper and places it beyond the obstacle", (t) => {
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.runScenarioTest();
  const phases = new Set();
  let maxHeight = 0;
  for (let i = 0; i < 700; i++) {
    now += 50;
    const previous = { ...source.state.robots[0] };
    source.tick();
    const state = source.state, robot = state.robots[0], arm = state.arm;
    phases.add(arm.mode);
    maxHeight = Math.max(maxHeight, robot.z ?? 0);
    const radius = Math.hypot(robot.footprint.width, robot.footprint.length) / 2;
    for (const obstacle of state.obstacles) {
      if (distanceTo(obstacle, robot) < radius - 0.004) assert.ok((robot.z ?? 0) >= obstacle.height, "entire carried footprint must clear the obstacle");
    }
    assert.ok(Math.hypot(robot.x - previous.x, robot.y - previous.y, (robot.z ?? 0) - (previous.z ?? 0)) < 0.035, "robot must not teleport");
    if (["lifting", "carrying", "placing"].includes(arm.mode) && (robot.z ?? 0) > 0.002) {
      const world = origin([arm.mount.x, arm.mount.y, 0], [0, 0, arm.mount.yaw]);
      const pads = armGeometry(arm.joints).filter((s) => s.name === "fixed-pad" || s.name === "moving-pad");
      const contacts = pads.map((s, i) => new Vector3((i === 0 ? 1 : -1) * s.size[0] / 2, 0, 0).applyMatrix4(s.matrix).applyMatrix4(world));
      const grip = contacts[0].clone().add(contacts[1]).multiplyScalar(0.5);
      const robotFrame = origin([robot.x, robot.y, robot.z ?? 0], [0, 0, robot.yaw]);
      const handles = sesameGeometry(robot).solids.filter((s) => s.name?.endsWith("-handle"));
      const handlePoints = handles.map((s) => new Vector3(-0.002, 0.0735, 0).applyMatrix4(s.matrix).applyMatrix4(robotFrame));
      assert.ok(Math.min(...handlePoints.map((p) => p.distanceTo(grip))) < 0.002, "the rendered pads must grip a handle rail, not the body centre");
      const normal = new Vector3(1, 0, 0).transformDirection(world.clone().multiply(pads[0].matrix));
      const rail = new Vector3(Math.cos(robot.yaw), Math.sin(robot.yaw), 0);
      assert.ok(Math.abs(normal.dot(rail)) < 0.02, "jaws must close across the handle rail, not along it");
      const railWidth = 0.0052 * Math.abs(normal.dot(new Vector3(-rail.y, rail.x, 0))) + 0.0082 * Math.abs(normal.z);
      near(contacts[1].clone().sub(contacts[0]).dot(normal), railWidth, 0.001);
    }
    if (state.simulation.status === "complete" || state.simulation.status === "blocked") break;
  }
  assert.equal(source.state.simulation.status, "complete", source.state.simulation.message);
  assert.equal(source.state.simulation.message, "Reached the test goal.");
  const goal = source.state.simulation.testGoal;
  assert.ok(Math.hypot(source.truth.x - goal.x, source.truth.y - goal.y) < 0.06);
  for (const phase of ["reaching", "grasping", "lifting", "carrying", "placing", "releasing", "returning"]) assert.ok(phases.has(phase), phase);
  assert.ok(maxHeight > 0.085);
  assert.ok(source.state.robots[0].y > 0.43);
  near(source.state.robots[0].z, 0, 0.001);
});

for (const retargetDuringCarry of [false, true]) test(`Mock goto resumes after placement (${retargetDuringCarry ? "updated" : "original"} destination)`, (t) => {
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.runScenarioTest();
  const target = retargetDuringCarry ? { x: 0.48, y: 0.5 } : { x: 0.3, y: 0.53 };
  const command = { id: "resume-goto", type: "goto", ts: 0, robotId: "sesame-1", target, speed: 0.3 };
  if (!retargetDuringCarry) source.sendCommand(command);
  for (let i = 0; i < 400 && source.state.arm.mode !== "carrying"; i++) { now += 50; source.tick(); }
  assert.equal(source.state.arm.mode, "carrying", source.state.simulation.message);
  if (retargetDuringCarry) source.sendCommand(command);
  for (let i = 0; i < 400 && source.state.arm.mode !== "idle"; i++) { now += 50; source.tick(); }
  assert.equal(source.state.arm.mode, "idle", source.state.simulation.message);
  const landing = { ...source.truth };
  assert.ok(Math.hypot(landing.x - target.x, landing.y - target.y) > 0.06);
  assert.equal(source.state.simulation.status, "running");
  assert.deepEqual(source.state.goal, target);
  assert.deepEqual(source.state.path, [{ x: landing.x, y: landing.y }, target]);
  assert.deepEqual(source.active.command, command);
  let walked = false;
  for (let i = 0; i < 400 && source.state.simulation.status === "running"; i++) {
    now += 50;
    source.tick();
    walked ||= source.state.robots[0].mode === "moving";
    assert.equal(source.state.arm.mode, "idle");
    near(source.state.robots[0].z, 0);
  }
  assert.ok(walked, "robot must walk from the landing point toward the destination");
  assert.ok(Math.hypot(source.truth.x - target.x, source.truth.y - target.y) < 0.06);
  assert.equal(source.state.simulation.status, "complete", source.state.simulation.message);
  assert.equal(source.state.simulation.message, "Reached the test goal.");
  assert.equal(source.active, null);
  assert.equal(source.state.goal, undefined);
  assert.equal(source.state.path, undefined);
});

test("Mock claw closes only at the handle, places before opening, and retreats without dragging Sesame", async (t) => {
  const { armGripPosition, sesameHandleTargets } = await import("./src/robot/geometry.ts");
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.runScenarioTest();
  let closedAtHandle = false, openedOnFloor = false, retreated = false;
  for (let i = 0; i < 700 && source.state.simulation.status === "running"; i++) {
    const before = { ...source.truth }, height = source.state.robots[0].z;
    const phase = source.state.arm.mode, opening = source.gripOpening;
    const previousGrip = armGripPosition(source.state.arm.mount, source.state.arm.joints);
    now += 50;
    source.tick();
    const robot = source.state.robots[0], arm = source.state.arm;
    const grip = armGripPosition(arm.mount, arm.joints);
    if (phase === "grasping" && source.gripOpening < opening - 0.001) {
      const handles = sesameHandleTargets({ ...robot, yaw: source.truth.yaw });
      assert.ok(Math.min(...handles.map(({ offset }) => grip.distanceTo(offset.clone().add(new Vector3(before.x, before.y, 0))))) < 0.002);
      assert.deepEqual(source.truth, before);
      near(robot.z, 0);
      closedAtHandle = true;
    }
    if (phase === "releasing") {
      assert.deepEqual(source.truth, before);
      near(height, 0);
      near(robot.z, 0);
      assert.ok(source.gripOpening >= opening);
      openedOnFloor = true;
    }
    if (phase === "returning" && source.armElapsed < 400 && arm.mode === "returning") {
      assert.deepEqual(source.truth, before);
      near(robot.z, 0);
      assert.ok(source.gripOpening > 0.99);
      assert.ok(grip.z >= previousGrip.z - 0.001);
      retreated = true;
    }
  }
  assert.equal(source.state.simulation.status, "complete", source.state.simulation.message);
  assert.ok(closedAtHandle && openedOnFloor && retreated);
});

test("Handle IK matches the rendered pad contacts and rotated handle rails", async () => {
  const { armGripPosition, sesameHandleTargets } = await import("./src/robot/geometry.ts");
  const mount = { x: 0, y: 0.3175, yaw: 0, side: "west" };
  for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
    const robot = { tracking: true, yaw };
    for (const handle of sesameHandleTargets(robot)) {
      const target = handle.offset.clone().add(new Vector3(0.19, 0.3175, 0));
      const joints = solveUrdfArmIK(mount, target, target.z, ARM_HOME, { yaw, opening: 0 });
      const grip = armGripPosition(mount, joints);
      assert.ok(grip.distanceTo(target) < 0.002, `${handle.name} at yaw ${yaw}`);
      const pads = armGeometry(joints).filter((s) => s.name === "fixed-pad" || s.name === "moving-pad");
      const world = origin([mount.x, mount.y, 0], [0, 0, mount.yaw]);
      const contacts = pads.map((s, i) => new Vector3((i === 0 ? 1 : -1) * s.size[0] / 2, 0, 0).applyMatrix4(s.matrix).applyMatrix4(world));
      assert.ok(grip.distanceTo(contacts[0].clone().add(contacts[1]).multiplyScalar(0.5)) < 1e-8);
      const normal = new Vector3(1, 0, 0).transformDirection(world.clone().multiply(pads[0].matrix));
      near(normal.dot(new Vector3(Math.cos(yaw), Math.sin(yaw), 0)), 0, 1e-7);
      const width = 0.0052 * Math.abs(normal.dot(new Vector3(-Math.sin(yaw), Math.cos(yaw), 0))) + 0.0082 * Math.abs(normal.z);
      near(contacts[1].clone().sub(contacts[0]).dot(normal), width, 1e-6);
      for (const [name, value] of Object.entries(joints)) {
        assert.ok(value >= ARM_URDF_LIMITS[name][0] && value <= ARM_URDF_LIMITS[name][1]);
      }
    }
  }
});

test("Unreachable mock obstacle does not claim a successful rescue", (t) => {
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.resetScenario("unreachable");
  source.runScenarioTest();
  for (let i = 0; i < 200; i++) { now += 50; source.tick(); }
  assert.equal(source.state.simulation.status, "blocked");
  assert.equal(source.state.arm.mode, "idle");
  near(source.state.robots[0].z, 0);
});

test("Mock stop freezes a carried robot and reset restores a clean scene", (t) => {
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.runScenarioTest();
  for (let i = 0; i < 400 && source.state.arm.mode !== "carrying"; i++) { now += 50; source.tick(); }
  assert.equal(source.state.arm.mode, "carrying");
  source.sendCommand({ id: "stop-test", type: "stop", ts: 0, robotId: "sesame-1" });
  const robot = structuredClone(source.state.robots[0]);
  const joints = structuredClone(source.state.arm.joints);
  for (let i = 0; i < 30; i++) { now += 50; source.tick(); }
  assert.deepEqual(source.state.arm.joints, joints);
  near(source.state.robots[0].x, robot.x);
  near(source.state.robots[0].y, robot.y);
  near(source.state.robots[0].z, robot.z);
  source.resetScenario("empty");
  assert.equal(source.state.obstacles.length, 0);
  assert.equal(source.state.arm.mode, "idle");
  near(source.state.robots[0].z, 0);
});

test("Mixed detection scene also completes the carry test", (t) => {
  let now = 0;
  t.mock.method(performance, "now", () => now);
  const source = new MockSource();
  source.resetScenario("mixed");
  source.runScenarioTest();
  assert.deepEqual(source.state.obstacles.map((o) => o.shape), ["rect", "circle", "polygon"]);
  for (let i = 0; i < 700 && source.state.simulation.status === "running"; i++) { now += 50; source.tick(); }
  assert.equal(source.state.simulation.status, "complete", source.state.simulation.message);
});

test("Arm geometry connects its pedestal and moving jaw to their parent assemblies", () => {
  const solids = armGeometry(ARM_HOME);
  const frames = armFrames(ARM_HOME);
  const waist = new Vector3().setFromMatrixPosition(frames[1]);
  const roll = new Vector3().setFromMatrixPosition(frames[5]);
  const jaw = new Vector3().setFromMatrixPosition(frames[6]);
  const hasBeam = (a, b) => solids.some((s) => {
    if (s.kind !== "capsule" && s.kind !== "box") return false;
    const ends = [-1, 1].map((sign) => new Vector3(0, sign * s.size[1] / 2, 0).applyMatrix4(s.matrix));
    return ends[0].distanceTo(a) < 1e-6 && ends[1].distanceTo(b) < 1e-6;
  });
  assert.ok(hasBeam(new Vector3(waist.x, waist.y, 0.016), waist));
  assert.ok(hasBeam(roll, jaw));
  assert.ok(urdfArmTip({ x: 0, y: 0, yaw: 0 }, ARM_HOME).z > 0.1);
});

test("Claw has angular fingers and gripping pads that open around the URDF pivot", async () => {
  const { clawGeometry } = await import("./src/robot/geometry.ts");
  const closed = clawGeometry({ ...ARM_HOME, gripper: 0 });
  const open = clawGeometry({ ...ARM_HOME, gripper: 0.9 });
  assert.equal(closed.filter((s) => s.kind === "prism").length, 2);
  assert.ok(closed.every((s) => s.kind !== "capsule"));
  const part = (parts, name) => parts.find((s) => s.name === name);
  assert.deepEqual(part(closed, "fixed-finger").matrix.elements, part(open, "fixed-finger").matrix.elements);
  assert.notDeepEqual(part(closed, "moving-finger").matrix.elements, part(open, "moving-finger").matrix.elements);
  const gap = (parts) => new Vector3().setFromMatrixPosition(part(parts, "fixed-pad").matrix)
    .distanceTo(new Vector3().setFromMatrixPosition(part(parts, "moving-pad").matrix));
  assert.ok(gap(open) > gap(closed) + 0.035);
  assert.ok(closed.every((s) => projectedOutline(s).length >= 3));
});

test("Walking animation respects live joints and stops while carried or idle", async () => {
  const { displayRobot } = await import("./src/robot/geometry.ts");
  const robot = { tracking: true, mode: "moving" };
  assert.notDeepEqual(displayRobot(robot, 0).joints, displayRobot(robot, 0.2).joints);
  const live = { ...robot, joints: { R1: 1.4 } };
  assert.equal(displayRobot(live, 0.2), live);
  assert.equal(displayRobot(robot, 0.2, true), robot);
  const idle = { ...robot, mode: "idle" };
  assert.equal(displayRobot(idle, 0.2), idle);
});

test("URDF fixed-axis RPY applies roll, then pitch, then yaw", () => {
  const p = new Vector3(0, 1, 0).applyMatrix4(origin([1, 2, 3], [Math.PI / 2, Math.PI / 2, 0]));
  near(p.x, 2); near(p.y, 2); near(p.z, 3);
});

test("SO-101 joint centres preserve the CAD baseframe offset and link distances", () => {
  const zero = Object.fromEntries(Object.keys(ARM_HOME).map((key) => [key, 0]));
  const frames = armFrames(zero);
  const points = frames.map((m) => new Vector3().setFromMatrixPosition(m));
  near(points[1].x, 0.038836); near(points[1].y, 0); near(points[1].z, 0.0624);
  near(points[2].distanceTo(points[3]), Math.hypot(0.11257, 0.028));
  near(points[3].distanceTo(points[4]), Math.hypot(0.1349, 0.0052));
  near(points[4].distanceTo(points[5]), Math.hypot(0.0611, 0.0181));
  assert.deepEqual(ARM_LIMITS, ARM_URDF_LIMITS);
  for (const [name, value] of Object.entries(ARM_REST_POSE)) {
    assert.ok(value >= ARM_LIMITS[name][0] && value <= ARM_LIMITS[name][1]);
  }
});

test("SO-101 enforces limits and wrist roll moves the offset tool frame", () => {
  const limited = armFrames({ ...ARM_HOME, shoulder: 100 });
  const expected = armFrames({ ...ARM_HOME, shoulder: ARM_URDF_LIMITS.shoulder[1] });
  assert.deepEqual(limited.map((m) => m.elements), expected.map((m) => m.elements));
  const mount = { x: 0, y: 0, yaw: 0, side: "east" };
  const a = urdfArmTip(mount, ARM_HOME);
  const b = urdfArmTip(mount, { ...ARM_HOME, wristRoll: 1 });
  assert.ok(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z) > 0.001);
});

test("Arm IK and render geometry share the same forward chain", () => {
  const mount = { x: 0.635, y: 0.3175, yaw: Math.PI, side: "east" };
  const desired = { ...ARM_HOME, shoulder: -1.1, elbow: 1.2 };
  const target = urdfArmTip(mount, desired);
  const result = solveUrdfArmIK(mount, target, target.z);
  const tip = urdfArmTip(mount, result);
  assert.ok(Math.hypot(tip.x - target.x, tip.y - target.y, tip.z - target.z) < 0.005);
  assert.ok(armGeometry(result).every((s) => s.matrix.elements.every(Number.isFinite)));
});

test("Sesame uses the 84 x 68 x 62 mm body and actual articulated leg geometry", () => {
  const model = sesameGeometry({ tracking: true });
  const body = model.solids.find((s) => s.name === "shell");
  assert.equal(body.kind, "prism");
  const corners = body.outline.flatMap(([x, y]) => [-1, 1].map((side) =>
    new Vector3(x, y, side * body.size[2] / 2).applyMatrix4(body.matrix)));
  for (const [axis, span] of [["x", 0.084], ["y", 0.068], ["z", 0.062]]) {
    near(Math.max(...corners.map((p) => p[axis])) - Math.min(...corners.map((p) => p[axis])), span);
  }
  const feet = model.solids.filter((s) => s.kind === "sphere" && s.size[0] === 0.007);
  assert.equal(feet.length, 4);
  feet.forEach((foot) => assert.ok(foot.matrix.elements[14] - 0.007 >= -1e-9));
  const moved = sesameGeometry({ tracking: true, joints: { R1: 2 } });
  assert.notDeepEqual(moved.solids.find((s) => s.name === "R3-servo").matrix.elements,
    model.solids.find((s) => s.name === "R3-servo").matrix.elements);
  assert.ok(model.solids.every((s) => projectedOutline(s).length >= 3));
});

test("Photo-inspired Sesame has two open side handles, a sloped shell, and four angular servo legs", async () => {
  const { ShapeUtils, Vector2 } = await import("three");
  const model = sesameGeometry({ tracking: true });
  const handles = model.solids.filter((s) => s.name?.endsWith("-handle"));
  assert.equal(handles.length, 2);
  const shell = model.solids.find((s) => s.name === "shell");
  assert.ok(shell.outline.length > 4);
  const contains = (outline, x, y) => {
    const points = outline.map(([px, py]) => new Vector2(px, py));
    return ShapeUtils.triangulateShape(points, []).some(([a, b, c]) => {
      const cross = (p, q) => (q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x);
      const signs = [cross(points[a], points[b]), cross(points[b], points[c]), cross(points[c], points[a])];
      return signs.every((v) => v >= 0) || signs.every((v) => v <= 0);
    });
  };
  for (const handle of handles) {
    assert.equal(handle.kind, "prism");
    assert.ok(Math.abs(handle.matrix.elements[13] - 0.004) >= 0.032);
    assert.ok(!contains(handle.outline, 0, 0.06), "handle must have a real open centre");
    assert.ok(contains(handle.outline, 0, 0.074), "handle needs a top grip");
    assert.ok(contains(handle.outline, -0.027, 0.04), "handle must connect to the shell");
    assert.ok(Math.max(...handle.outline.map((p) => p[1])) > 0.07);
  }
  assert.equal(model.solids.filter((s) => s.name?.endsWith("-servo")).length, 8);
  assert.equal(model.solids.filter((s) => s.name?.endsWith("-shin") && s.kind === "prism").length, 4);
  assert.ok(model.solids.every((s) => s.kind !== "capsule"));
  const recolored = sesameGeometry({ tracking: true, shellColor: "#345678" });
  for (const name of ["shell", "left-handle", "right-handle"]) {
    assert.equal(recolored.solids.find((s) => s.name === name).color, "#345678");
  }
  const moved = sesameGeometry({ tracking: true, joints: { R1: 2 } });
  for (const handle of handles) {
    const other = moved.solids.find((s) => s.name === handle.name);
    near(other.matrix.elements[14] - moved.baseHeight, handle.matrix.elements[14] - model.baseHeight);
    assert.deepEqual(other.outline, handle.outline);
  }
});

test("Simplified top view keeps four articulated legs and two handles without internal mesh faces", async () => {
  const { sesameTopView } = await import("./src/robot/geometry.ts");
  const robot = { tracking: true };
  const view = sesameTopView(robot);
  assert.equal(view.legs.length, 4);
  assert.equal(view.handles.length, 2);
  near(Math.max(...view.shell.map((p) => p.x)) - Math.min(...view.shell.map((p) => p.x)), 0.084);
  near(Math.max(...view.shell.map((p) => p.y)) - Math.min(...view.shell.map((p) => p.y)), 0.068);
  const feet = sesameGeometry(robot).solids.filter((s) => s.name?.endsWith("-foot"));
  view.legs.forEach((leg, i) => assert.ok(leg.foot.distanceTo(new Vector3().setFromMatrixPosition(feet[i].matrix)) < 1e-8));
  const moved = sesameTopView({ ...robot, joints: { R1: 2 } });
  assert.notDeepEqual(moved.legs[0].foot.toArray(), view.legs[0].foot.toArray());
  assert.ok(view.handles.every((handle) => handle.length <= 4));
});

test("Compact 2D leg spread tucks the legs in without changing the 3D model or handles", async () => {
  const { sesameTopView } = await import("./src/robot/geometry.ts");
  for (const joints of [undefined, { R1: 2 }]) {
    const robot = { tracking: true, joints };
    const full = sesameTopView(robot);
    const compact = sesameTopView(robot, 0.6);
    assert.deepEqual(compact.shell, full.shell);
    assert.deepEqual(compact.handles, full.handles);
    compact.legs.forEach((leg, i) => {
      assert.deepEqual(leg.hip, full.legs[i].hip);
      for (const part of ["knee", "foot"]) {
        for (const axis of ["x", "y"]) near(leg[part][axis] - leg.hip[axis], (full.legs[i][part][axis] - leg.hip[axis]) * 0.6);
        near(leg[part].z, full.legs[i][part].z);
      }
    });
    assert.deepEqual(sesameTopView(robot), full);
  }
});

test("Top-down projection keeps real dimensions and yaw", () => {
  const box = { kind: "box", size: [0.084, 0.068, 0.062], matrix: origin([0.2, 0.3, 0], [0, 0, Math.PI / 2]), color: "#000000" };
  const points = projectedOutline(box);
  near(Math.max(...points.map((p) => p.x)) - Math.min(...points.map((p) => p.x)), 0.068);
  near(Math.max(...points.map((p) => p.y)) - Math.min(...points.map((p) => p.y)), 0.084);
});

test("Obstacle colour and unknown-height fallbacks do not invent measurements", () => {
  const obstacle = { id: "test", source: "cv", shape: "rect", x: 0.2, y: 0.3, yaw: Math.PI / 2, width: 0.1, length: 0.2 };
  assert.equal(obstacleColor({ ...obstacle, color: "#20A0f0" }), "#20A0f0");
  assert.equal(obstacleColor({ ...obstacle, color: "not-a-colour" }), OBSTACLE);
  assert.equal(obstacleColor(obstacle), OBSTACLE);
  assert.equal(obstacleHeight(obstacle), 0.004);
  assert.equal(obstacleHeight({ ...obstacle, height: 0.06 }), 0.06);
  assert.equal(obstacleHeight({ ...obstacle, height: NaN }), 0.004);
  const points = obstacleOutline(obstacle);
  assert.deepEqual(points[0], points.at(-1));
  near(Math.max(...points.map((p) => p.x)) - Math.min(...points.map((p) => p.x)), 0.2);
});
