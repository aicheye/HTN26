// Walks the Sesame robot from the laptop by streaming servo poses over the firmware's WebSocket
// ({"servos": {...}}), so the gait can be changed without reflashing the robot.
//
// Leg layout (from the firmware's turn gait): hip R1 has knee R3, hip R2 has knee R4, hip L1 has knee L3,
// hip L2 has knee L4. R1 with L2 is one diagonal pair, R2 with L1 the other.
// Knees: a leg is lifted at R3 135, R4 45, L3 45, L4 135 and down at R3 180, R4 0, L3 0, L4 180.

export const STAND = { R1: 135, R2: 45, L1: 45, L2: 135, R4: 0, R3: 180, L3: 0, L4: 180 };

// The firmware's own gaits, frame for frame (checked against movement-sequences.h by gait.test.mjs).
// In its walk, the right legs are in the air for 4 of 6 frames and the left legs for 2 of 6. In two frames
// per cycle both right legs are up at once, so the robot rests on its left legs only.
export const FIRMWARE = {
  forward: {
    start: [{ R3: 135, L3: 45, R2: 100, L1: 25 }],
    cycle: [{ R3: 135, L3: 0 }, { L4: 135, L2: 90, R4: 0, R1: 180 }, { R2: 45, L1: 90 }, { R4: 45, L4: 180 }, { R3: 180, L3: 45, R2: 90, L1: 0 }, { L2: 135, R1: 90 }],
  },
  backward: {
    start: [],
    cycle: [{ R3: 135, L3: 0 }, { L4: 135, L2: 135, R4: 0, R1: 90 }, { R2: 90, L1: 0 }, { R4: 45, L4: 180 }, { R3: 180, L3: 45, R2: 45, L1: 90 }, { L2: 90, R1: 180 }],
  },
  left: {
    start: [],
    cycle: [{ R3: 135, L4: 135 }, { R1: 180, L2: 180 }, { R3: 180, L4: 180 }, { R1: 135, L2: 135 }, { R4: 45, L3: 45 }, { R2: 90, L1: 90 }, { R4: 0, L3: 0 }, { R2: 45, L1: 45 }],
  },
  right: {
    start: [],
    cycle: [{ R4: 45, L3: 45 }, { R2: 0, L1: 0 }, { R4: 0, L3: 0 }, { R2: 45, L1: 45 }, { R3: 135, L4: 135 }, { R1: 90, L2: 90 }, { R3: 180, L4: 180 }, { R1: 135, L2: 135 }],
  },
};

// A trot with the same hip angles as the firmware walk but mirror-symmetric timing: the two diagonal pairs
// alternate, the airborne pair swings forward while the grounded pair pushes, and each leg is in the air for
// the same share of the cycle. Turning in place uses the firmware's turn gaits, which are already symmetric.
export const TROT = {
  ...FIRMWARE,
  forward: {
    start: [],
    cycle: [
      { R3: 135, L4: 135 },                 // lift pair R1 + L2
      { R1: 180, L2: 90, R2: 45, L1: 90 },  // swing it forward, pair R2 + L1 pushes
      { R3: 180, L4: 180 },                 // put it down
      { R4: 45, L3: 45 },                   // lift pair R2 + L1
      { R2: 90, L1: 0, R1: 90, L2: 135 },   // swing it forward, pair R1 + L2 pushes
      { R4: 0, L3: 0 },                     // put it down
    ],
  },
  backward: {
    start: [],
    cycle: [{ R3: 135, L4: 135 }, { R1: 90, L2: 135, R2: 90, L1: 0 }, { R3: 180, L4: 180 }, { R4: 45, L3: 45 }, { R2: 45, L1: 90, R1: 180, L2: 90 }, { R4: 0, L3: 0 }],
  },
};

const HIPS = { R1: "right", R2: "right", L1: "left", L2: "left" };
const MIN_STRIDE = 0.3;

// Centre of each hip's swing within one gait, so a shorter stride stays centred on the same leg position.
function hipCentres(gait) {
  const centres = {};
  for (const hip of Object.keys(HIPS)) {
    const angles = gait.cycle.map((frame) => frame[hip]).filter((a) => a !== undefined);
    if (angles.length) centres[hip] = (Math.min(...angles) + Math.max(...angles)) / 2;
  }
  return centres;
}

export class GaitEngine {
  // sendServos: (servos object) => void. Call onRobotState() with every state message from the robot.
  constructor(sendServos, options = {}) {
    this.sendServos = sendServos;
    this.options = { gait: "trot", trim: 0, frameDelay: 100, confirmTimeout: 300, subtrim: {}, ...options };
    this.command = "";
    this.steer = 0;
    this.servos = {};
    this.running = false;
    this.timings = [];  // per frame: how long the robot took to confirm the pose, for diagnosing pauses
  }

  configure(options) {
    Object.assign(this.options, options);
  }

  onRobotState(state) {
    if (state?.servos) this.servos = state.servos;
    this.wake?.();
  }

  // command: forward, backward, left, right or stop.
  // steer (walking only): -1 to 1, positive curves left by shortening the left stride.
  set(command, steer = 0) {
    this.command = command === "stop" ? "" : command;
    this.steer = steer;
    this.wake?.();
    if (this.command && !this.running) this.run();
  }

  // Stride factor for each side. trim > 0 shortens the right stride, which corrects a robot that veers left.
  strideScale() {
    const { trim } = this.options, clamp = (v) => Math.max(MIN_STRIDE, Math.min(1, v));
    return { left: clamp(1 - Math.max(0, -trim) - Math.max(0, this.steer) * 0.6), right: clamp(1 - Math.max(0, trim) - Math.max(0, -this.steer) * 0.6) };
  }

  pose(frame, centres, walking) {
    const scale = this.strideScale(), out = {};
    for (const [servo, angle] of Object.entries(frame)) {
      const side = HIPS[servo];
      const scaled = walking && side && centres[servo] !== undefined ? centres[servo] + (angle - centres[servo]) * scale[side] : angle;
      out[servo] = Math.round(Math.max(0, Math.min(180, scaled + (this.options.subtrim[servo] ?? 0))));
    }
    return out;
  }

  // Sends one frame and waits until the robot reports those servo angles, then for the frame delay.
  // The firmware applies only the newest pose it has received, so sending without waiting would drop frames.
  async sendFrame(target) {
    this.sendServos(target);
    const sentAt = Date.now(), deadline = sentAt + this.options.confirmTimeout;
    const reached = () => Object.entries(target).every(([servo, angle]) => this.servos[servo] === angle);
    while (!reached() && Date.now() < deadline) {
      await new Promise((resolve) => {
        this.wake = resolve;
        setTimeout(resolve, 50);
      });
    }
    this.timings.push({ at: sentAt, servos: Object.keys(target).length, confirmMs: Date.now() - sentAt, timedOut: !reached() });
    if (this.timings.length > 2000) this.timings.shift();
    await new Promise((resolve) => setTimeout(resolve, this.options.frameDelay));
  }

  async run() {
    this.running = true;
    while (this.command) {
      const command = this.command;
      const gait = (this.options.gait === "firmware" ? FIRMWARE : TROT)[command];
      if (!gait) break;
      const centres = hipCentres(gait), walking = command === "forward" || command === "backward";
      let first = true;
      while (this.command === command) {
        for (const frame of first ? [...gait.start, ...gait.cycle] : gait.cycle) {
          if (this.command !== command) break;
          await this.sendFrame(this.pose(frame, centres, walking));
        }
        first = false;
      }
      await this.sendFrame(this.pose(STAND, {}, false));  // like the firmware, stand between gaits
    }
    this.running = false;
  }
}
