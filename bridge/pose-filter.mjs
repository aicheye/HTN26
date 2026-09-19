// Extended Kalman filter for one tracked marker (the robot). Units: centimetres and radians, tracker floor frame.
//
// State: x, y, heading, and the marker's height z above the floor.
// Measurement: the marker centre in pixels plus the measured heading, with the camera pose of that frame.
//
// What the recordings of a robot standing still under a moving camera showed (recordings/rec-001, rec-002):
//  - The tracker's height from apparent marker size reads 1.5 to 5 cm, and is noisy (2 cm per frame). The frames
//    with the best camera pose hold the position stillest at a height of 11 cm. A wrong height shifts x, y
//    whenever the camera is not straight above, so the filter works from the pixel and a given marker height.
//  - Learning the height from parallax does not work on real data: subsets of one recording gave 3.5 to 12.5 cm,
//    because camera pose error is as large as the parallax. It stays available (markerHeight: null) but the height
//    should be measured with a ruler and passed in.
//  - Noise depends on how many floor markers fixed the camera pose: 0.04 cm of spread with four, 0.25 to 0.34 cm
//    with two or three, and a 0.6 cm offset between the two cases. The pixel noise settings below come from that.
// Prediction uses the gait the robot was told to run and the speeds measured by the navigator's calibration, so
// the pose keeps moving sensibly while the marker is hidden.
import { floorToPixel } from "../pi/client/floor.js";

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const zeros = (n, m) => Array.from({ length: n }, () => new Array(m).fill(0));
const mul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((sum, a, k) => sum + a * B[k][j], 0)));
const transpose = (A) => A[0].map((_, j) => A.map((row) => row[j]));
const add = (A, B) => A.map((row, i) => row.map((a, j) => a + B[i][j]));
function inverse3(m) {
  const [[a, b, c], [d, e, f], [g, h, i]] = m;
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  return [[e * i - f * h, c * h - b * i, b * f - c * e], [f * g - d * i, a * i - c * g, c * d - a * f], [d * h - e * g, b * g - a * h, a * e - b * d]]
    .map((row) => row.map((v) => v / det));
}

export const FILTER_DEFAULTS = {
  markerHeight: 11,         // cm above the floor. null = learn it (unreliable, see above)
  pixelSigmaByFloorMarkers: [0, 10, 5, 5, 1],  // px, indexed by how many floor markers fixed the camera pose
  headingSigma: 0.03,       // rad. The recordings show 0.75 to 1.0 degrees of spread on a robot standing still
  initialHeight: 6, initialHeightSigma: 4,
  heightDrift: 0.02,        // cm per sqrt(s): the height only changes when the robot changes pose
  idleDrift: 0.15,          // cm per sqrt(s) while no gait is running (someone may nudge the robot)
  movingDrift: 1.5,         // cm per sqrt(s) while walking: the gait speed is only roughly known
  idleTurnDrift: 0.02, movingTurnDrift: 0.25,  // rad per sqrt(s)
  gate: 16,                 // squared Mahalanobis distance above which a measurement is ignored
  relocateAfter: 12,        // consecutive ignored measurements before starting over (robot picked up and moved)
};

export class PoseFilter {
  constructor(motion = {}, options = {}) {
    this.motion = { walkSpeed: 0.04, turnRate: 0.5, veer: 0, ...motion };  // metres and radians, as the navigator stores them
    this.options = { ...FILTER_DEFAULTS, ...options };
    this.state = null;  // [x, y, heading, z]
    this.rejected = 0;
  }

  reset(measurement) {
    const o = this.options;
    const known = o.markerHeight !== null;
    this.state = [measurement.x, measurement.y, measurement.heading, known ? o.markerHeight : o.initialHeight];
    this.P = zeros(4, 4);
    [25, 25, 0.1, known ? 0 : o.initialHeightSigma ** 2].forEach((v, i) => { this.P[i][i] = v; });
    this.rejected = 0;
  }

  // command: the gait running during dt ("forward", "backward", "left", "right" or ""). mirrored: the tracker's floor
  // frame is left-handed seen from above (zUp false), so a physical left turn decreases the heading.
  predict(dt, command = "", mirrored = false) {
    if (!this.state || dt <= 0) return;
    const o = this.options, [x, y, heading, z] = this.state;
    const speed = (command === "forward" ? 1 : command === "backward" ? -1 : 0) * this.motion.walkSpeed * 100;
    const turn = ((command === "left" ? 1 : command === "right" ? -1 : 0) * this.motion.turnRate + this.motion.veer * Math.abs(speed) / 100) * (mirrored ? -1 : 1);
    this.state = [x + speed * Math.cos(heading) * dt, y + speed * Math.sin(heading) * dt, wrap(heading + turn * dt), z];
    const F = [[1, 0, -speed * Math.sin(heading) * dt, 0], [0, 1, speed * Math.cos(heading) * dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    const moving = command !== "", drift = moving ? o.movingDrift : o.idleDrift, turnDrift = moving ? o.movingTurnDrift : o.idleTurnDrift;
    const Q = zeros(4, 4);
    [drift ** 2 * dt, drift ** 2 * dt, turnDrift ** 2 * dt, o.markerHeight !== null ? 0 : o.heightDrift ** 2 * dt].forEach((v, i) => { Q[i][i] = v; });
    this.P = add(mul(mul(F, this.P), transpose(F)), Q);
  }

  // measurement: { px: [u, v], heading (rad), x, y (tracker's own estimate, used only to start), camera, floorMarkers }
  // Returns true when the measurement was used.
  update(measurement) {
    if (!this.state) { this.reset(measurement); return true; }
    const o = this.options, camera = measurement.camera;
    const project = ([x, y, , z]) => { const p = floorToPixel(camera, x, y, z); return [p.u, p.v]; };
    const predicted = project(this.state);
    const H = zeros(3, 4);
    for (const [column, step] of [[0, 0.05], [1, 0.05], [3, 0.05]]) {  // numeric derivative of the pixel in x, y, z
      const shifted = [...this.state];
      shifted[column] += step;
      const moved = project(shifted);
      H[0][column] = (moved[0] - predicted[0]) / step;
      H[1][column] = (moved[1] - predicted[1]) / step;
    }
    H[2][2] = 1;
    const innovation = [measurement.px[0] - predicted[0], measurement.px[1] - predicted[1], wrap(measurement.heading - this.state[2])];
    const sigma = o.pixelSigmaByFloorMarkers[Math.max(1, Math.min(4, measurement.floorMarkers))];
    const R = [[sigma ** 2, 0, 0], [0, sigma ** 2, 0], [0, 0, o.headingSigma ** 2]];
    const PHt = mul(this.P, transpose(H)), S = add(mul(H, PHt), R), Sinv = inverse3(S);
    const distance = innovation.reduce((sum, a, i) => sum + a * innovation.reduce((s, b, j) => s + Sinv[i][j] * b, 0), 0);
    if (distance > o.gate) {
      if (++this.rejected >= o.relocateAfter) this.reset(measurement);
      return false;
    }
    this.rejected = 0;
    const K = mul(PHt, Sinv);
    this.state = this.state.map((value, i) => value + K[i].reduce((sum, k, j) => sum + k * innovation[j], 0));
    this.state[2] = wrap(this.state[2]);
    const KH = mul(K, H);
    this.P = mul(zeros(4, 4).map((row, i) => row.map((_, j) => (i === j ? 1 : 0) - KH[i][j])), this.P);
    return true;
  }

  get pose() {
    if (!this.state) return null;
    const [x, y, heading, z] = this.state;
    return { x, y, heading, z, sigma: Math.sqrt(this.P[0][0] + this.P[1][1]) };
  }
}
