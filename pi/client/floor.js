// Converts between camera pixels and floor centimetres, using the `camera` object from the tracker's JSON.
// The camera model is: pixel = K * (R * floorPoint + tvec), with R = rodrigues(rvec).

function rodrigues([x, y, z]) {
  const angle = Math.hypot(x, y, z);
  if (angle < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const [kx, ky, kz] = [x / angle, y / angle, z / angle];
  const c = Math.cos(angle), s = Math.sin(angle), v = 1 - c;
  return [
    [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
    [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
    [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
  ];
}

// Floor position {x, y} in cm of the point seen at pixel (u, v), assuming it lies at height z cm. Floor level is z = 0.
export function pixelToFloor(camera, u, v, z = 0) {
  const R = rodrigues(camera.rvec), t = camera.tvec;
  const ray = [(u - camera.cx) / camera.f, (v - camera.cy) / camera.f, 1];
  // Rotate into floor axes with the transpose of R. The camera centre is -R^T * t.
  const rt = (p) => [0, 1, 2].map((i) => R[0][i] * p[0] + R[1][i] * p[1] + R[2][i] * p[2]);
  const dir = rt(ray), centre = rt(t).map((n) => -n);
  // The floor z axis points away from the camera when markers 1 to 4 run clockwise seen from above.
  const planeZ = centre[2] > 0 ? z : -z;
  const k = (planeZ - centre[2]) / dir[2];
  return { x: centre[0] + k * dir[0], y: centre[1] + k * dir[1] };
}

// Pixel {u, v} where the floor point (x, y) at height z cm appears in the full-size frame.
export function floorToPixel(camera, x, y, z = 0) {
  const R = rodrigues(camera.rvec), t = camera.tvec;
  const centreZ = -(R[0][2] * t[0] + R[1][2] * t[1] + R[2][2] * t[2]);
  const p = [x, y, centreZ > 0 ? z : -z];
  const c = [0, 1, 2].map((i) => R[i][0] * p[0] + R[i][1] * p[1] + R[i][2] * p[2] + t[i]);
  return { u: camera.cx + (camera.f * c[0]) / c[2], v: camera.cy + (camera.f * c[1]) / c[2] };
}
