export const distL2FromLngLat = (a, b) => {
  //Estimate differences of lat and lng degrees in meters.
  //https://en.wikipedia.org/wiki/Geographic_coordinate_system
  const latMid = ((a[1] + b[1]) / 2) * (Math.PI / 180);
  const mPerLat =
    111132.92 -
    559.82 * Math.cos(2 * latMid) +
    1.175 * Math.cos(4 * latMid) -
    0.0023 * Math.cos(6 * latMid);
  const mPerLng =
    111412.84 * Math.cos(latMid) -
    93.5 * Math.cos(3 * latMid) +
    0.118 * Math.cos(5 * latMid);
  const diffInMeters = [(b[0] - a[0]) * mPerLng, (b[1] - a[1]) * mPerLat];
  return Math.hypot(...diffInMeters);
};

export const mdistL2FromLngLat = (t0, t1) => {
  const mdist = [];
  for (const a of t0) {
    const row = [];
    for (const b of t1) {
      row.push(distL2FromLngLat(a, b));
    }
    mdist.push(row);
  }
  return mdist;
};

export const lngLatToXY = (lngLat) => {
  // Radius of the Earth in meters
  const R = 6378137;

  // Convert latitude and longitude from degrees to radians
  const longRad = (lngLat[0] * Math.PI) / 180;
  const latRad = (lngLat[1] * Math.PI) / 180;

  // Mercator projection formula
  const x = R * longRad;
  const y = R * Math.log(Math.tan(Math.PI / 4 + latRad / 2));

  return [x, y];
};

export const distanceCoordsToSegment = (p, a, b) => {
  const [px, py] = lngLatToXY(p);
  const [ax, ay] = lngLatToXY(a);
  const [bx, by] = lngLatToXY(b);
  const A = px - ax;
  const B = py - ay;
  const C = bx - ax;
  const D = by - ay;

  const dot = A * C + B * D;
  const len_sq = C * C + D * D;
  let param = -1;
  if (len_sq !== 0) {
    param = dot / len_sq;
  }

  let xx, yy;

  if (param < 0) {
    xx = ax;
    yy = ay;
  } else if (param > 0 && param < 1) {
    xx = ax + param * C;
    yy = ay + param * D;
  } else {
    xx = bx;
    yy = by;
  }

  const dx = px - xx;
  const dy = py - yy;
  return Math.sqrt(dx * dx + dy * dy);
};

/**
 * Returns the edit distance with real penalty (ERP) between two GNSS trajectories
 *
 * @param {array} t0 Trajectory (array) of [lng, lat] coordinates
 * @param {array} t1 Trajectory (array) of [lng, lat] coordinates
 * @param {array} g Reference [lng, lat] coordinate
 * @return {number} ERP distance
 *
 * @see {@link https://link.springer.com/article/10.1007/s00778-019-00574-9 | Trajectory distance article}
 * @see {@link https://github.com/bguillouet/traj-dist/tree/master | Python implementation}
 */
export const distErp = (t0, t1, g) => {
  const n0 = t0.length;
  const n1 = t1.length;
  const C = Array.from({ length: n0 + 1 }).map(() =>
    Array.from({ length: n1 + 1 }).fill(0)
  );

  const gt0Dist = t0.map((x) => distL2FromLngLat(g, x));
  const gt1Dist = t1.map((x) => distL2FromLngLat(g, x));
  const mdist = mdistL2FromLngLat(t0, t1);

  const sumgt0Dist = gt0Dist.reduce((acc, el) => acc + el, 0);
  const sumgt1Dist = gt1Dist.reduce((acc, el) => acc + el, 0);
  for (let i = 1; i < n0 + 1; i++) {
    C[i][0] = sumgt0Dist;
  }
  for (let j = 1; j < n1 + 1; j++) {
    C[0][j] = sumgt1Dist;
  }
  for (let i = 1; i < n0 + 1; ++i) {
    for (let j = 1; j < n1 + 1; ++j) {
      const derp0 = C[i - 1][j] + gt0Dist[i - 1];
      const derp1 = C[i][j - 1] + gt1Dist[j - 1];
      const derp01 = C[i - 1][j - 1] + mdist[i - 1][j - 1];
      C[i][j] = Math.min(derp0, derp1, derp01);
    }
  }
  const erp = C[n0][n1];
  return erp;
};

/**
 * Returns the dynamic time wrapping distance (DTW) between two GNSS trajectories
 *
 * @param {array} t0 Trajectory (array) of [lng, lat] coordinates
 * @param {array} t1 Trajectory (array) of [lng, lat] coordinates
 * @return {number} DTW distance
 *
 * @see {@link https://link.springer.com/article/10.1007/s00778-019-00574-9 | Trajectory distance article}
 * @see {@link https://github.com/bguillouet/traj-dist/tree/master | Python implementation}
 */
export const distDtw = (t0, t1) => {
  const n0 = t0.length;
  const n1 = t1.length;
  const C = Array.from({ length: n0 + 1 }).map(() =>
    Array.from({ length: n1 + 1 }).fill(0)
  );
  for (let i = 1; i < n0 + 1; i++) {
    C[i][0] = Infinity;
  }
  for (let j = 1; j < n1 + 1; j++) {
    C[0][j] = Infinity;
  }
  for (let i = 1; i < n0 + 1; ++i) {
    for (let j = 1; j < n1 + 1; ++j) {
      C[i][j] =
        distL2FromLngLat(t0[i - 1], t1[j - 1]) +
        Math.min(C[i][j - 1], C[i - 1][j - 1], C[i - 1][j]);
    }
  }
  const dtw = C[n0][n1];
  return dtw;
};

/**
 * Returns the trajectory tracking error (lateral path distance) between a source and a target GNSS trajectory
 *
 * @param {array} source Source trajectory (array) of [lng, lat] coordinates
 * @param {array} target Target trajectory (array) of [lng, lat] coordinates
 * @return {number} lateral tracking error
 *
 */
export const trajectoryTrackingError = (source, target) => {
  let totalError = 0;
  source.forEach((p) => {
    let minDistance = Infinity;
    for (let i = 0; i < target.length - 1; i++) {
      const distance = distanceCoordsToSegment(p, target[i], target[i + 1]);
      if (distance < minDistance) {
        minDistance = distance;
      }
    }
    totalError += minDistance;
  });
  return totalError;
};
