export const distL2 = (a, b) => {
  //Calculate 2D point distance in meters.
  const diff = [b[0] - a[0], b[1] - a[1]];
  return Math.hypot(...diff);
};

export const mdistL2 = (t0, t1) => {
  //Calculate 2D distance matrix between the trajectory points.
  const mdist = [];
  for (const a of t0) {
    const row = [];
    for (const b of t1) {
      row.push(distL2(a, b));
    }
    mdist.push(row);
  }
  return mdist;
};

export const diffInMetersFromLngLat = (a, b) => {
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
  return [(b[0] - a[0]) * mPerLng, (b[1] - a[1]) * mPerLat];
};

export const distL2FromLngLat = (a, b) => {
  //Estimate the distance between lng, lat coordinates
  const diffInMeters = diffInMetersFromLngLat(a, b);
  return Math.hypot(...diffInMeters);
};

export const mdistL2FromLngLat = (t0, t1) => {
  //Calculate 2D distance matrix between the trajectory lng,lat coordinates.
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

export const distPointToSegmentFromLngLat = (p, a, b) => {
  //Calculate the distance between a lng, lat coordinate and the line defined by other two coordinates
  //First, project the points to the euclidean space, taking "a" as reference
  const [px, py] = diffInMetersFromLngLat(a, p);
  const [bx, by] = diffInMetersFromLngLat(a, b);

  //Then, calculate the point to line distance in the euclidean space
  const dot = px * bx + py * by;
  const len_sq = bx * bx + by * by;
  let param = -1;
  if (len_sq !== 0) {
    param = dot / len_sq;
  }

  let xx, yy;

  if (param < 0) {
    xx = 0;
    yy = 0;
  } else if (param > 0 && param < 1) {
    xx = param * bx;
    yy = param * by;
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
 * Returns the minimum distance (smallest point-to-line distance) between a source and a target GNSS trajectory
 *
 * @param {array} source Source trajectory (array) of [lng, lat] coordinates
 * @param {array} target Target trajectory (array) of [lng, lat] coordinates
 * @return {number} minimum distance
 *
 */
export const minimumDistance = (source, target) => {
  let totalError = source.length > 0 ? 0 : Infinity;
  source.forEach((p) => {
    let minDistance = Infinity;
    for (let i = 0; i < target.length - 1; i++) {
      const distance = distPointToSegmentFromLngLat(
        p,
        target[i],
        target[i + 1]
      );
      if (distance < minDistance) {
        minDistance = distance;
      }
    }
    totalError += minDistance;
  });
  return totalError;
};
