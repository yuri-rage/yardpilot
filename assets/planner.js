// Coverage path planning — B-RV (Boustrophedon + Rapid Voronoi)
// Reference: Huang et al. (2021) doi:10.1007/s41315-021-00199-8

const R_EARTH = 6371000;

// ── §4.1 Equal Earth map projection (Bojan et al. 2019) ──────────────────────
// Adopted in the paper to convert GPS ↔ a 2-D metric plane.
// Coefficients A1–A4 as specified in the paper (§4.1, Eq. 4–6).

const EE_A1 = 1.340264;
const EE_A2 = -0.081106;
const EE_A3 = 0.000893;
const EE_A4 = 0.003796;

// Parametric latitude θ:  sin θ = (√3/2) sin φ   [Eq. 4]
function _eeTheta(phi) {
    return Math.asin((Math.sqrt(3) / 2) * Math.sin(phi));
}

// y(θ) = θ(A1 + A2 θ² + A3 θ⁶ + A4 θ⁸)   [Eq. 6]
function _eeY(th) {
    const t2 = th * th;
    const t6 = t2 * t2 * t2;
    return th * (EE_A1 + EE_A2 * t2 + EE_A3 * t6 + EE_A4 * t6 * t2);
}

// dy/dθ = A1 + 3A2 θ² + 7A3 θ⁶ + 9A4 θ⁸ — used in x-denominator [Eq. 5] and Newton inversion
function _eeDY(th) {
    const t2 = th * th;
    const t6 = t2 * t2 * t2;
    return EE_A1 + 3 * EE_A2 * t2 + 7 * EE_A3 * t6 + 9 * EE_A4 * t6 * t2;
}

// GPS → local metric [x, y] in metres, origin at (oLat, oLon).
//   x:  2√3 λ cos θ / (3 dy/dθ)   [Eq. 5]  × R_EARTH
//   y:  [_eeY(θ) − _eeY(θ₀)]      [Eq. 6]  × R_EARTH
function toLocal(lat, lon, oLat, oLon) {
    const phi = (lat * Math.PI) / 180;
    const lam = ((lon - oLon) * Math.PI) / 180;
    const th = _eeTheta(phi);
    const oTh = _eeTheta((oLat * Math.PI) / 180);
    const x =
        (R_EARTH * 2 * Math.sqrt(3) * lam * Math.cos(th)) / (3 * _eeDY(th));
    const y = R_EARTH * (_eeY(th) - _eeY(oTh));
    return [x, y];
}

// Local metric [x, y] → [lat, lon].  Newton's method inverts _eeY for θ.
function fromLocal(x, y, oLat, oLon) {
    const oTh = _eeTheta((oLat * Math.PI) / 180);
    const yN = y / R_EARTH + _eeY(oTh);
    let th = yN / EE_A1; // first-order initial guess
    for (let i = 0; i < 12; i++) th -= (_eeY(th) - yN) / _eeDY(th);
    const lat = (Math.asin((2 * Math.sin(th)) / Math.sqrt(3)) * 180) / Math.PI;
    const lam =
        (x * 3 * _eeDY(th)) / (R_EARTH * 2 * Math.sqrt(3) * Math.cos(th));
    return [lat, oLon + (lam * 180) / Math.PI];
}

// ── Convex hull (Andrew's monotone chain) ─────────────────

function cross2d(O, A, B) {
    return (A[0] - O[0]) * (B[1] - O[1]) - (A[1] - O[1]) * (B[0] - O[0]);
}

function convexHull(pts) {
    if (pts.length < 3) return pts.slice();
    const s = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const lo = [];
    const hi = [];
    for (const p of s) {
        while (lo.length >= 2 && cross2d(lo.at(-2), lo.at(-1), p) <= 0)
            lo.pop();
        lo.push(p);
    }
    for (let i = s.length - 1; i >= 0; i--) {
        const p = s[i];
        while (hi.length >= 2 && cross2d(hi.at(-2), hi.at(-1), p) <= 0)
            hi.pop();
        hi.push(p);
    }
    hi.pop();
    lo.pop();
    return lo.concat(hi);
}

// ── §4.2 Sweep direction via MBB (rotating calipers, Algorithm 1) ─────────────

function rotPts(pts, a) {
    const c = Math.cos(a);
    const s = Math.sin(a);
    return pts.map(([x, y]) => [x * c - y * s, x * s + y * c]);
}

function mbbSweepAngle(hull) {
    let minArea = Number.POSITIVE_INFINITY;
    let best = 0;
    for (let i = 0; i < hull.length; i++) {
        const [x1, y1] = hull[i];
        const [x2, y2] = hull[(i + 1) % hull.length];
        const a = Math.atan2(y2 - y1, x2 - x1);
        const r = rotPts(hull, -a);
        const xs = r.map((p) => p[0]);
        const ys = r.map((p) => p[1]);
        const w = Math.max(...xs) - Math.min(...xs);
        const h = Math.max(...ys) - Math.min(...ys);
        const area = w * h;
        if (area < minArea - 1e-6) {
            minArea = area;
            // Sweep parallel to the longer dimension of this bounding box:
            best = w >= h ? a : a + Math.PI / 2;
        }
    }
    while (best > Math.PI) best -= 2 * Math.PI;
    while (best <= -Math.PI) best += 2 * Math.PI;
    return best;
}

// ── Polygon / segment geometry ─────────────────────────────

function getSegIntersection(A, B, C, D) {
    const dxAB = B[0] - A[0];
    const dyAB = B[1] - A[1];
    const dxCD = D[0] - C[0];
    const dyCD = D[1] - C[1];

    const denom = dxAB * dyCD - dyAB * dxCD;
    if (Math.abs(denom) < 1e-9) return null;

    const t = ((C[0] - A[0]) * dyCD - (C[1] - A[1]) * dxCD) / denom;
    const u = ((C[0] - A[0]) * dyAB - (C[1] - A[1]) * dxAB) / denom;

    const EPS = 1e-7;
    if (t >= -EPS && t <= 1 + EPS && u >= -EPS && u <= 1 + EPS) {
        const tClamped = Math.max(0, Math.min(1, t));
        const uClamped = Math.max(0, Math.min(1, u));
        return {
            pt: [A[0] + tClamped * dxAB, A[1] + tClamped * dyAB],
            t: tClamped,
            u: uClamped,
        };
    }
    return null;
}

function getBBox(poly) {
    let minX = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    for (const [x, y] of poly) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
    }
    return { min: [minX, minY], max: [maxX, maxY] };
}

function crossesBoundary(a, b, poly, polyBBox = null) {
    const sMinX = Math.min(a[0], b[0]);
    const sMaxX = Math.max(a[0], b[0]);
    const sMinY = Math.min(a[1], b[1]);
    const sMaxY = Math.max(a[1], b[1]);

    if (polyBBox) {
        if (
            sMaxX < polyBBox.min[0] ||
            sMinX > polyBBox.max[0] ||
            sMaxY < polyBBox.min[1] ||
            sMinY > polyBBox.max[1]
        ) {
            return false;
        }
    }

    for (let i = 0; i < poly.length; i++) {
        const p1 = poly[i];
        const p2 = poly[(i + 1) % poly.length];

        // Edge bounding box check
        const eMinX = Math.min(p1[0], p2[0]);
        const eMaxX = Math.max(p1[0], p2[0]);
        const eMinY = Math.min(p1[1], p2[1]);
        const eMaxY = Math.max(p1[1], p2[1]);

        if (sMaxX < eMinX || sMinX > eMaxX || sMaxY < eMinY || sMinY > eMaxY) {
            continue;
        }

        const hit = getSegIntersection(a, b, p1, p2);
        if (hit && hit.t > 1e-4 && hit.t < 1 - 1e-4) return true;
    }
    return false;
}

function pointInPoly([px, py], poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i];
        const [xj, yj] = poly[j];
        if (
            yi > py !== yj > py &&
            px < ((xj - xi) * (py - yi)) / (yj - yi) + xi
        )
            inside = !inside;
    }
    return inside;
}

// Is segment [a,b] entirely in free space?  Checks boundary crossings + three
// interior sample points (catches concavities). exclC = inset exclusions for
// interior point tests (avoids boundary-vertex ambiguity).
function segmentFree(
    a,
    b,
    perim,
    excl,
    exclC,
    perimBBox = null,
    exclBBoxes = [],
    _exclCBBoxes = [],
) {
    if (crossesBoundary(a, b, perim, perimBBox)) return false;
    for (let i = 0; i < excl.length; i++) {
        if (crossesBoundary(a, b, excl[i], exclBBoxes[i])) return false;
    }
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const numSamples = Math.max(5, Math.ceil(L / 0.5));
    for (let k = 1; k < numSamples; k++) {
        const t = k / numSamples;
        const p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        if (!pointInPoly(p, perim)) return false;
        for (let i = 0; i < exclC.length; i++) {
            if (pointInPoly(p, exclC[i])) return false;
        }
    }
    return true;
}

function getSquareSegmentDistance(p, a, b) {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    if (dx === 0 && dy === 0) {
        return (p[0] - a[0]) * (p[0] - a[0]) + (p[1] - a[1]) * (p[1] - a[1]);
    }
    let t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy);
    t = Math.max(0, Math.min(1, t));
    const prx = a[0] + t * dx;
    const pry = a[1] + t * dy;
    return (p[0] - prx) * (p[0] - prx) + (p[1] - pry) * (p[1] - pry);
}

function simplifyPath(path, tolerance) {
    if (path.length <= 2) return path.slice();
    const result = [path[0]];
    let lastKeptIdx = 0;

    for (let i = 1; i < path.length; i++) {
        if (i === path.length - 1) {
            result.push(path[i]);
            break;
        }

        let ok = true;
        const segmentStart = path[lastKeptIdx];
        const segmentEnd = path[i + 1];

        for (let j = lastKeptIdx + 1; j <= i; j++) {
            const dSq = getSquareSegmentDistance(
                path[j],
                segmentStart,
                segmentEnd,
            );
            if (dSq > tolerance * tolerance) {
                ok = false;
                break;
            }
        }

        if (!ok) {
            result.push(path[i]);
            lastKeptIdx = i;
        }
    }
    return result;
}

function simplifyGreedy(points, tolerance) {
    if (points.length <= 2) return points.slice();

    if (tolerance > 0.05) {
        const result = [];
        let i = 0;
        const n = points.length;
        while (i < n) {
            if (i === n - 1) {
                const p1 = points[i];
                const p2 = points[0];
                const d = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                if (d < tolerance) {
                    const avg = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2];
                    result.push(avg);
                    i += 2;
                } else {
                    result.push(p1);
                    i++;
                }
            } else {
                const p1 = points[i];
                const p2 = points[i + 1];
                const d = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                if (d < tolerance) {
                    const avg = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2];
                    result.push(avg);
                    i += 2;
                } else {
                    result.push(p1);
                    i++;
                }
            }
        }
        if (result.length >= 3) {
            return result;
        }
    }

    let currentTolerance = tolerance;
    for (let attempt = 0; attempt < 10; attempt++) {
        // 1. Pre-collapse points that are extremely close (within 20% of tolerance)
        const epsilon = currentTolerance * 0.2;
        const collapsed = [points[0]];
        for (let i = 1; i < points.length; i++) {
            const prev = collapsed[collapsed.length - 1];
            const curr = points[i];
            if (Math.hypot(curr[0] - prev[0], curr[1] - prev[1]) > epsilon) {
                collapsed.push(curr);
            }
        }
        if (collapsed.length > 2) {
            const first = collapsed[0];
            const last = collapsed[collapsed.length - 1];
            if (Math.hypot(last[0] - first[0], last[1] - first[1]) <= epsilon) {
                collapsed.pop();
            }
        }

        if (collapsed.length < 3) {
            currentTolerance /= 2;
            continue;
        }

        // 2. Find the two furthest vertices to split the closed polygon
        let maxDistSq = 0;
        let idx1 = 0;
        let idx2 = 1;
        const n = collapsed.length;
        for (let i = 0; i < n; i++) {
            for (let j = i + 1; j < n; j++) {
                const dx = collapsed[i][0] - collapsed[j][0];
                const dy = collapsed[i][1] - collapsed[j][1];
                const distSq = dx * dx + dy * dy;
                if (distSq > maxDistSq) {
                    maxDistSq = distSq;
                    idx1 = i;
                    idx2 = j;
                }
            }
        }

        // Ensure idx1 < idx2
        if (idx1 > idx2) {
            const tmp = idx1;
            idx1 = idx2;
            idx2 = tmp;
        }

        // Path 1: from idx1 to idx2
        const path1 = collapsed.slice(idx1, idx2 + 1);
        // Path 2: from idx2 to idx1 (wrapping around)
        const path2 = collapsed
            .slice(idx2)
            .concat(collapsed.slice(0, idx1 + 1));

        // Simplify each path
        const sim1 = simplifyPath(path1, currentTolerance);
        const sim2 = simplifyPath(path2, currentTolerance);

        // Join simplified paths back into a closed polygon
        const joined = sim1.slice(0, -1).concat(sim2.slice(0, -1));

        if (joined.length >= 3) {
            return joined;
        }
        currentTolerance /= 2;
    }
    return points.slice();
}

// ── Zone geometry helpers ──────────────────────────────────

function isPolygonCW(poly) {
    let area = 0;
    for (let i = 0; i < poly.length; i++) {
        const p1 = poly[i];
        const p2 = poly[(i + 1) % poly.length];
        area += p1[0] * p2[1] - p2[0] * p1[1];
    }
    return area < 0;
}

function polygonWindingNumber([px, py], poly) {
    let wn = 0;
    const n = poly.length;
    for (let i = 0; i < n; i++) {
        const [x1, y1] = poly[i];
        const [x2, y2] = poly[(i + 1) % n];
        if (y1 <= py) {
            if (y2 > py && (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) > 0) {
                wn++;
            }
        } else if (
            y2 <= py &&
            (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < 0
        ) {
            wn--;
        }
    }
    return wn;
}

function insetPolyFallback(poly, dist) {
    if (poly.length < 3) return poly.slice();
    if (dist <= 0) return poly.slice();

    const N = poly.length;

    // Check orientation to know which way is inward
    const cw = isPolygonCW(poly);
    const d = dist * (cw ? -1 : 1);

    const result = [];
    for (let i = 0; i < N; i++) {
        const A = poly[(i - 1 + N) % N];
        const B = poly[i];
        const C = poly[(i + 1) % N];

        // Edge vectors
        const dx1 = B[0] - A[0];
        const dy1 = B[1] - A[1];
        const len1 = Math.hypot(dx1, dy1);
        const dx2 = C[0] - B[0];
        const dy2 = C[1] - B[1];
        const len2 = Math.hypot(dx2, dy2);

        if (len1 < 1e-9 || len2 < 1e-9) {
            result.push([B[0], B[1]]);
            continue;
        }

        // Unit directions
        const v1 = [dx1 / len1, dy1 / len1];
        const v2 = [dx2 / len2, dy2 / len2];

        // Left normals
        const n1 = [-v1[1], v1[0]];
        const n2 = [-v2[1], v2[0]];

        // Bisector
        const nb = [n1[0] + n2[0], n1[1] + n2[1]];
        const lenB = Math.hypot(nb[0], nb[1]);
        const m = lenB < 1e-9 ? n1 : [nb[0] / lenB, nb[1] / lenB];

        const cosHalfAngle = n1[0] * m[0] + n1[1] * m[1];

        // Cap miter length to prevent extreme spikes at sharp corners while allowing
        // proper insetting along long edges (e.g. sharp acute wedges)
        const maxMiter = Math.max(Math.abs(d) * 4, Math.min(len1, len2) * 0.85);
        let L = d;
        if (Math.abs(cosHalfAngle) > 1e-4) {
            L = d / cosHalfAngle;
            if (Math.abs(L) > maxMiter) {
                L = Math.sign(L) * maxMiter;
            }
        } else {
            L = Math.sign(d || 1) * maxMiter;
        }

        result.push([B[0] + m[0] * L, B[1] + m[1] * L]);
    }

    // Safety check to prevent self-intersections (inversion past centroid, checked per vertex)
    const cx = poly.reduce((s, p) => s + p[0], 0) / N;
    const cy = poly.reduce((s, p) => s + p[1], 0) / N;

    const finalResult = [];
    for (let i = 0; i < N; i++) {
        const orig = poly[i];
        const offset = result[i];
        const shiftLen = Math.hypot(offset[0] - orig[0], offset[1] - orig[1]);
        const distToCentroid = Math.hypot(cx - orig[0], cy - orig[1]);
        const maxAllowed = distToCentroid * 0.9;

        if (shiftLen > maxAllowed && shiftLen > 1e-9) {
            const scale = maxAllowed / shiftLen;
            finalResult.push([
                orig[0] + (offset[0] - orig[0]) * scale,
                orig[1] + (offset[1] - orig[1]) * scale,
            ]);
        } else {
            finalResult.push(offset);
        }
    }

    return finalResult;
}

function insetPolyMulti(poly, dist) {
    if (poly.length < 3) return [poly.slice()];
    if (dist <= 0) return [poly.slice()];

    if (typeof ClipperLib !== "undefined") {
        try {
            const scale = 1000000;
            const co = new ClipperLib.ClipperOffset();
            const path = poly.map(([x, y]) => ({
                X: Math.round(x * scale),
                Y: Math.round(y * scale),
            }));
            co.AddPath(
                path,
                ClipperLib.JoinType.jtMiter,
                ClipperLib.EndType.etClosedPolygon,
            );
            const solution = new ClipperLib.Paths();
            co.Execute(solution, -dist * scale);
            if (solution.length > 0) {
                const polys = [];
                for (let i = 0; i < solution.length; i++) {
                    const area = Math.abs(ClipperLib.Clipper.Area(solution[i]));
                    if (area / (scale * scale) >= 0.05) {
                        polys.push(
                            solution[i].map((pt) => [
                                pt.X / scale,
                                pt.Y / scale,
                            ]),
                        );
                    }
                }
                if (polys.length > 0) return polys;
            }
            return [];
        } catch (e) {
            console.warn(
                "Clipper inset failed, falling back to miter-offset:",
                e,
            );
        }
    }

    const single = insetPolyFallback(poly, dist);
    return single.length >= 3 ? [single] : [];
}

function insetPoly(poly, dist) {
    const polys = insetPolyMulti(poly, dist);
    if (!polys.length) return [];
    let best = polys[0];
    let maxArea = -1;
    for (const p of polys) {
        const a = getPolygonArea(p);
        if (a > maxArea) {
            maxArea = a;
            best = p;
        }
    }
    return best;
}

function differencePolyMulti(subjectPolys, clipPolys) {
    if (
        typeof ClipperLib === "undefined" ||
        !subjectPolys.length ||
        !clipPolys.length
    ) {
        return subjectPolys;
    }
    try {
        const scale = 1000000;
        const clipper = new ClipperLib.Clipper();
        for (const p of subjectPolys) {
            clipper.AddPath(
                p.map(([x, y]) => ({
                    X: Math.round(x * scale),
                    Y: Math.round(y * scale),
                })),
                ClipperLib.PolyType.ptSubject,
                true,
            );
        }
        for (const p of clipPolys) {
            clipper.AddPath(
                p.map(([x, y]) => ({
                    X: Math.round(x * scale),
                    Y: Math.round(y * scale),
                })),
                ClipperLib.PolyType.ptClip,
                true,
            );
        }
        const solution = new ClipperLib.Paths();
        clipper.Execute(
            ClipperLib.ClipType.ctDifference,
            solution,
            ClipperLib.PolyFillType.pftNonZero,
            ClipperLib.PolyFillType.pftNonZero,
        );
        const result = [];
        for (const path of solution) {
            if (ClipperLib.Clipper.Orientation(path)) {
                result.push(path.map((pt) => [pt.X / scale, pt.Y / scale]));
            }
        }
        return result;
    } catch (_e) {
        return subjectPolys;
    }
}

function unionPolygons(polys) {
    if (!polys || polys.length === 0) return [];
    if (polys.length === 1) return [polys[0]];
    if (typeof ClipperLib !== "undefined") {
        try {
            const scale = 1000000;
            const clipper = new ClipperLib.Clipper();
            for (const p of polys) {
                clipper.AddPath(
                    p.map(([x, y]) => ({
                        X: Math.round(x * scale),
                        Y: Math.round(y * scale),
                    })),
                    ClipperLib.PolyType.ptSubject,
                    true,
                );
            }
            const solution = new ClipperLib.Paths();
            clipper.Execute(
                ClipperLib.ClipType.ctUnion,
                solution,
                ClipperLib.PolyFillType.pftNonZero,
                ClipperLib.PolyFillType.pftNonZero,
            );
            const result = [];
            for (const path of solution) {
                if (ClipperLib.Clipper.Orientation(path)) {
                    result.push(path.map((pt) => [pt.X / scale, pt.Y / scale]));
                }
            }
            if (result.length > 0) return result;
        } catch (_e) {}
    }
    const groups = [];
    const assigned = new Array(polys.length).fill(false);
    for (let i = 0; i < polys.length; i++) {
        if (assigned[i]) continue;
        const group = [polys[i]];
        assigned[i] = true;
        let added = true;
        while (added) {
            added = false;
            for (let j = 0; j < polys.length; j++) {
                if (assigned[j]) continue;
                const overlaps = group.some((g) => {
                    if (g.some((pt) => pointInPoly(pt, polys[j]))) return true;
                    if (polys[j].some((pt) => pointInPoly(pt, g))) return true;
                    const gLen = g.length;
                    const pjLen = polys[j].length;
                    for (let gi = 0; gi < gLen; gi++) {
                        const g1 = g[gi];
                        const g2 = g[(gi + 1) % gLen];
                        for (let pj = 0; pj < pjLen; pj++) {
                            const p1 = polys[j][pj];
                            const p2 = polys[j][(pj + 1) % pjLen];
                            if (getSegIntersection(g1, g2, p1, p2)) return true;
                        }
                    }
                    return false;
                });
                if (overlaps) {
                    group.push(polys[j]);
                    assigned[j] = true;
                    added = true;
                }
            }
        }
        if (group.length === 1) {
            groups.push(group[0]);
        } else {
            groups.push(convexHull(group.flat()));
        }
    }
    return groups;
}

function makeCW(poly) {
    return isPolygonCW(poly) ? poly.slice() : poly.slice().reverse();
}

function subtractPolygonAllPieces(P_orig, E_orig) {
    if (typeof ClipperLib !== "undefined") {
        try {
            const scale = 1000000;
            const clipper = new ClipperLib.Clipper();

            const pathP = P_orig.map(([x, y]) => ({
                X: Math.round(x * scale),
                Y: Math.round(y * scale),
            }));
            const pathE = E_orig.map(([x, y]) => ({
                X: Math.round(x * scale),
                Y: Math.round(y * scale),
            }));

            clipper.AddPath(pathP, ClipperLib.PolyType.ptSubject, true);
            clipper.AddPath(pathE, ClipperLib.PolyType.ptClip, true);

            const solution = new ClipperLib.Paths();
            clipper.Execute(
                ClipperLib.ClipType.ctDifference,
                solution,
                ClipperLib.PolyFillType.pftNonZero,
                ClipperLib.PolyFillType.pftNonZero,
            );

            const pieces = [];
            for (let i = 0; i < solution.length; i++) {
                if (!ClipperLib.Clipper.Orientation(solution[i])) continue;
                const area =
                    Math.abs(ClipperLib.Clipper.Area(solution[i])) /
                    (scale * scale);
                if (area >= 0.1) {
                    pieces.push(
                        solution[i].map((pt) => [pt.X / scale, pt.Y / scale]),
                    );
                }
            }
            if (pieces.length > 0) {
                pieces.sort(
                    (a, b) =>
                        Math.abs(getPolygonArea(b)) -
                        Math.abs(getPolygonArea(a)),
                );
                return pieces;
            }
            return [];
        } catch (e) {
            console.warn(
                "Clipper subtraction failed, falling back to Weiler-Atherton:",
                e,
            );
        }
    }

    const P = makeCW(P_orig);
    const E = makeCW(E_orig);

    const N = P.length;
    const M = E.length;

    const listP = [];
    for (let i = 0; i < N; i++) {
        listP.push({
            pt: P[i],
            isIntersection: false,
            idx: i,
            intersections: [],
        });
    }

    const listE = [];
    for (let i = 0; i < M; i++) {
        listE.push({
            pt: E[i],
            isIntersection: false,
            idx: i,
            intersections: [],
        });
    }

    // Find all intersections
    for (let i = 0; i < N; i++) {
        const A = P[i];
        const B = P[(i + 1) % N];
        for (let j = 0; j < M; j++) {
            const C = E[j];
            const D = E[(j + 1) % M];

            const inter = getSegIntersection(A, B, C, D);
            if (inter) {
                const nodeP = {
                    pt: inter.pt,
                    isIntersection: true,
                    t: inter.t,
                    partner: null,
                };
                const nodeE = {
                    pt: inter.pt,
                    isIntersection: true,
                    t: inter.u,
                    partner: null,
                };
                nodeP.partner = nodeE;
                nodeE.partner = nodeP;

                listP[i].intersections.push(nodeP);
                listE[j].intersections.push(nodeE);
            }
        }
    }

    // If no intersections, check containment
    let anyInter = false;
    for (let i = 0; i < N; i++) {
        if (listP[i].intersections.length > 0) anyInter = true;
    }

    if (!anyInter) {
        const _E_in_P = pointInPoly(E[0], P);
        const P_in_E = pointInPoly(P[0], E);
        if (P_in_E) return []; // P is completely inside E
        return [P_orig]; // E is completely inside P or disjoint
    }

    // Flatten lists
    const flatP = [];
    for (let i = 0; i < N; i++) {
        flatP.push(listP[i]);
        listP[i].intersections.sort((a, b) => a.t - b.t);
        for (const inter of listP[i].intersections) {
            flatP.push(inter);
        }
    }

    const flatE = [];
    for (let j = 0; j < M; j++) {
        flatE.push(listE[j]);
        listE[j].intersections.sort((a, b) => a.t - b.t);
        for (const inter of listE[j].intersections) {
            flatE.push(inter);
        }
    }

    // Assign flat indices
    for (let i = 0; i < flatP.length; i++) flatP[i].flatIdx = i;
    for (let i = 0; i < flatE.length; i++) flatE[i].flatIdx = i;

    for (let i = 0; i < flatP.length; i++) {
        const node = flatP[i];
        if (node.isIntersection) {
            const nextNode = flatP[(i + 1) % flatP.length];
            const mid = [
                0.5 * (node.pt[0] + nextNode.pt[0]),
                0.5 * (node.pt[1] + nextNode.pt[1]),
            ];
            node.isEntering = pointInPoly(mid, E);
            node.partner.isEntering = node.isEntering;
        }
    }

    const enteringNodes = flatP.filter((n) => n.isIntersection && n.isEntering);
    const pieces = [];
    const visitedEntering = new Set();

    for (const startNode of enteringNodes) {
        if (visitedEntering.has(startNode)) continue;
        const resultPts = [];
        let currList = flatP;
        let currIdx = startNode.flatIdx;
        let dir = 1;
        let loopCount = 0;
        const maxLoops = flatP.length + flatE.length + 100;

        while (loopCount < maxLoops) {
            const node = currList[currIdx];
            resultPts.push(node.pt);
            if (node.isIntersection) {
                if (currList === flatP) {
                    if (node.isEntering) visitedEntering.add(node);
                    currList = flatE;
                    currIdx = node.partner.flatIdx;
                    dir = -1; // Walk E CCW
                } else {
                    currList = flatP;
                    currIdx = node.partner.flatIdx;
                    dir = 1; // Walk P CW
                }
            }
            currIdx = (currIdx + dir + currList.length) % currList.length;
            if (currList === flatP && currIdx === startNode.flatIdx) break;
            loopCount++;
        }

        const cleaned = [];
        for (const pt of resultPts) {
            if (
                cleaned.length === 0 ||
                Math.hypot(
                    pt[0] - cleaned.at(-1)[0],
                    pt[1] - cleaned.at(-1)[1],
                ) > 1e-5
            ) {
                cleaned.push(pt);
            }
        }
        if (
            cleaned.length > 1 &&
            Math.hypot(
                cleaned[0][0] - cleaned.at(-1)[0],
                cleaned[0][1] - cleaned.at(-1)[1],
            ) < 1e-5
        ) {
            cleaned.pop();
        }
        const area = Math.abs(getPolygonArea(cleaned));
        if (cleaned.length >= 3 && area >= 0.1) {
            pieces.push(makeCW(cleaned));
        }
    }

    if (pieces.length > 0) {
        pieces.sort(
            (a, b) => Math.abs(getPolygonArea(b)) - Math.abs(getPolygonArea(a)),
        );
        return pieces;
    }
    return [];
}

function subtractPolygonsMulti(pieces, ex) {
    const nextPieces = [];
    for (const piece of pieces) {
        const res = subtractPolygonAllPieces(piece, ex);
        if (res.length > 0) {
            for (const r of res) {
                if (r.length >= 3 && Math.abs(getPolygonArea(r)) >= 0.1) {
                    nextPieces.push(r);
                }
            }
        }
    }
    return nextPieces;
}

function expandPoly(poly, dist) {
    if (poly.length < 3) return poly.slice();

    if (typeof ClipperLib !== "undefined") {
        try {
            const scale = 1000000;
            const co = new ClipperLib.ClipperOffset();
            const path = poly.map(([x, y]) => ({
                X: Math.round(x * scale),
                Y: Math.round(y * scale),
            }));
            co.AddPath(
                path,
                ClipperLib.JoinType.jtMiter,
                ClipperLib.EndType.etClosedPolygon,
            );
            const solution = new ClipperLib.Paths();
            co.Execute(solution, dist * scale);
            if (solution.length > 0) {
                let bestIdx = 0;
                let maxArea = -1;
                for (let i = 0; i < solution.length; i++) {
                    const area = Math.abs(ClipperLib.Clipper.Area(solution[i]));
                    if (area > maxArea) {
                        maxArea = area;
                        bestIdx = i;
                    }
                }
                return solution[bestIdx].map((pt) => [
                    pt.X / scale,
                    pt.Y / scale,
                ]);
            }
        } catch (e) {
            console.warn(
                "Clipper offset failed, falling back to miter-offset:",
                e,
            );
        }
    }

    // Fallback: Edge-normal expansion (miter offset outward)
    const N = poly.length;
    const cw = isPolygonCW(poly);
    // Outward direction is opposite of inward
    const d = dist * (cw ? 1 : -1);

    const result = [];
    for (let i = 0; i < N; i++) {
        const A = poly[(i - 1 + N) % N];
        const B = poly[i];
        const C = poly[(i + 1) % N];

        const dx1 = B[0] - A[0];
        const dy1 = B[1] - A[1];
        const len1 = Math.hypot(dx1, dy1);
        const dx2 = C[0] - B[0];
        const dy2 = C[1] - B[1];
        const len2 = Math.hypot(dx2, dy2);

        if (len1 < 1e-9 || len2 < 1e-9) {
            result.push([B[0], B[1]]);
            continue;
        }

        const v1 = [dx1 / len1, dy1 / len1];
        const v2 = [dx2 / len2, dy2 / len2];

        const n1 = [-v1[1], v1[0]];
        const n2 = [-v2[1], v2[0]];

        const nb = [n1[0] + n2[0], n1[1] + n2[1]];
        const lenB = Math.hypot(nb[0], nb[1]);
        const m = lenB < 1e-9 ? n1 : [nb[0] / lenB, nb[1] / lenB];

        const cosHalfAngle = n1[0] * m[0] + n1[1] * m[1];

        let L = d;
        if (Math.abs(cosHalfAngle) > 0.1) {
            L = d / cosHalfAngle;
            if (Math.abs(L) > Math.abs(d) * 4) {
                L = Math.sign(L) * Math.abs(d) * 4;
            }
        } else {
            L = d * 4;
        }

        result.push([B[0] + m[0] * L, B[1] + m[1] * L]);
    }
    return result;
}

function circlePoly(cx, cy, r, n = 64) {
    return Array.from({ length: n }, (_, i) => {
        const a = (2 * Math.PI * i) / n;
        return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
    });
}

// ── Scan-line helpers ────────────────────────────────────
// X-coordinates where the horizontal line y=sy crosses polygon edges.
function scanXs(poly, sy) {
    const xs = [];
    for (let i = 0; i < poly.length; i++) {
        const [x1, y1] = poly[i];
        const [x2, y2] = poly[(i + 1) % poly.length];
        if (y1 <= sy !== y2 <= sy)
            xs.push(x1 + ((sy - y1) / (y2 - y1)) * (x2 - x1));
    }
    return xs.sort((a, b) => a - b);
}

// Remove from [lo,hi] the intervals covered by [exXs[0],exXs[1]], [exXs[2],exXs[3]], …
function subtractIntervals(lo, hi, exXs) {
    let segs = [[lo, hi]];
    for (let i = 0; i + 1 < exXs.length; i += 2) {
        const [eL, eR] = [exXs[i], exXs[i + 1]];
        const next = [];
        for (const [sl, sr] of segs) {
            if (eR <= sl || eL >= sr) {
                next.push([sl, sr]);
                continue;
            }
            if (eL > sl) next.push([sl, eL]);
            if (eR < sr) next.push([eR, sr]);
        }
        segs = next;
    }
    return segs;
}

// ── §4.3 Grid-based coverage map  (Eq. 7) ─────────────────
// Cell states:  0 = unvisited,  0.5 = visited,  1 = unvisitable

function buildGrid(perimR, _exR, cellSize) {
    const polys =
        perimR.length > 0 && Array.isArray(perimR[0][0]) ? perimR : [perimR];
    const allPts = polys.flat();
    const xs = allPts.map((p) => p[0]);
    const ys = allPts.map((p) => p[1]);
    const xMin = Math.min(...xs);
    const yMin = Math.min(...ys);
    const cols = Math.max(1, Math.ceil((Math.max(...xs) - xMin) / cellSize));
    const rows = Math.max(1, Math.ceil((Math.max(...ys) - yMin) / cellSize));
    const grid = Array.from({ length: rows }, () => new Uint8Array(cols));
    return { grid, xMin, yMin, rows, cols };
}

// Strip endpoint nudge constants — keep endpoints off exact polygon boundaries.
//   EPS_PERIM : inset from the perimeter so midpoints of perimeter-edge transitions
//               are clearly inside the polygon (avoids pointInPoly boundary ambiguity).
//   EPS_EXCL  : expand each exclusion interval to account for the chord error of the
//               n=64 circle polygon (~0.7 cm for r=5.5 m) plus a safety margin, so
//               strip endpoints are never on or inside the exclusion boundary.
const EPS_PERIM = 0.001; // 1 mm inset from perimeter edge
const EPS_EXCL = 0.02; // 2 cm expansion of every exclusion interval

// Boustrophedon sweep: start at corner of longest edge (§4.3, Eq. 8).
// Strip endpoints are computed by exact scan-line intersection with the
// perimeter and exclusion boundaries, so no lane ever overshoots into
// invalid territory.  The grid is updated 0→0.5 for each traversed cell
// to maintain the paper's cell-state tracking (Eq. 7).
function generateStrips(
    { grid, xMin, yMin, rows, cols },
    cellSize,
    perimR,
    exR,
    lawnPolyR = null,
    fillPolysR = null,
    expandedPolysR = null,
) {
    const polys =
        perimR.length > 0 && Array.isArray(perimR[0][0]) ? perimR : [perimR];
    const strips = [];
    let ltr = true; // start left-to-right from the corner of the longest edge

    for (let r = 0; r < rows; r++) {
        const cy = yMin + (r + 0.5) * cellSize;
        let segs = [];

        // Exact perimeter intersections at this y-level across all lobes; inset EPS_PERIM
        for (const poly of polys) {
            const pxs = scanXs(poly, cy);
            if (pxs.length < 2) continue;
            const cw = isPolygonCW(poly);
            for (let i = 0; i + 1 < pxs.length; i += 2) {
                const mid = [(pxs[i] + pxs[i + 1]) / 2, cy];
                const wn = polygonWindingNumber(mid, poly);
                if (cw ? wn < 0 : wn > 0) {
                    segs.push([pxs[i] + EPS_PERIM, pxs[i + 1] - EPS_PERIM]);
                }
            }
        }

        // Offline narrow-area fill: take fill area's intervals on this row and subtract expanded main strip intervals
        if (fillPolysR && fillPolysR.length > 0) {
            let fillSegs = [];
            for (const poly of fillPolysR) {
                const pxs = scanXs(poly, cy);
                if (pxs.length < 2) continue;
                for (let i = 0; i + 1 < pxs.length; i += 2) {
                    fillSegs.push({
                        lo: pxs[i] + EPS_PERIM,
                        hi: pxs[i + 1] - EPS_PERIM,
                        poly,
                    });
                }
            }
            if (fillSegs.length > 0) {
                if (expandedPolysR && expandedPolysR.length > 0) {
                    const expSegs = [];
                    for (const poly of expandedPolysR) {
                        const pxs = scanXs(poly, cy);
                        if (pxs.length < 2) continue;
                        for (let i = 0; i + 1 < pxs.length; i += 2) {
                            expSegs.push([pxs[i], pxs[i + 1]]);
                        }
                    }
                    expSegs.sort((a, b) => a[0] - b[0]);
                    const mergedExpXs = [];
                    for (const [lo, hi] of expSegs) {
                        if (!mergedExpXs.length) {
                            mergedExpXs.push(lo, hi);
                        } else {
                            const lastHi = mergedExpXs[mergedExpXs.length - 1];
                            if (lo <= lastHi) {
                                mergedExpXs[mergedExpXs.length - 1] = Math.max(
                                    lastHi,
                                    hi,
                                );
                            } else {
                                mergedExpXs.push(lo, hi);
                            }
                        }
                    }
                    if (mergedExpXs.length >= 2) {
                        fillSegs = fillSegs.flatMap(({ lo, hi, poly }) =>
                            subtractIntervals(lo, hi, mergedExpXs).map(
                                ([sL, sR]) => ({ lo: sL, hi: sR, poly }),
                            ),
                        );
                    }
                }
                for (const { lo: fLo, hi: fHi, poly } of fillSegs) {
                    if (fHi - fLo >= cellSize * 0.5) {
                        const mid = [(fLo + fHi) / 2, cy];
                        const cw = isPolygonCW(poly);
                        const wn = polygonWindingNumber(mid, poly);
                        if (cw ? wn < 0 : wn > 0) {
                            segs.push([fLo, fHi]);
                        }
                    }
                }
            }
        }

        if (!segs.length) continue;

        segs.sort((a, b) => a[0] - b[0]);

        const mergedSegs = [];
        for (const [lo, hi] of segs) {
            if (!mergedSegs.length) {
                mergedSegs.push([lo, hi]);
            } else {
                const last = mergedSegs[mergedSegs.length - 1];
                if (lo <= last[1]) {
                    last[1] = Math.max(last[1], hi);
                } else {
                    mergedSegs.push([lo, hi]);
                }
            }
        }
        segs = mergedSegs;

        if (lawnPolyR) {
            const yardXs = scanXs(lawnPolyR, cy);
            if (yardXs.length >= 2) {
                const clippedSegs = [];
                for (const [lo, hi] of segs) {
                    for (let k = 0; k + 1 < yardXs.length; k += 2) {
                        const yLo = yardXs[k] + EPS_PERIM;
                        const yHi = yardXs[k + 1] - EPS_PERIM;
                        const intLo = Math.max(lo, yLo);
                        const intHi = Math.min(hi, yHi);
                        if (intHi - intLo > cellSize * 0.1) {
                            clippedSegs.push([intLo, intHi]);
                        }
                    }
                }
                segs = clippedSegs;
            }
        }

        for (const ex of exR) {
            const exXs = scanXs(ex, cy);
            if (exXs.length >= 2) {
                // Expand the exclusion interval so strip endpoints land clearly
                // outside the zone (accounts for polygon chord approximation error).
                const exXsExp = exXs.map((x, k) =>
                    k % 2 === 0 ? x - EPS_EXCL : x + EPS_EXCL,
                );
                segs = segs.flatMap(([lo, hi]) =>
                    subtractIntervals(lo, hi, exXsExp),
                );
            }
        }
        segs = segs.filter(([lo, hi]) => hi - lo > cellSize * 0.1);
        if (!segs.length) continue;

        // Mark corresponding grid cells visited (0 → 0.5) for state tracking
        for (const [lo, hi] of segs) {
            const c0 = Math.max(0, Math.floor((lo - xMin) / cellSize));
            const c1 = Math.min(
                cols - 1,
                Math.floor((hi - xMin - 1e-9) / cellSize),
            );
            for (let c = c0; c <= c1; c++)
                if (grid[r] && grid[r][c] === 0) grid[r][c] = 0.5;
        }

        if (!ltr) segs.reverse();
        for (const [lo, hi] of segs) {
            strips.push({
                start: ltr ? [lo, cy] : [hi, cy],
                end: ltr ? [hi, cy] : [lo, cy],
                r: r,
            });
        }
        ltr = !ltr;
    }
    return strips;
}

// ── §4.4 Voronoi roadmap (Algorithms 2 & 3) ───────────────

const N_KNN = 10; // k-nearest neighbors in Voronoi roadmap (empirically set, §4.4.3)

// Circumscribed circle of triangle (a, b, c).  Returns null for degenerate triangles.
function circumcircle(a, b, c) {
    const ax = b[0] - a[0];
    const ay = b[1] - a[1];
    const bx = c[0] - a[0];
    const by = c[1] - a[1];
    const D = 2 * (ax * by - ay * bx);
    if (Math.abs(D) < 1e-10) return null;
    const ux = (by * (ax * ax + ay * ay) - ay * (bx * bx + by * by)) / D;
    const uy = (ax * (bx * bx + by * by) - bx * (ax * ax + ay * ay)) / D;
    return { cx: a[0] + ux, cy: a[1] + uy, r2: ux * ux + uy * uy };
}

// Bowyer-Watson incremental Delaunay triangulation.
// Returns array of triangles { v:[i,j,k], cc:circumcircle }.
function delaunayTriangulation(points) {
    const N = points.length;
    if (N < 3) return [];

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    for (const [x, y] of points) {
        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
    }
    const d = Math.max(maxX - minX, maxY - minY) * 3 + 1;
    const mx = (minX + maxX) / 2;
    const my = (minY + maxY) / 2;

    // Super-triangle vertices at indices N, N+1, N+2
    const pts = [
        ...points,
        [mx - d, my - d * 2],
        [mx, my + d * 2],
        [mx + d, my - d * 2],
    ];
    const mkTri = (i, j, k) => ({
        v: [i, j, k],
        cc: circumcircle(pts[i], pts[j], pts[k]),
    });

    let tris = [mkTri(N, N + 1, N + 2)];

    for (let p = 0; p < N; p++) {
        const [px, py] = pts[p];

        // Bad triangles: circumcircle contains the new point p
        const badIdx = new Set();
        for (let t = 0; t < tris.length; t++) {
            const { cc } = tris[t];
            if (!cc) continue;
            const dx = px - cc.cx;
            const dy = py - cc.cy;
            if (dx * dx + dy * dy < cc.r2 + 1e-10) badIdx.add(t);
        }

        // Boundary polygon: edges of bad triangles not shared with another bad triangle
        const boundary = [];
        for (const t of badIdx) {
            const [a, b, c] = tris[t].v;
            for (const [e0, e1] of [
                [a, b],
                [b, c],
                [c, a],
            ]) {
                let shared = false;
                for (const t2 of badIdx) {
                    if (t2 === t) continue;
                    const v2 = tris[t2].v;
                    if (v2.includes(e0) && v2.includes(e1)) {
                        shared = true;
                        break;
                    }
                }
                if (!shared) boundary.push([e0, e1]);
            }
        }

        // Remove bad triangles; re-triangulate cavity with p
        tris = tris.filter((_, t) => !badIdx.has(t));
        for (const [e0, e1] of boundary) tris.push(mkTri(e0, e1, p));
    }

    // Drop triangles sharing a vertex with the super-triangle
    return tris.filter(({ v }) => !v.some((i) => i >= N));
}

// Sample evenly-spaced points along every edge of a polygon.
function sampleEdgePoints(poly, spacing) {
    const pts = [];
    for (let i = 0; i < poly.length; i++) {
        const [x1, y1] = poly[i];
        const [x2, y2] = poly[(i + 1) % poly.length];
        const len = Math.hypot(x2 - x1, y2 - y1);
        if (len < 1e-9) continue;
        const n = Math.max(1, Math.round(len / spacing));
        for (let k = 0; k < n; k++) {
            const t = k / n;
            pts.push([x1 + t * (x2 - x1), y1 + t * (y2 - y1)]);
        }
    }
    return pts;
}

// Deduplicate points within epsilon (prevents Bowyer-Watson degeneracies).
function dedupPts(pts, eps = 1e-4) {
    const seen = new Set();
    return pts.filter(([x, y]) => {
        const k = `${Math.round(x / eps)},${Math.round(y / eps)}`;
        if (seen.has(k)) return false;
        seen.add(k);
        return true;
    });
}

// Build Voronoi roadmap (Algorithm 2, §4.4.3):
//  1. Site-points on boundary + obstacle edges → Delaunay triangulation.
//  2. Voronoi vertices = circumcenters of Delaunay triangles (dual graph).
//  3. Filter to free-space vertices (inside perim, outside obstacles).
//  4. Connect each free vertex to N_KNN nearest free neighbors where the
//     edge is collision-free (local planner check).
// The roadmap is built once and reused for all inter-strip transitions.
function buildVoronoiRoadmap(
    perimR,
    exR,
    exRC,
    laneWidth,
    perimBBox,
    exBBoxes,
    exCBBoxes,
) {
    // Site points on perimeter + buffered obstacle boundaries
    // Limit sampling density to at least 1.0m to avoid redundant nodes on narrow lanes
    const spacing = Math.max(laneWidth, 1.0);
    const sitePts = dedupPts([
        ...sampleEdgePoints(perimR, spacing),
        ...exR.flatMap((ex) => sampleEdgePoints(ex, spacing)),
    ]);
    if (sitePts.length < 3) return { verts: [], adj: [] };

    // Delaunay triangulation → Voronoi dual
    const tris = delaunayTriangulation(sitePts);

    // Collect unique Voronoi vertices (circumcentres) with coordinate-key dedup
    const vertMap = new Map();
    const verts = [];
    const getIdx = (cx, cy) => {
        const k = `${Math.round(cx * 1e3)},${Math.round(cy * 1e3)}`;
        if (!vertMap.has(k)) {
            vertMap.set(k, verts.length);
            verts.push([cx, cy]);
        }
        return vertMap.get(k);
    };
    for (const { cc } of tris) {
        if (cc) getIdx(cc.cx, cc.cy);
    }

    // Filter Voronoi vertices to those in free space
    const isFree = verts.map(
        (v) => pointInPoly(v, perimR) && !exRC.some((ex) => pointInPoly(v, ex)),
    );
    const freeIdxs = verts.map((_, i) => i).filter((i) => isFree[i]);

    const nv = verts.length;
    const adj = Array.from({ length: nv }, () => []);

    // Connect each free vertex to its N_KNN nearest collision-free free neighbors.
    // Optimization: Maintain a small sorted list of nearest neighbors instead of sorting all nodes.
    for (const i of freeIdxs) {
        const target = verts[i];
        const neighbors = [];
        for (const j of freeIdxs) {
            if (j === i) continue;
            const d = Math.hypot(
                verts[j][0] - target[0],
                verts[j][1] - target[1],
            );
            neighbors.push({ j, d });
        }
        neighbors.sort((a, b) => a.d - b.d);

        let connected = 0;
        for (const { j, d } of neighbors) {
            if (connected >= N_KNN) break;
            if (adj[i].some((e) => e.v === j)) {
                connected++;
                continue;
            } // already linked
            if (
                segmentFree(
                    verts[i],
                    verts[j],
                    perimR,
                    exR,
                    exRC,
                    perimBBox,
                    exBBoxes,
                    exCBBoxes,
                )
            ) {
                adj[i].push({ v: j, w: d });
                adj[j].push({ v: i, w: d });
                connected++;
            }
        }
    }

    const oldToNew = new Map();
    freeIdxs.forEach((oldIdx, newIdx) => {
        oldToNew.set(oldIdx, newIdx);
    });
    const compactedVerts = freeIdxs.map((i) => verts[i]);
    const compactedAdj = freeIdxs.map((i) =>
        adj[i]
            .filter(({ v }) => oldToNew.has(v))
            .map(({ v, w }) => ({ v: oldToNew.get(v), w })),
    );
    return { verts: compactedVerts, adj: compactedAdj };
}

// ── Algorithm 3: Dijkstra shortest path ───────────────────

function dijkstra(adj, src, dst, n) {
    const dist = Array(n).fill(Number.POSITIVE_INFINITY);
    const prev = Array(n).fill(-1);
    const done = Array(n).fill(false);
    dist[src] = 0;
    for (let iter = 0; iter < n; iter++) {
        let u = -1;
        for (let k = 0; k < n; k++)
            if (!done[k] && (u === -1 || dist[k] < dist[u])) u = k;
        if (u === -1 || dist[u] === Number.POSITIVE_INFINITY || u === dst)
            break;
        done[u] = true;
        for (const { v, w } of adj[u])
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
            }
    }
    if (dist[dst] === Number.POSITIVE_INFINITY) return null;
    const path = [];
    for (let c = dst; c !== -1; c = prev[c]) path.unshift(c);
    return path;
}

// Build visibility graph once per plan for detour routing around corners and obstacles.
function buildVisibilityGraph(
    perimR,
    exR,
    exRC,
    perimBBox,
    exBBoxes,
    exCBBoxes,
) {
    const cand = [];

    const polys =
        perimR.length > 0 && Array.isArray(perimR[0][0]) ? perimR : [perimR];
    for (const poly of polys) {
        const M = poly.length;
        if (M < 3) continue;
        const cw = isPolygonCW(poly);
        for (let i = 0; i < M; i++) {
            const pPrev = poly[(i - 1 + M) % M];
            const pCurr = poly[i];
            const pNext = poly[(i + 1) % M];
            const v1 = [pCurr[0] - pPrev[0], pCurr[1] - pPrev[1]];
            const v2 = [pNext[0] - pCurr[0], pNext[1] - pCurr[1]];
            const cross = v1[0] * v2[1] - v1[1] * v2[0];
            const isReflex = cw ? cross > 0 : cross < 0;
            if (isReflex || M <= 20) {
                const l1 = Math.hypot(v1[0], v1[1]);
                const l2 = Math.hypot(v2[0], v2[1]);
                if (l1 > 1e-6 && l2 > 1e-6) {
                    const u1 = [v1[0] / l1, v1[1] / l1];
                    const u2 = [v2[0] / l2, v2[1] / l2];
                    // perimeter vertices (inward):
                    const n1 = cw ? [u1[1], -u1[0]] : [-u1[1], u1[0]];
                    const n2 = cw ? [u2[1], -u2[0]] : [-u2[1], u2[0]];
                    const nb = [n1[0] + n2[0], n1[1] + n2[1]];
                    const lnb = Math.hypot(nb[0], nb[1]);
                    if (lnb > 1e-6) {
                        const m = [nb[0] / lnb, nb[1] / lnb];
                        cand.push([
                            pCurr[0] + m[0] * 0.1,
                            pCurr[1] + m[1] * 0.1,
                        ]);
                    }
                }
            }
        }
    }

    for (const ex of exR) {
        const M = ex.length;
        if (M < 3) continue;
        const cw = isPolygonCW(ex);
        for (let i = 0; i < M; i++) {
            const pPrev = ex[(i - 1 + M) % M];
            const pCurr = ex[i];
            const pNext = ex[(i + 1) % M];
            const v1 = [pCurr[0] - pPrev[0], pCurr[1] - pPrev[1]];
            const v2 = [pNext[0] - pCurr[0], pNext[1] - pCurr[1]];
            const l1 = Math.hypot(v1[0], v1[1]);
            const l2 = Math.hypot(v2[0], v2[1]);
            if (l1 > 1e-6 && l2 > 1e-6) {
                const u1 = [v1[0] / l1, v1[1] / l1];
                const u2 = [v2[0] / l2, v2[1] / l2];
                // exclusion vertices (outward):
                const n1 = cw ? [-u1[1], u1[0]] : [u1[1], -u1[0]];
                const n2 = cw ? [-u2[1], u2[0]] : [u2[1], -u2[0]];
                const nb = [n1[0] + n2[0], n1[1] + n2[1]];
                const lnb = Math.hypot(nb[0], nb[1]);
                if (lnb > 1e-6) {
                    const m = [nb[0] / lnb, nb[1] / lnb];
                    cand.push([pCurr[0] + m[0] * 0.1, pCurr[1] + m[1] * 0.1]);
                }
            }
        }
    }

    for (const exC of exRC) {
        const nSamp = 12;
        for (let i = 0; i < nSamp; i++) {
            const idx = Math.floor((i * exC.length) / nSamp);
            cand.push(exC[idx]);
        }
    }

    const verts = [];
    for (let i = 0; i < cand.length; i++) {
        const pt = cand[i];
        if (!pointInPoly(pt, perimR)) continue;
        if (exRC.some((ex) => pointInPoly(pt, ex))) continue;
        verts.push(pt);
    }

    const n = verts.length;
    const adj = Array.from({ length: n }, () => []);
    for (let i = 0; i < n; i++) {
        const pi = verts[i];
        for (let j = i + 1; j < n; j++) {
            const pj = verts[j];
            const d = Math.hypot(pj[0] - pi[0], pj[1] - pi[1]);
            if (
                segmentFree(
                    pi,
                    pj,
                    perimR,
                    exR,
                    exRC,
                    perimBBox,
                    exBBoxes,
                    exCBBoxes,
                )
            ) {
                adj[i].push({ v: j, w: d });
                adj[j].push({ v: i, w: d });
            }
        }
    }

    return { verts, adj };
}

// Route from → to using a full visibility graph fallback (Algorithm guarantee for 2D polygons).
// Used when Voronoi roadmap is disconnected or cannot connect to endpoints.
function routeVisibilityGraph(
    from,
    to,
    visGraph,
    perimR,
    _perimR_tolerance,
    exR,
    exRC,
    perimBBox,
    _perimBBox_tolerance,
    exBBoxes,
    exCBBoxes,
) {
    if (
        segmentFree(from, to, perimR, exR, exRC, perimBBox, exBBoxes, exCBBoxes)
    ) {
        return [from, to];
    }

    const { verts: sv, adj: sa } = visGraph || { verts: [], adj: [] };
    const ns = sv.length;
    if (ns === 0) return null;

    const pts = [from, to, ...sv];
    const n = pts.length;
    const adj = [[], []];
    for (let i = 0; i < ns; i++) {
        adj.push(sa[i].map(({ v, w }) => ({ v: v + 2, w })));
    }

    for (const qi of [0, 1]) {
        const pt = pts[qi];
        for (let j = 0; j < ns; j++) {
            const sj = sv[j];
            const d = Math.hypot(sj[0] - pt[0], sj[1] - pt[1]);
            if (
                segmentFree(
                    pt,
                    sj,
                    perimR,
                    exR,
                    exRC,
                    perimBBox,
                    exBBoxes,
                    exCBBoxes,
                )
            ) {
                adj[qi].push({ v: j + 2, w: d });
                adj[j + 2].push({ v: qi, w: d });
            }
        }
    }

    const idxPath = dijkstra(adj, 0, 1, n);
    if (!idxPath) return null;

    return idxPath.map((i) => pts[i]);
}

// Route from → to via the Voronoi roadmap + Dijkstra (Eq. 10, §4.4.2):
//   lse = ‖p(s)−p(e)‖  if collision-free,
//         lse_Voronoi   otherwise.
// from and to are wired into the static roadmap as temporary query nodes.
function routeTransition(
    from,
    to,
    roadmap,
    visGraph,
    perimR,
    perimR_tolerance,
    exR,
    exRC,
    perimBBox = null,
    perimBBox_tolerance = null,
    exBBoxes = [],
    exCBBoxes = [],
) {
    // Check if both endpoints are inside the tolerance perimeter
    const fromInTolerance =
        perimR_tolerance.length >= 3 && pointInPoly(from, perimR_tolerance);
    const toInTolerance =
        perimR_tolerance.length >= 3 && pointInPoly(to, perimR_tolerance);
    const useToleranceForDirect = fromInTolerance && toInTolerance;

    if (useToleranceForDirect) {
        if (
            segmentFree(
                from,
                to,
                perimR_tolerance,
                exR,
                exRC,
                perimBBox_tolerance,
                exBBoxes,
                exCBBoxes,
            )
        ) {
            return [from, to];
        }
    } else {
        if (
            segmentFree(
                from,
                to,
                perimR,
                exR,
                exRC,
                perimBBox,
                exBBoxes,
                exCBBoxes,
            )
        ) {
            return [from, to];
        }
    }

    const { verts: sv, adj: sa } = roadmap || { verts: [], adj: [] };
    const ns = sv.length;
    let path = null;

    if (ns > 0) {
        // Augment roadmap: from=0, to=1; prebuilt roadmap nodes start at index 2
        const pts = [from, to, ...sv];
        const n = pts.length;
        const adj = [[], []];
        for (let i = 0; i < ns; i++)
            adj.push(sa[i].map(({ v, w }) => ({ v: v + 2, w })));

        let allConnected = true;
        for (const qi of [0, 1]) {
            const target = pts[qi];
            const dists = [];
            for (let i = 0; i < ns; i++) {
                const d = Math.hypot(
                    sv[i][0] - target[0],
                    sv[i][1] - target[1],
                );
                dists.push({ i, d });
            }
            dists.sort((a, b) => a.d - b.d);

            let connected = 0;
            const targetInTolerance =
                perimR_tolerance.length >= 3 &&
                pointInPoly(target, perimR_tolerance);

            // First pass: try to connect cleanly within the tolerance perimeter
            if (targetInTolerance) {
                for (const { i, d } of dists) {
                    if (connected >= N_KNN) break;
                    if (
                        segmentFree(
                            pts[qi],
                            sv[i],
                            perimR_tolerance,
                            exR,
                            exRC,
                            perimBBox_tolerance,
                            exBBoxes,
                            exCBBoxes,
                        )
                    ) {
                        adj[qi].push({ v: i + 2, w: d });
                        adj[i + 2].push({ v: qi, w: d });
                        connected++;
                    }
                }
            }

            // Fallback pass (or default if target is outside tolerance): wire using actual boundary
            if (connected === 0) {
                for (const { i, d } of dists) {
                    if (connected >= N_KNN) break;
                    if (
                        segmentFree(
                            pts[qi],
                            sv[i],
                            perimR,
                            exR,
                            exRC,
                            perimBBox,
                            exBBoxes,
                            exCBBoxes,
                        )
                    ) {
                        adj[qi].push({ v: i + 2, w: d });
                        adj[i + 2].push({ v: qi, w: d });
                        connected++;
                    }
                }
            }
            if (connected === 0) {
                allConnected = false;
                break;
            }
        }

        if (allConnected) {
            const idxPath = dijkstra(adj, 0, 1, n);
            if (idxPath) {
                path = idxPath.map((i) => pts[i]);
            }
        }
    }

    if (!path) {
        path = routeVisibilityGraph(
            from,
            to,
            visGraph,
            perimR,
            perimR_tolerance,
            exR,
            exRC,
            perimBBox,
            perimBBox_tolerance,
            exBBoxes,
            exCBBoxes,
        );
    }
    if (!path) return null;

    // Greedy path smoothing (short-cutting)
    if (path.length > 2) {
        const smoothed = [path[0]];
        let curr = 0;
        while (curr < path.length - 1) {
            let next = curr + 1;
            for (let i = path.length - 1; i > curr + 1; i--) {
                const p1 = path[curr];
                const p2 = path[i];
                const p1InTol =
                    perimR_tolerance.length >= 3 &&
                    pointInPoly(p1, perimR_tolerance);
                const p2InTol =
                    perimR_tolerance.length >= 3 &&
                    pointInPoly(p2, perimR_tolerance);
                const useTol = p1InTol && p2InTol;
                if (
                    segmentFree(
                        p1,
                        p2,
                        useTol ? perimR_tolerance : perimR,
                        exR,
                        exRC,
                        useTol ? perimBBox_tolerance : perimBBox,
                        exBBoxes,
                        exCBBoxes,
                    )
                ) {
                    next = i;
                    break;
                }
            }
            smoothed.push(path[next]);
            curr = next;
        }
        path = smoothed;
    }

    // Verify all legs in final path
    for (let i = 0; i < path.length - 1; i++) {
        if (
            !segmentFree(
                path[i],
                path[i + 1],
                perimR,
                exR,
                exRC,
                perimBBox,
                exBBoxes,
                exCBBoxes,
            )
        ) {
            return null;
        }
    }

    return path;
}

function pathLength(path) {
    let len = 0;
    for (let i = 1; i < path.length; i++) {
        len += Math.hypot(
            path[i][0] - path[i - 1][0],
            path[i][1] - path[i - 1][1],
        );
    }
    return len;
}

// ── Main entry point ───────────────────────────────────────

function getClosestPointOnSegment(p, a, b) {
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const apx = p[0] - a[0];
    const apy = p[1] - a[1];
    const ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-9) return [a[0], a[1]];

    let t = (apx * abx + apy * aby) / ab2;
    t = Math.max(0, Math.min(1, t));
    return [a[0] + t * abx, a[1] + t * aby];
}

function getClosestPointOnPoly(pt, poly) {
    let minD = Number.POSITIVE_INFINITY;
    let closestPt = [pt[0], pt[1]];
    for (let i = 0; i < poly.length; i++) {
        const a = poly[i];
        const b = poly[(i + 1) % poly.length];
        const cp = getClosestPointOnSegment(pt, a, b);
        const d = Math.hypot(pt[0] - cp[0], pt[1] - cp[1]);
        if (d < minD) {
            minD = d;
            closestPt = cp;
        }
    }
    return closestPt;
}

function removeLoopsAndBacktracks(path, threshold = 0.2, isFree = null) {
    if (path.length < 3) return path.slice();
    const result = path.slice();
    let changed = true;
    let iterations = 0;
    while (changed && iterations < 15) {
        changed = false;
        iterations++;
        const n = result.length;
        for (let i = 0; i < n - 2; i++) {
            const p1 = result[i];
            const maxJ = Math.min(n, i + 10);
            for (let j = maxJ - 1; j > i + 1; j--) {
                const p2 = result[j];
                const d = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                if (d < threshold && !(j >= n - 2 && i <= 1)) {
                    if (isFree && !isFree(p1, p2)) continue;
                    result.splice(i + 1, j - i - 1);
                    changed = true;
                    break;
                }
            }
            if (changed) break;
        }
    }

    const finalResult = [];
    if (result.length > 0) {
        finalResult.push(result[0]);
        for (let i = 1; i < result.length; i++) {
            const last = finalResult[finalResult.length - 1];
            const curr = result[i];
            if (Math.hypot(curr[0] - last[0], curr[1] - last[1]) > 1e-9) {
                finalResult.push(curr);
            }
        }
    }
    return finalResult;
}

function removeSharpPeaks(pts, isFree = null) {
    const result = pts.slice();
    let changed = true;
    let iterations = 0;
    while (changed && iterations < 1000) {
        changed = false;
        const N = result.length;
        if (N < 3) break;
        for (let i = 1; i < N - 1; i++) {
            const prev = result[i - 1];
            const curr = result[i];
            const next = result[i + 1];

            const d1 = [curr[0] - prev[0], curr[1] - prev[1]];
            const d2 = [next[0] - curr[0], next[1] - curr[1]];
            const len1 = Math.hypot(d1[0], d1[1]);
            const len2 = Math.hypot(d2[0], d2[1]);
            if (len1 < 1e-5 || len2 < 1e-5) continue;

            const cosAngle = (d1[0] * d2[0] + d1[1] * d2[1]) / (len1 * len2);
            if (cosAngle < -0.5) {
                // angle > 120 degrees
                if (isFree && !isFree(prev, next)) continue;
                result.splice(i, 1);
                changed = true;
                break;
            }
        }
        iterations++;
    }
    return result;
}

function condenseCollinearPoints(pts, tolerance = 0.01, isFree = null) {
    if (pts.length < 2) return pts.slice();

    // Stage 1: remove collinear points with a tight tolerance (2cm when tolerance > 0.05, else tolerance)
    const collinearTolerance =
        tolerance > 0.05 ? 0.02 : Math.max(0.01, tolerance);

    let collinearRemoved = pts.slice();
    if (pts.length >= 3) {
        const result = [pts[0]];
        let lastKeptIdx = 0;
        for (let i = 1; i < pts.length; i++) {
            if (i === pts.length - 1) {
                result.push(pts[i]);
                break;
            }

            let ok = true;
            const segmentStart = pts[lastKeptIdx];
            const segmentEnd = pts[i + 1];

            for (let j = lastKeptIdx + 1; j <= i; j++) {
                const dSq = getSquareSegmentDistance(
                    pts[j],
                    segmentStart,
                    segmentEnd,
                );
                if (dSq > collinearTolerance * collinearTolerance) {
                    ok = false;
                    break;
                }
            }

            if (ok && isFree && !isFree(segmentStart, segmentEnd)) {
                ok = false;
            }

            if (!ok) {
                result.push(pts[i]);
                lastKeptIdx = i;
            }
        }
        collinearRemoved = result;
    }

    // Stage 2: remove consecutive close vertices if custom tolerance is active
    if (tolerance > 0.05) {
        const finalResult = [collinearRemoved[0]];
        for (let i = 1; i < collinearRemoved.length; i++) {
            const last = finalResult[finalResult.length - 1];
            const curr = collinearRemoved[i];
            const d = Math.hypot(curr[0] - last[0], curr[1] - last[1]);
            if (i === collinearRemoved.length - 1 || d >= tolerance) {
                finalResult.push(curr);
            } else if (
                isFree &&
                !isFree(
                    last,
                    collinearRemoved[
                        Math.min(i + 1, collinearRemoved.length - 1)
                    ],
                )
            ) {
                finalResult.push(curr);
            }
        }
        return finalResult;
    }

    return collinearRemoved;
}

function getPolygonArea(poly) {
    if (poly.length < 3) return 0;
    let area = 0;
    const N = poly.length;
    for (let i = 0; i < N; i++) {
        const [x1, y1] = poly[i];
        const [x2, y2] = poly[(i + 1) % N];
        area += x1 * y2 - x2 * y1;
    }
    return 0.5 * Math.abs(area);
}

function polygonOverlapsPolygon(polyA, polyB) {
    if (!polyA || !polyB || polyA.length < 3 || polyB.length < 3) return false;
    let minAx = Infinity,
        maxAx = -Infinity,
        minAy = Infinity,
        maxAy = -Infinity;
    for (let i = 0; i < polyA.length; i++) {
        const x = polyA[i][0];
        const y = polyA[i][1];
        if (x < minAx) minAx = x;
        if (x > maxAx) maxAx = x;
        if (y < minAy) minAy = y;
        if (y > maxAy) maxAy = y;
    }
    let minBx = Infinity,
        maxBx = -Infinity,
        minBy = Infinity,
        maxBy = -Infinity;
    for (let i = 0; i < polyB.length; i++) {
        const x = polyB[i][0];
        const y = polyB[i][1];
        if (x < minBx) minBx = x;
        if (x > maxBx) maxBx = x;
        if (y < minBy) minBy = y;
        if (y > maxBy) maxBy = y;
    }
    if (maxAx < minBx || maxBx < minAx || maxAy < minBy || maxBy < minAy) {
        return false;
    }
    for (let i = 0; i < polyA.length; i++) {
        if (pointInPoly(polyA[i], polyB)) return true;
    }
    for (let i = 0; i < polyB.length; i++) {
        if (pointInPoly(polyB[i], polyA)) return true;
    }
    for (let i = 0; i < polyA.length; i++) {
        const a1 = polyA[i];
        const a2 = polyA[(i + 1) % polyA.length];
        for (let j = 0; j < polyB.length; j++) {
            const b1 = polyB[j];
            const b2 = polyB[(j + 1) % polyB.length];
            if (getSegIntersection(a1, a2, b1, b2)) return true;
        }
    }
    return false;
}

function generateCoveragePath(
    perimCoords,
    exclusions,
    laneWidth,
    buffer,
    nPasses = 0,
    direction = "CW",
    tolerance = 0,
    skipLanes = 0,
    sweepMode = "auto",
    sweepAngle = 0,
    circleSegments = 64,
    spiralMode = false,
    reverseSpiral = false,
    headlandMargin = 0,
    combineWaypoints = 0,
    isPiece = false,
) {
    if (perimCoords.length < 3) return null;
    const warnings = [];

    const oLat = perimCoords.reduce((s, c) => s + c[0], 0) / perimCoords.length;
    const oLon = perimCoords.reduce((s, c) => s + c[1], 0) / perimCoords.length;
    const xy = ([lat, lon]) => toLocal(lat, lon, oLat, oLon);

    // Project perimeter; drop closing duplicate if present
    let perim = perimCoords.map(xy);
    if (perim.length > 1) {
        const [fx, fy] = perim[0];
        const [lx, ly] = perim.at(-1);
        if (Math.hypot(fx - lx, fy - ly) < 0.01) perim.pop();
    }

    if (combineWaypoints > 0) {
        const simplifiedPerim = simplifyGreedy(perim, combineWaypoints);
        if (simplifiedPerim.length >= 3) {
            perim = simplifiedPerim;
        }
    }

    // Project + buffer each exclusion zone
    const exPolys = exclusions.map((s) => {
        if (s.type === "circle") {
            const [cx, cy] = toLocal(s.lat, s.lon, oLat, oLon);
            return circlePoly(cx, cy, s.radius + buffer, circleSegments);
        }
        let poly = s.vertices.map(xy);
        if (poly.length > 1) {
            const [fx, fy] = poly[0];
            const [lx, ly] = poly.at(-1);
            if (Math.hypot(fx - lx, fy - ly) < 0.01) poly.pop();
        }
        if (combineWaypoints > 0) {
            const simplifiedEx = simplifyGreedy(poly, combineWaypoints);
            if (simplifiedEx.length >= 3) {
                poly = simplifiedEx;
            }
        }
        return buffer > 0 ? expandPoly(poly, buffer) : poly;
    });

    // Subtract intersecting exclusion zones from perimeter in local metric space
    let navigablePerimeterPieces = [perim];
    const isIntersecting = exclusions.map(() => false);
    if (!isPiece) {
        for (let eIdx = 0; eIdx < exPolys.length; eIdx++) {
            const ex = exPolys[eIdx];
            // Only boundary exclusions (obstacles that cross or touch the yard perimeter)
            // can partition or cut the perimeter. Interior obstacles are handled via headlands.
            let crossesPerim = false;
            for (let i = 0; i < ex.length; i++) {
                if (!pointInPoly(ex[i], perim)) {
                    crossesPerim = true;
                    break;
                }
            }
            if (!crossesPerim) {
                for (let i = 0; i < ex.length && !crossesPerim; i++) {
                    const a1 = ex[i];
                    const a2 = ex[(i + 1) % ex.length];
                    for (let j = 0; j < perim.length; j++) {
                        const b1 = perim[j];
                        const b2 = perim[(j + 1) % perim.length];
                        if (getSegIntersection(a1, a2, b1, b2)) {
                            crossesPerim = true;
                            break;
                        }
                    }
                }
            }
            if (!crossesPerim) {
                continue;
            }

            const nextPieces = subtractPolygonsMulti(
                navigablePerimeterPieces,
                ex,
            );
            const totalAreaBefore = navigablePerimeterPieces.reduce(
                (sum, p) => sum + Math.abs(getPolygonArea(p)),
                0,
            );
            const totalAreaAfter = nextPieces.reduce(
                (sum, p) => sum + Math.abs(getPolygonArea(p)),
                0,
            );
            if (nextPieces.length > 0 && totalAreaAfter >= 0.1) {
                if (Math.abs(totalAreaBefore - totalAreaAfter) > 0.01) {
                    isIntersecting[eIdx] = true;
                }
                navigablePerimeterPieces = nextPieces;
            } else {
                if (totalAreaBefore >= 0.1) {
                    warnings.push(
                        "Boundary obstacle subtraction produced degenerate polygon; falling back to previous perimeter",
                    );
                }
            }
        }
    } else {
        // In a sub-piece, the perimeter has already been partitioned by boundary exclusions.
        // Mark only exclusions that cross or touch the piece boundary as intersecting.
        // Fully interior obstacles inside the piece must NOT be marked so they receive headland loops.
        for (let eIdx = 0; eIdx < exPolys.length; eIdx++) {
            const ex = exPolys[eIdx];
            let crosses = false;
            for (let i = 0; i < ex.length; i++) {
                if (!pointInPoly(ex[i], perim)) {
                    crosses = true;
                    break;
                }
            }
            if (!crosses) {
                for (let i = 0; i < ex.length && !crosses; i++) {
                    const a1 = ex[i];
                    const a2 = ex[(i + 1) % ex.length];
                    for (let j = 0; j < perim.length; j++) {
                        const b1 = perim[j];
                        const b2 = perim[(j + 1) % perim.length];
                        if (getSegIntersection(a1, a2, b1, b2)) {
                            crosses = true;
                            break;
                        }
                    }
                }
            }
            if (crosses) {
                isIntersecting[eIdx] = true;
            }
        }
    }

    // If boundary obstacles split the yard into multiple pieces, plan each piece as an independent mission.
    // Disconnected pieces are returned in result.pieces[] without connecting legs across obstacles.
    if (!isPiece && navigablePerimeterPieces.length > 1) {
        const validPieces = [];
        const topWarnings = [...warnings];

        for (const piece of navigablePerimeterPieces) {
            const area = Math.abs(getPolygonArea(piece));
            if (area < 1.0) {
                topWarnings.push(
                    `Disconnected yard piece (${area.toFixed(1)} m²) dropped: area is too small to plan coverage`,
                );
                continue;
            }
            const pieceCoords = piece.map(([x, y]) =>
                fromLocal(x, y, oLat, oLon),
            );
            const pieceExclusions = exclusions.filter((_s, idx) => {
                const ex = exPolys[idx];
                return polygonOverlapsPolygon(ex, piece);
            });
            const res = generateCoveragePath(
                pieceCoords,
                pieceExclusions,
                laneWidth,
                buffer,
                nPasses,
                direction,
                tolerance,
                skipLanes,
                sweepMode,
                sweepAngle,
                circleSegments,
                spiralMode,
                reverseSpiral,
                headlandMargin,
                combineWaypoints,
                true,
            );
            if (res?.path && res.path.length > 0) {
                validPieces.push(res);
                if (res.warnings && res.warnings.length > 0) {
                    const pNum = validPieces.length;
                    for (const w of res.warnings) {
                        topWarnings.push(`Piece ${pNum}: ${w}`);
                    }
                }
            } else {
                topWarnings.push(
                    `Disconnected yard piece (${area.toFixed(1)} m²) dropped: could not generate coverage path`,
                );
            }
        }

        if (validPieces.length > 0) {
            const primary = validPieces[0];
            const totalDistAll = validPieces.reduce(
                (sum, p) => sum + p.totalDistM,
                0,
            );
            const sweepDistAll = validPieces.reduce(
                (sum, p) => sum + p.sweepDistM,
                0,
            );
            const coveredAreaAll = validPieces.reduce(
                (sum, p) => sum + p.coveredAreaSqM,
                0,
            );

            return {
                path: primary.path,
                count: primary.count,
                totalDistM: Math.round(totalDistAll * 10) / 10,
                sweepDistM: Math.round(sweepDistAll * 10) / 10,
                coveredAreaSqM: Math.round(coveredAreaAll * 10) / 10,
                totalDistAllM: totalDistAll,
                sweepDistAllM: sweepDistAll,
                coveredAreaAllSqM: coveredAreaAll,
                warnings: Array.from(new Set(topWarnings)),
                pieces: validPieces,
            };
        }
    }

    let navigablePerimeter = navigablePerimeterPieces[0] || perim;
    if (
        navigablePerimeter.length < 3 ||
        Math.abs(getPolygonArea(navigablePerimeter)) < 0.1
    ) {
        navigablePerimeter = perim;
    }

    let computedNPasses = nPasses;
    if (spiralMode) {
        let p = 0;
        const yardInsetLapsTemp = [];
        while (true) {
            const inset = insetPoly(navigablePerimeter, p * laneWidth);
            const area = getPolygonArea(inset);
            if (
                area < 0.01 ||
                (yardInsetLapsTemp.length > 0 &&
                    area >=
                        getPolygonArea(
                            yardInsetLapsTemp[yardInsetLapsTemp.length - 1],
                        ))
            ) {
                break;
            }
            yardInsetLapsTemp.push(inset);
            p++;
            if (p > 1000) break;
        }
        computedNPasses = yardInsetLapsTemp.length;
    }

    const stripOffset =
        computedNPasses > 0
            ? Math.max(0, computedNPasses - 1) * laneWidth + headlandMargin
            : Math.max(0, headlandMargin);

    let stripPolysOrig = [];
    let offlineFillPolys = null;
    let offlineExpandedPolys = null;
    if (stripOffset <= 0) {
        stripPolysOrig = [navigablePerimeter];
    } else {
        stripPolysOrig = insetPolyMulti(navigablePerimeter, stripOffset);
        if (!stripPolysOrig.length) {
            const single = insetPoly(navigablePerimeter, stripOffset);
            if (single.length >= 3) stripPolysOrig = [single];
        }
        // In narrow regions where higher perimeter passes collapse, fill uncovered areas with strips
        if (computedNPasses > 1) {
            const minStripOffset = Math.max(
                0.05,
                0.5 * laneWidth + headlandMargin,
            );
            if (minStripOffset < stripOffset) {
                const narrowPolys = insetPolyMulti(
                    navigablePerimeter,
                    minStripOffset,
                );
                if (narrowPolys.length > 0) {
                    const expandedStripPolys = stripPolysOrig.map((p) =>
                        expandPoly(p, stripOffset - minStripOffset + 0.05),
                    );
                    if (typeof ClipperLib !== "undefined") {
                        const diffPolys = differencePolyMulti(
                            narrowPolys,
                            expandedStripPolys,
                        );
                        for (const poly of diffPolys) {
                            const area = getPolygonArea(poly);
                            if (area >= laneWidth * laneWidth * 0.5) {
                                stripPolysOrig.push(poly);
                            }
                        }
                    } else {
                        offlineFillPolys = narrowPolys;
                        offlineExpandedPolys = expandedStripPolys;
                    }
                }
            }
        }
    }

    // §4.2 Optimal sweep direction from MBB of convex hull (Algorithm 1) or forced sweep direction
    let angle;
    if (sweepMode === "auto") {
        const allPolys = offlineFillPolys
            ? stripPolysOrig.concat(offlineFillPolys)
            : stripPolysOrig;
        const hullPts =
            allPolys.length > 0 && allPolys[0].length >= 3
                ? allPolys.flat()
                : navigablePerimeter;
        angle = mbbSweepAngle(convexHull(hullPts));
    } else if (sweepMode === "custom") {
        angle = (sweepAngle * Math.PI) / 180;
    } else {
        const parsedAngle = Number.parseFloat(sweepMode);
        angle = Number.isNaN(parsedAngle) ? 0 : (parsedAngle * Math.PI) / 180;
    }

    // Rotate everything into sweep space (sweep direction = +x)
    const perimOuterR = rotPts(navigablePerimeter, -angle);
    const exR = exPolys.map((p) => rotPts(p, -angle));

    // Tolerance buffer for transition path safety check
    const effBuffer = Math.max(0, buffer - tolerance);
    const exPolysTolerance = exclusions.map((s) => {
        if (s.type === "circle") {
            const [cx, cy] = toLocal(s.lat, s.lon, oLat, oLon);
            return circlePoly(cx, cy, s.radius + effBuffer, circleSegments);
        }
        const poly = s.vertices.map(xy);
        return effBuffer > 0 ? expandPoly(poly, effBuffer) : poly;
    });

    const exR_tolerance = exPolysTolerance.map((p) => rotPts(p, -angle));
    const exRC_tolerance = exR_tolerance.map((ex) => insetPoly(ex, 0.05));
    const exBBoxes_tolerance = exR_tolerance.map(getBBox);
    const exCBBoxes_tolerance = exRC_tolerance.map(getBBox);

    // Tolerance buffer for outer perimeter safety check (transition paths)
    const perimSafetyDist = Math.max(0.05, 0.5 * laneWidth - tolerance);
    const perimOuterR_tolerance = insetPoly(perimOuterR, perimSafetyDist);
    const perimOuterR_expanded = expandPoly(perimOuterR, 0.01);
    const perimBBox_tolerance = getBBox(perimOuterR_tolerance);
    const perimBBox_expanded = getBBox(perimOuterR_expanded);

    // In sweep space, the outer perimeter already has exclusions subtracted
    const navigablePerimeterRotated = perimOuterR;

    const perimR =
        stripPolysOrig.length > 0
            ? stripPolysOrig.map((poly) => rotPts(poly, -angle))
            : [perimOuterR];

    const offlineFillPolysR = offlineFillPolys
        ? offlineFillPolys.map((poly) => rotPts(poly, -angle))
        : null;
    const offlineExpandedPolysR = offlineExpandedPolys
        ? offlineExpandedPolys.map((poly) => rotPts(poly, -angle))
        : null;

    // Inset exclusion polys (5 cm) for interior point tests — avoids boundary-vertex
    // ambiguity in pointInPoly for near-boundary Voronoi vertices.
    const exRC = exR.map((ex) => insetPoly(ex, 0.05));

    // Precompute bounding boxes for faster segment intersection tests
    const perimBBox = getBBox(perimOuterR);
    const _exBBoxes = exR.map(getBBox);
    const _exCBBoxes = exRC.map(getBBox);

    // Associate rotated exclusions with their geometries in sweep space for direct point pushing
    const _exData = exclusions.map((s, idx) => {
        const polyR = exR[idx];
        if (s.type === "circle") {
            const [cx, cy] = toLocal(s.lat, s.lon, oLat, oLon);
            const [cxR, cyR] = rotPts([[cx, cy]], -angle)[0];
            return {
                type: "circle",
                cx: cxR,
                cy: cyR,
                r: s.radius + buffer,
                poly: polyR,
            };
        }
        return {
            type: "polygon",
            poly: polyR,
        };
    });

    // §4.4 Build Voronoi roadmap once; reuse for all inter-strip transitions
    const roadmap = buildVoronoiRoadmap(
        perimOuterR_tolerance,
        exR_tolerance,
        exRC_tolerance,
        laneWidth,
        perimBBox_tolerance,
        exBBoxes_tolerance,
        exCBBoxes_tolerance,
    );

    const visGraph = buildVisibilityGraph(
        perimOuterR,
        exR_tolerance,
        exRC_tolerance,
        perimBBox,
        exBBoxes_tolerance,
        exCBBoxes_tolerance,
    );

    const _isFreeRotated = (a, b) =>
        segmentFree(
            a,
            b,
            perimOuterR_tolerance,
            exR_tolerance,
            exRC_tolerance,
            perimBBox_tolerance,
            exBBoxes_tolerance,
            exCBBoxes_tolerance,
        );

    const isFreePermissiveRotated = (a, b) =>
        segmentFree(
            a,
            b,
            perimOuterR_expanded,
            exR_tolerance,
            exRC_tolerance,
            perimBBox_expanded,
            exBBoxes_tolerance,
            exCBBoxes_tolerance,
        );
    const isFreePermissiveMetric = (a, b) => {
        const [aR, bR] = rotPts([a, b], -angle);
        return isFreePermissiveRotated(aR, bR);
    };

    // Generate perimeter passes in rotated space for clean collision checking
    let perimeterPathRotated = [];
    if (computedNPasses > 0) {
        let poly = [...navigablePerimeterRotated];
        if (isPolygonCW(poly) !== (direction === "CW")) {
            poly.reverse();
        }
        const M = poly.length;

        // Compute unit miter offset vectors for each original vertex
        let origShiftVectors = [];
        const cw = isPolygonCW(poly);
        const sign = cw ? -1 : 1;
        for (let i = 0; i < M; i++) {
            const A = poly[(i - 1 + M) % M];
            const B = poly[i];
            const C = poly[(i + 1) % M];

            const dx1 = B[0] - A[0];
            const dy1 = B[1] - A[1];
            const len1 = Math.hypot(dx1, dy1);
            const dx2 = C[0] - B[0];
            const dy2 = C[1] - B[1];
            const len2 = Math.hypot(dx2, dy2);

            if (len1 < 1e-9 || len2 < 1e-9) {
                origShiftVectors.push([0, 0]);
                continue;
            }

            const v1 = [dx1 / len1, dy1 / len1];
            const v2 = [dx2 / len2, dy2 / len2];

            const n1 = [-v1[1], v1[0]];
            const n2 = [-v2[1], v2[0]];

            const nb = [n1[0] + n2[0], n1[1] + n2[1]];
            const lenB = Math.hypot(nb[0], nb[1]);
            const m = lenB < 1e-9 ? n1 : [nb[0] / lenB, nb[1] / lenB];

            const cosHalfAngle = n1[0] * m[0] + n1[1] * m[1];

            let L = sign;
            if (Math.abs(cosHalfAngle) > 0.1) {
                L = sign / cosHalfAngle;
                if (Math.abs(L) > 4) L = Math.sign(L) * 4;
            } else {
                L = sign * 4;
            }
            origShiftVectors.push([m[0] * L, m[1] * L]);
        }

        // Deliberately select start vertex far from obstacles with room for ramp
        let bestStartIdx = 0;
        let bestScore = Number.NEGATIVE_INFINITY;
        for (let i = 0; i < M; i++) {
            const A = poly[(i - 1 + M) % M];
            const B = poly[i];
            const nextPt = poly[(i + 1) % M];
            const edgeLen = Math.hypot(nextPt[0] - B[0], nextPt[1] - B[1]);
            let minDist = Number.POSITIVE_INFINITY;
            for (const ex of exR_tolerance) {
                if (ex.length < 3) continue;
                const cp = getClosestPointOnPoly(B, ex);
                const d = Math.hypot(cp[0] - B[0], cp[1] - B[1]);
                minDist = Math.min(minDist, d);
            }
            const cross =
                (B[0] - A[0]) * (nextPt[1] - B[1]) -
                (B[1] - A[1]) * (nextPt[0] - B[0]);
            const isConvex = cw ? cross < 0 : cross > 0;
            let score = Math.min(minDist, 10) * 3;
            if (minDist >= laneWidth * 1.5) score += 25;
            else if (minDist < laneWidth * 0.5) score -= 50;
            if (edgeLen >= laneWidth * 1.5) score += 15;
            if (isConvex) score += 10;
            if (score > bestScore) {
                bestScore = score;
                bestStartIdx = i;
            }
        }
        if (bestStartIdx > 0) {
            poly = [
                ...poly.slice(bestStartIdx),
                ...poly.slice(0, bestStartIdx),
            ];
            origShiftVectors = [
                ...origShiftVectors.slice(bestStartIdx),
                ...origShiftVectors.slice(0, bestStartIdx),
            ];
        }

        const yardInsetLaps = [];
        for (let p = 0; p < computedNPasses; p++) {
            yardInsetLaps.push(insetPolyMulti(perimOuterR, p * laneWidth));
        }

        const projectPointIntoLap = (pt, lapPolys) => {
            if (!lapPolys || lapPolys.length === 0) return pt;
            for (const poly of lapPolys) {
                if (pointInPoly(pt, poly)) return pt;
            }
            let bestPt = pt;
            let minDist = Number.POSITIVE_INFINITY;
            for (const poly of lapPolys) {
                if (poly.length < 3) continue;
                const cp = getClosestPointOnPoly(pt, poly);
                const d = Math.hypot(cp[0] - pt[0], cp[1] - pt[1]);
                if (d < minDist) {
                    minDist = d;
                    bestPt = cp;
                }
            }
            return bestPt;
        };

        const spacing = Math.max(laneWidth / 4, 0.5);

        const isPointFree = (pt, nominalDist) => {
            for (const ex of exR_tolerance) {
                if (pointInPoly(pt, ex)) return false;
            }
            if (nominalDist < perimSafetyDist) {
                return (
                    perimOuterR_expanded.length < 3 ||
                    pointInPoly(pt, perimOuterR_expanded)
                );
            }
            return (
                perimOuterR_tolerance.length < 3 ||
                pointInPoly(pt, perimOuterR_tolerance)
            );
        };

        for (let p = 0; p < computedNPasses; p++) {
            const passPoints = [];
            const isReversedPass = spiralMode && reverseSpiral && p >= nPasses;
            const nominalDist = p * laneWidth;

            if (isReversedPass) {
                // CCW traversal starting at poly[1]: poly[1] -> poly[0] -> poly[M-1] -> ... -> poly[2] -> poly[1]
                for (let i = 0; i < M; i++) {
                    const p1 = poly[(1 - i + M) % M];
                    const p2 = poly[(1 - i - 1 + M) % M];
                    const sv1 = origShiftVectors[(1 - i + M) % M];
                    const sv2 = origShiftVectors[(1 - i - 1 + M) % M];

                    const len = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                    if (len < 1e-9) continue;

                    const n = Math.max(1, Math.round(len / spacing));
                    for (let k = 0; k < n; k++) {
                        const t = k / n;
                        const pt = [
                            p1[0] + t * (p2[0] - p1[0]),
                            p1[1] + t * (p2[1] - p1[1]),
                        ];
                        const sv = [
                            sv1[0] + t * (sv2[0] - sv1[0]),
                            sv1[1] + t * (sv2[1] - sv1[1]),
                        ];

                        let dist;
                        let currentYardInset;

                        if (p > 0 && i === 0) {
                            const rampLen = Math.min(
                                len * 0.5,
                                Math.max(laneWidth * 1.5, 2.0),
                            );
                            const rampFrac = Math.min(1.0, (t * len) / rampLen);
                            dist = (p - 1 + rampFrac) * laneWidth;
                            currentYardInset =
                                rampFrac >= 1.0
                                    ? yardInsetLaps[Math.max(0, p)]
                                    : yardInsetLaps[Math.max(0, p - 1)];
                        } else {
                            dist = p * laneWidth;
                            currentYardInset = yardInsetLaps[Math.max(0, p)];
                        }

                        const offset = [
                            pt[0] + sv[0] * dist,
                            pt[1] + sv[1] * dist,
                        ];

                        const finalPt = projectPointIntoLap(
                            offset,
                            currentYardInset,
                        );
                        passPoints.push(finalPt);
                    }
                }
                // Push the final point poly[1] to complete the loop
                {
                    const pt = poly[1];
                    const sv = origShiftVectors[1];
                    const dist = p * laneWidth;
                    const offset = [pt[0] + sv[0] * dist, pt[1] + sv[1] * dist];
                    const targetLapIdx = Math.max(0, p);
                    const finalPt = projectPointIntoLap(
                        offset,
                        yardInsetLaps[targetLapIdx],
                    );
                    passPoints.push(finalPt);
                }
            } else {
                // CW traversal: poly[0] -> poly[1] -> ... -> poly[M-1] -> poly[0]
                const isLastForward = reverseSpiral && p === nPasses - 1;
                for (let i = 0; i < M; i++) {
                    const p1 = poly[i];
                    const p2 = poly[(i + 1) % M];
                    const sv1 = origShiftVectors[i];
                    const sv2 = origShiftVectors[(i + 1) % M];

                    const len = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                    if (len < 1e-9) continue;

                    const n = Math.max(1, Math.round(len / spacing));
                    for (let k = 0; k < n; k++) {
                        const t = k / n;
                        const pt = [
                            p1[0] + t * (p2[0] - p1[0]),
                            p1[1] + t * (p2[1] - p1[1]),
                        ];
                        const sv = [
                            sv1[0] + t * (sv2[0] - sv1[0]),
                            sv1[1] + t * (sv2[1] - sv1[1]),
                        ];

                        let dist;
                        let currentYardInset;

                        if (p > 0 && i === 0) {
                            const rampLen = Math.min(
                                len * 0.5,
                                Math.max(laneWidth * 1.5, 2.0),
                            );
                            const rampFrac = Math.min(1.0, (t * len) / rampLen);
                            dist = (p - 1 + rampFrac) * laneWidth;
                            currentYardInset =
                                rampFrac >= 1.0
                                    ? yardInsetLaps[Math.max(0, p)]
                                    : yardInsetLaps[Math.max(0, p - 1)];
                        } else {
                            dist = p * laneWidth;
                            currentYardInset = yardInsetLaps[Math.max(0, p)];
                        }

                        const offset = [
                            pt[0] + sv[0] * dist,
                            pt[1] + sv[1] * dist,
                        ];

                        const finalPt = projectPointIntoLap(
                            offset,
                            currentYardInset,
                        );
                        passPoints.push(finalPt);
                    }
                }

                // Close the loop: always land at poly[0], then for the last
                // forward pass also append poly[1] so the reversal can begin
                // seamlessly at that vertex without retracing.
                for (const closeIdx of isLastForward ? [0, 1] : [0]) {
                    const pt = poly[closeIdx];
                    const sv = origShiftVectors[closeIdx];
                    const dist = p * laneWidth;
                    const offset = [pt[0] + sv[0] * dist, pt[1] + sv[1] * dist];
                    const finalPt = projectPointIntoLap(
                        offset,
                        yardInsetLaps[p],
                    );
                    passPoints.push(finalPt);
                }
            }

            // Check if this pass has any free points
            const hasFreePoint = passPoints.some(
                (pt) =>
                    isPointFree(pt, nominalDist) &&
                    !exR_tolerance.some((ex) => pointInPoly(pt, ex)),
            );
            if (!hasFreePoint) {
                continue;
            }

            // Connect pass points in free space, routing around obstacles when needed
            const routedPassPoints = [];
            if (passPoints.length > 0) {
                let startIdx = 0;
                while (
                    startIdx < passPoints.length &&
                    exR_tolerance.some((ex) =>
                        pointInPoly(passPoints[startIdx], ex),
                    )
                ) {
                    startIdx++;
                }
                if (startIdx >= passPoints.length) continue;

                const reordered = [
                    ...passPoints.slice(startIdx),
                    ...passPoints.slice(0, startIdx),
                ];

                routedPassPoints.push(reordered[0]);
                let curr = 0;
                let next = 1;
                while (next < reordered.length) {
                    while (
                        next < reordered.length &&
                        exR_tolerance.some((ex) =>
                            pointInPoly(reordered[next], ex),
                        )
                    ) {
                        next++;
                    }
                    if (next >= reordered.length) break;

                    const p1 = reordered[curr];
                    const p2 = reordered[next];
                    const segDist = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);

                    if (segDist < 0.05) {
                        curr = next;
                        next++;
                    } else if (
                        segmentFree(
                            p1,
                            p2,
                            perimOuterR_expanded,
                            exR_tolerance,
                            exRC_tolerance,
                            null,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        routedPassPoints.push(p2);
                        curr = next;
                        next++;
                    } else {
                        const transit = routeTransition(
                            p1,
                            p2,
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        if (transit) {
                            routedPassPoints.push(...transit.slice(1));
                            curr = next;
                            next++;
                        } else {
                            next++;
                        }
                    }
                }
            }

            // Append cleanPassPoints to perimeterPathRotated
            if (routedPassPoints.length > 0) {
                const cleanPassPoints = removeLoopsAndBacktracks(
                    routedPassPoints,
                    Math.min(0.2, laneWidth * 0.3),
                );
                if (perimeterPathRotated.length > 0) {
                    // Transition from previous pass's end to current pass's start
                    const prevEnd = perimeterPathRotated.at(-1);
                    const currStart = cleanPassPoints[0];
                    const distToPrev = Math.hypot(
                        currStart[0] - prevEnd[0],
                        currStart[1] - prevEnd[1],
                    );
                    if (distToPrev < 0.01) {
                        perimeterPathRotated.push(...cleanPassPoints.slice(1));
                    } else if (
                        segmentFree(
                            prevEnd,
                            currStart,
                            perimOuterR_expanded,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox_expanded,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        perimeterPathRotated.push(...cleanPassPoints);
                    } else {
                        const transit = routeTransition(
                            prevEnd,
                            currStart,
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        if (transit) {
                            perimeterPathRotated.push(...transit.slice(1));
                            perimeterPathRotated.push(
                                ...cleanPassPoints.slice(1),
                            );
                        } else {
                            let routed = false;
                            for (
                                let ptIdx = 1;
                                ptIdx < cleanPassPoints.length;
                                ptIdx++
                            ) {
                                const altTransit = routeTransition(
                                    prevEnd,
                                    cleanPassPoints[ptIdx],
                                    roadmap,
                                    visGraph,
                                    perimOuterR,
                                    perimOuterR_tolerance,
                                    exR_tolerance,
                                    exRC_tolerance,
                                    perimBBox,
                                    perimBBox_tolerance,
                                    exBBoxes_tolerance,
                                    exCBBoxes_tolerance,
                                );
                                if (altTransit) {
                                    const reorderedPass = [
                                        ...cleanPassPoints.slice(ptIdx),
                                        ...cleanPassPoints.slice(0, ptIdx),
                                        cleanPassPoints[ptIdx],
                                    ];
                                    perimeterPathRotated.push(
                                        ...altTransit.slice(1),
                                    );
                                    perimeterPathRotated.push(
                                        ...reorderedPass.slice(1),
                                    );
                                    routed = true;
                                    break;
                                }
                            }
                            if (!routed) {
                                warnings.push(
                                    "Could not route transition between perimeter passes",
                                );
                            }
                        }
                    }
                } else {
                    perimeterPathRotated.push(...cleanPassPoints);
                }
            }
        }

        if (!spiralMode && !reverseSpiral) {
            perimeterPathRotated = removeSharpPeaks(
                perimeterPathRotated,
                isFreePermissiveRotated,
            );
        }
        perimeterPathRotated = condenseCollinearPoints(
            perimeterPathRotated,
            Math.max(0.01, combineWaypoints),
            isFreePermissiveRotated,
        );

        // Interior obstacle headland passes
        if (computedNPasses > 0 && !spiralMode) {
            const interiorPolys = [];
            for (let eIdx = 0; eIdx < exR.length; eIdx++) {
                if (isIntersecting[eIdx]) continue;
                const exPoly = exR[eIdx];
                if (exPoly.length >= 3 && pointInPoly(exPoly[0], perimOuterR)) {
                    interiorPolys.push(exPoly);
                }
            }

            const mergedLoops = unionPolygons(interiorPolys);

            for (const mergedPoly of mergedLoops) {
                if (mergedPoly.length < 3) continue;
                const ccw = isPolygonCW(mergedPoly)
                    ? mergedPoly.slice().reverse()
                    : mergedPoly.slice();
                const lastPt =
                    perimeterPathRotated.length > 0
                        ? perimeterPathRotated.at(-1)
                        : ccw[0];
                let bestIdx = 0;
                let bestD = Number.POSITIVE_INFINITY;
                for (let k = 0; k < ccw.length; k++) {
                    const d = Math.hypot(
                        ccw[k][0] - lastPt[0],
                        ccw[k][1] - lastPt[1],
                    );
                    if (d < bestD) {
                        bestD = d;
                        bestIdx = k;
                    }
                }
                const loop = [
                    ...ccw.slice(bestIdx),
                    ...ccw.slice(0, bestIdx),
                    ccw[bestIdx],
                ];
                const sampled = [];
                for (let k = 0; k < loop.length - 1; k++) {
                    const p1 = loop[k];
                    const p2 = loop[k + 1];
                    const len = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                    const steps = Math.max(1, Math.round(len / 0.5));
                    for (let s = 0; s < steps; s++) {
                        const t = s / steps;
                        sampled.push([
                            p1[0] + t * (p2[0] - p1[0]),
                            p1[1] + t * (p2[1] - p1[1]),
                        ]);
                    }
                }
                sampled.push(loop.at(-1));

                // Find a free starting point in sampled
                let startSample = 0;
                while (
                    startSample < sampled.length &&
                    (!pointInPoly(
                        sampled[startSample],
                        perimOuterR_tolerance,
                    ) ||
                        exR_tolerance.some((ex) =>
                            pointInPoly(sampled[startSample], ex),
                        ))
                ) {
                    startSample++;
                }
                if (startSample >= sampled.length) continue;

                const reorderedSampled = [
                    ...sampled.slice(startSample),
                    ...sampled.slice(0, startSample),
                ];

                // Validate each leg through segmentFree
                let curr = 0;
                let next = 1;
                const routedLoop = [reorderedSampled[0]];
                while (next < reorderedSampled.length) {
                    const p1 = reorderedSampled[curr];
                    const p2 = reorderedSampled[next];
                    if (
                        !pointInPoly(p2, perimOuterR_tolerance) ||
                        exR_tolerance.some((ex) => pointInPoly(p2, ex))
                    ) {
                        next++;
                        continue;
                    }
                    if (Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.01) {
                        next++;
                    } else if (
                        segmentFree(
                            p1,
                            p2,
                            perimOuterR,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        routedLoop.push(p2);
                        curr = next;
                        next++;
                    } else {
                        const transit = routeTransition(
                            p1,
                            p2,
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        if (transit) {
                            routedLoop.push(...transit.slice(1));
                            curr = next;
                            next++;
                        } else {
                            next++;
                        }
                    }
                }

                let isClosed = false;
                if (routedLoop.length >= 3) {
                    const last = routedLoop.at(-1);
                    const first = routedLoop[0];
                    if (
                        Math.hypot(first[0] - last[0], first[1] - last[1]) <
                        0.05
                    ) {
                        isClosed = true;
                    } else if (
                        segmentFree(
                            last,
                            first,
                            perimOuterR_expanded,
                            exR_tolerance,
                            exRC_tolerance,
                            null,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        routedLoop.push(first);
                        isClosed = true;
                    } else {
                        const closeTransit = routeTransition(
                            last,
                            first,
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        if (closeTransit) {
                            routedLoop.push(...closeTransit.slice(1));
                            isClosed = true;
                        }
                    }
                }

                const cleanLoop = removeLoopsAndBacktracks(
                    routedLoop,
                    Math.min(0.2, laneWidth * 0.3),
                    isFreePermissiveRotated,
                );
                if (cleanLoop.length < 3) continue;

                if (perimeterPathRotated.length > 0) {
                    if (isClosed) {
                        let bestTransit = null;
                        let bestIdx = 0;
                        let bestDist = Number.POSITIVE_INFINITY;
                        for (let ptIdx = 0; ptIdx < cleanLoop.length; ptIdx++) {
                            const d = Math.hypot(
                                cleanLoop[ptIdx][0] - lastPt[0],
                                cleanLoop[ptIdx][1] - lastPt[1],
                            );
                            if (d < bestDist) {
                                const tr = routeTransition(
                                    lastPt,
                                    cleanLoop[ptIdx],
                                    roadmap,
                                    visGraph,
                                    perimOuterR,
                                    perimOuterR_tolerance,
                                    exR_tolerance,
                                    exRC_tolerance,
                                    perimBBox,
                                    perimBBox_tolerance,
                                    exBBoxes_tolerance,
                                    exCBBoxes_tolerance,
                                );
                                if (tr) {
                                    bestTransit = tr;
                                    bestIdx = ptIdx;
                                    bestDist = d;
                                }
                            }
                        }
                        if (bestTransit) {
                            perimeterPathRotated.push(...bestTransit.slice(1));
                            const N = cleanLoop.length;
                            const reordered = [
                                ...cleanLoop.slice(bestIdx, N - 1),
                                ...cleanLoop.slice(0, bestIdx),
                                cleanLoop[bestIdx],
                            ];
                            perimeterPathRotated.push(...reordered.slice(1));
                        }
                    } else {
                        const transitFwd = routeTransition(
                            lastPt,
                            cleanLoop[0],
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        const transitRev = routeTransition(
                            lastPt,
                            cleanLoop.at(-1),
                            roadmap,
                            visGraph,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        const lenFwd = transitFwd
                            ? pathLength(transitFwd)
                            : Number.POSITIVE_INFINITY;
                        const lenRev = transitRev
                            ? pathLength(transitRev)
                            : Number.POSITIVE_INFINITY;
                        if (transitFwd && lenFwd <= lenRev) {
                            perimeterPathRotated.push(...transitFwd.slice(1));
                            perimeterPathRotated.push(...cleanLoop.slice(1));
                        } else if (transitRev) {
                            perimeterPathRotated.push(...transitRev.slice(1));
                            const rev = cleanLoop.slice().reverse();
                            perimeterPathRotated.push(...rev.slice(1));
                        }
                    }
                } else {
                    perimeterPathRotated.push(...cleanLoop);
                }
            }
        }
    }

    // §4.3 Build grid coverage map (Eq. 7) and generate boustrophedon strips
    const gridPolys = offlineFillPolysR
        ? perimR.concat(offlineFillPolysR)
        : perimR;
    const gridData = spiralMode ? null : buildGrid(gridPolys, exR, laneWidth);
    const strips = spiralMode
        ? []
        : generateStrips(
              gridData,
              laneWidth,
              perimR,
              exR,
              navigablePerimeterRotated,
              offlineFillPolysR,
              offlineExpandedPolysR,
          );

    // Get final perimeter passes in original space (safely pushed outside obstacles)
    let safePerimeterPath = [];
    if (computedNPasses > 0) {
        safePerimeterPath = rotPts(perimeterPathRotated, angle);
    }

    if (!strips.length) {
        if (computedNPasses > 0) {
            let finalPathMetric = condenseCollinearPoints(
                safePerimeterPath,
                combineWaypoints,
                isFreePermissiveMetric,
            );

            if (spiralMode && finalPathMetric.length >= 3) {
                const smoothedSpiral = [finalPathMetric[0]];
                for (let i = 1; i < finalPathMetric.length - 1; i++) {
                    const prev = smoothedSpiral.at(-1);
                    const curr = finalPathMetric[i];
                    const next = finalPathMetric[i + 1];
                    const d1 = [curr[0] - prev[0], curr[1] - prev[1]];
                    const d2 = [next[0] - curr[0], next[1] - curr[1]];
                    const len1 = Math.hypot(d1[0], d1[1]);
                    const len2 = Math.hypot(d2[0], d2[1]);
                    if (len1 > 1e-6 && len2 > 1e-6) {
                        const cosAngle =
                            (d1[0] * d2[0] + d1[1] * d2[1]) / (len1 * len2);
                        if (
                            cosAngle < -0.9 &&
                            Math.min(len1, len2) < 0.15 &&
                            (!isFreePermissiveMetric ||
                                isFreePermissiveMetric(prev, next))
                        ) {
                            continue;
                        }
                    }
                    smoothedSpiral.push(curr);
                }
                smoothedSpiral.push(finalPathMetric.at(-1));
                finalPathMetric = smoothedSpiral;
            }

            const dedupedMetric = [];
            for (let i = 0; i < finalPathMetric.length; i++) {
                if (
                    dedupedMetric.length === 0 ||
                    Math.hypot(
                        finalPathMetric[i][0] - dedupedMetric.at(-1)[0],
                        finalPathMetric[i][1] - dedupedMetric.at(-1)[1],
                    ) >= 0.005
                ) {
                    dedupedMetric.push(finalPathMetric[i]);
                }
            }
            finalPathMetric = dedupedMetric;

            const path = finalPathMetric.map(([x, y]) =>
                fromLocal(x, y, oLat, oLon),
            );

            // Compute total distance
            let totalDistM = 0;
            for (let i = 0; i < finalPathMetric.length - 1; i++) {
                totalDistM += Math.hypot(
                    finalPathMetric[i + 1][0] - finalPathMetric[i][0],
                    finalPathMetric[i + 1][1] - finalPathMetric[i][1],
                );
            }

            // Compute sweep distance (perimeter passes + boustrophedon sweep legs)
            let perimeterDistM = 0;
            if (perimeterPathRotated.length > 1) {
                for (let i = 0; i < perimeterPathRotated.length - 1; i++) {
                    perimeterDistM += Math.hypot(
                        perimeterPathRotated[i + 1][0] -
                            perimeterPathRotated[i][0],
                        perimeterPathRotated[i + 1][1] -
                            perimeterPathRotated[i][1],
                    );
                }
            }
            const sweepDistM = perimeterDistM;

            // Compute covered area
            let coveredAreaSqM = getPolygonArea(navigablePerimeter);
            exclusions.forEach((_s, idx) => {
                if (!isIntersecting[idx]) {
                    coveredAreaSqM -= getPolygonArea(exPolys[idx]);
                }
            });
            coveredAreaSqM = Math.max(0, coveredAreaSqM);

            return {
                path,
                count: path.length,
                totalDistM,
                sweepDistM,
                coveredAreaSqM,
                warnings,
            };
        }
        return null;
    }

    // §4.4.2 Connect strips using greedy backtracking based on true transition distance and skipLanes.
    // This resolves the split-row obstacle backtracking inefficiency while supporting skip-lane turns.
    const passesCount = skipLanes + 1;
    const passStrips = Array.from({ length: passesCount }, () => []);
    for (const strip of strips) {
        passStrips[strip.r % passesCount].push(strip);
    }

    const fullPath = [];
    let currentEnd = null;

    for (let p = 0; p < passesCount; p++) {
        const unvisited = new Set(passStrips[p]);
        if (unvisited.size === 0) continue;

        let firstStrip = null;
        if (currentEnd === null) {
            firstStrip = passStrips[p][0];
            fullPath.push(firstStrip.start, firstStrip.end);
            currentEnd = firstStrip.end;
            unvisited.delete(firstStrip);
        } else {
            let bestCost = Number.POSITIVE_INFINITY;
            let bestOrient = null;
            let bestTransit = null;
            for (const strip of unvisited) {
                const orients = [
                    { start: strip.start, end: strip.end },
                    { start: strip.end, end: strip.start },
                ];
                for (const orient of orients) {
                    const dEuc = Math.hypot(
                        currentEnd[0] - orient.start[0],
                        currentEnd[1] - orient.start[1],
                    );
                    if (dEuc >= bestCost) continue;

                    const transit = routeTransition(
                        currentEnd,
                        orient.start,
                        roadmap,
                        visGraph,
                        perimOuterR,
                        perimOuterR_tolerance,
                        exR_tolerance,
                        exRC_tolerance,
                        perimBBox,
                        perimBBox_tolerance,
                        exBBoxes_tolerance,
                        exCBBoxes_tolerance,
                    );
                    if (!transit) continue;
                    const cost = pathLength(transit);
                    if (cost < bestCost) {
                        bestCost = cost;
                        firstStrip = strip;
                        bestOrient = orient;
                        bestTransit = transit;
                    }
                }
            }
            if (firstStrip && bestTransit) {
                unvisited.delete(firstStrip);
                fullPath.push(...bestTransit.slice(1));
                fullPath.push(bestOrient.end);
                currentEnd = bestOrient.end;
            }
        }

        while (unvisited.size > 0) {
            let bestCost = Number.POSITIVE_INFINITY;
            let bestStrip = null;
            let bestOrient = null;
            let bestTransit = null;

            for (const strip of unvisited) {
                const orients = [
                    { start: strip.start, end: strip.end },
                    { start: strip.end, end: strip.start },
                ];

                for (const orient of orients) {
                    const dEuc = Math.hypot(
                        currentEnd[0] - orient.start[0],
                        currentEnd[1] - orient.start[1],
                    );
                    if (dEuc >= bestCost) continue;

                    const transit = routeTransition(
                        currentEnd,
                        orient.start,
                        roadmap,
                        visGraph,
                        perimOuterR,
                        perimOuterR_tolerance,
                        exR_tolerance,
                        exRC_tolerance,
                        perimBBox,
                        perimBBox_tolerance,
                        exBBoxes_tolerance,
                        exCBBoxes_tolerance,
                    );
                    if (!transit) continue;
                    const cost = pathLength(transit);

                    if (cost < bestCost) {
                        bestCost = cost;
                        bestStrip = strip;
                        bestOrient = orient;
                        bestTransit = transit;
                    }
                }
            }

            if (!bestStrip || !bestTransit) {
                if (unvisited.size > 0) {
                    warnings.push(
                        `Unreachable strips dropped: ${unvisited.size}`,
                    );
                }
                break;
            }

            unvisited.delete(bestStrip);
            fullPath.push(...bestTransit.slice(1));
            fullPath.push(bestOrient.end);
            currentEnd = bestOrient.end;
        }
    }

    // Rotate back to original frame and convert to lat/lon
    let finalPathMetric = [];
    if (computedNPasses > 0 && fullPath.length > 0) {
        // Transition smoothly from end of perimeter passes to start of boustrophedon sweep
        const startTransitRotated = perimeterPathRotated.at(-1);
        const endTransitRotated = fullPath[0];
        const transitRotated = routeTransition(
            startTransitRotated,
            endTransitRotated,
            roadmap,
            visGraph,
            perimOuterR,
            perimOuterR_tolerance,
            exR_tolerance,
            exRC_tolerance,
            perimBBox,
            perimBBox_tolerance,
            exBBoxes_tolerance,
            exCBBoxes_tolerance,
        );
        const boustrophedonPath = rotPts(fullPath, angle);
        if (transitRotated) {
            const transit = rotPts(transitRotated, angle);
            finalPathMetric = [
                ...safePerimeterPath,
                ...transit.slice(1),
                ...boustrophedonPath.slice(1),
            ];
        } else {
            let altTransit = null;
            let altIdx = 0;
            for (let i = 1; i < Math.min(fullPath.length, 10); i++) {
                const tr = routeTransition(
                    startTransitRotated,
                    fullPath[i],
                    roadmap,
                    visGraph,
                    perimOuterR,
                    perimOuterR_tolerance,
                    exR_tolerance,
                    exRC_tolerance,
                    perimBBox,
                    perimBBox_tolerance,
                    exBBoxes_tolerance,
                    exCBBoxes_tolerance,
                );
                if (tr) {
                    altTransit = tr;
                    altIdx = i;
                    break;
                }
            }
            if (altTransit) {
                const transit = rotPts(altTransit, angle);
                finalPathMetric = [
                    ...safePerimeterPath,
                    ...transit.slice(1),
                    ...boustrophedonPath.slice(altIdx + 1),
                ];
                warnings.push(
                    `Could not route transition to initial sweep strip; skipped first ${altIdx} strip point(s)`,
                );
            } else {
                warnings.push(
                    "Could not route transition to sweep strips; unreachable strips dropped",
                );
                finalPathMetric = safePerimeterPath;
            }
        }
    } else if (computedNPasses > 0) {
        finalPathMetric = safePerimeterPath;
    } else {
        finalPathMetric = rotPts(fullPath, angle);
    }

    finalPathMetric = condenseCollinearPoints(
        finalPathMetric,
        Math.min(combineWaypoints, laneWidth * 0.45),
        isFreePermissiveMetric,
    );

    const dedupedMetric = [];
    for (let i = 0; i < finalPathMetric.length; i++) {
        if (
            dedupedMetric.length === 0 ||
            Math.hypot(
                finalPathMetric[i][0] - dedupedMetric.at(-1)[0],
                finalPathMetric[i][1] - dedupedMetric.at(-1)[1],
            ) >= 0.005
        ) {
            dedupedMetric.push(finalPathMetric[i]);
        }
    }
    finalPathMetric = dedupedMetric;

    const path = finalPathMetric.map(([x, y]) => fromLocal(x, y, oLat, oLon));

    // Compute total distance
    let totalDistM = 0;
    for (let i = 0; i < finalPathMetric.length - 1; i++) {
        totalDistM += Math.hypot(
            finalPathMetric[i + 1][0] - finalPathMetric[i][0],
            finalPathMetric[i + 1][1] - finalPathMetric[i][1],
        );
    }

    // Compute sweep distance (perimeter passes + boustrophedon sweep legs)
    let perimeterDistM = 0;
    if (computedNPasses > 0 && perimeterPathRotated.length > 1) {
        for (let i = 0; i < perimeterPathRotated.length - 1; i++) {
            perimeterDistM += Math.hypot(
                perimeterPathRotated[i + 1][0] - perimeterPathRotated[i][0],
                perimeterPathRotated[i + 1][1] - perimeterPathRotated[i][1],
            );
        }
    }
    const sweepDistM =
        perimeterDistM +
        strips.reduce(
            (sum, s) =>
                sum + Math.hypot(s.end[0] - s.start[0], s.end[1] - s.start[1]),
            0,
        );

    // Compute covered area
    let coveredAreaSqM = getPolygonArea(navigablePerimeter);
    exclusions.forEach((_s, idx) => {
        if (!isIntersecting[idx]) {
            coveredAreaSqM -= getPolygonArea(exPolys[idx]);
        }
    });
    coveredAreaSqM = Math.max(0, coveredAreaSqM);

    const baseRes = {
        path,
        count: path.length,
        totalDistM,
        sweepDistM,
        coveredAreaSqM,
        warnings,
    };
    if (!isPiece) {
        baseRes.pieces = [baseRes];
    }
    return baseRes;
}

window.generateCoveragePath = generateCoveragePath;
