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
        const area =
            (Math.max(...xs) - Math.min(...xs)) *
            (Math.max(...ys) - Math.min(...ys));
        if (area < minArea) {
            minArea = area;
            best = a;
        }
    }
    return best;
}

// ── Polygon / segment geometry ─────────────────────────────

function ccw3(A, B, C) {
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0]);
}

function segIntersect(A, B, C, D) {
    return ccw3(A, C, D) !== ccw3(B, C, D) && ccw3(A, B, C) !== ccw3(A, B, D);
}

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

        if (segIntersect(a, b, p1, p2)) return true;
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
    for (const t of [0.25, 0.5, 0.75]) {
        const p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        if (!pointInPoly(p, perim)) return false;
        for (let i = 0; i < exclC.length; i++) {
            if (pointInPoly(p, exclC[i])) return false;
        }
    }
    return true;
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

function insetPoly(poly, dist) {
    if (poly.length < 3) return poly.slice();
    if (dist <= 0) return poly.slice();

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
            } else {
                return [];
            }
        } catch (e) {
            console.warn(
                "Clipper inset failed, falling back to miter-offset:",
                e,
            );
        }
    }

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

        // Cap miter length to prevent extreme spikes at sharp corners
        let L = d;
        if (Math.abs(cosHalfAngle) > 0.1) {
            L = d / cosHalfAngle;
            // Limit to 4 times the shift distance to avoid huge spikes
            if (Math.abs(L) > Math.abs(d) * 4) {
                L = Math.sign(L) * Math.abs(d) * 4;
            }
        } else {
            L = d * 4;
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

function makeCW(poly) {
    return isPolygonCW(poly) ? poly.slice() : poly.slice().reverse();
}

function subtractPolygon(P_orig, E_orig) {
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
        return P_orig; // E is completely inside P or disjoint
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

    // Find a start node in flatP that is outside E
    let startNode = null;
    for (const node of flatP) {
        if (!node.isIntersection && !pointInPoly(node.pt, E)) {
            startNode = node;
            break;
        }
    }

    if (!startNode) return []; // P is entirely inside E

    const visited = new Set();
    const resultPts = [];
    let currList = flatP;
    let currIdx = startNode.flatIdx;
    let dir = 1;

    const startIdx = currIdx;
    let loopCount = 0;
    const maxLoops = flatP.length + flatE.length + 100;

    while (loopCount < maxLoops) {
        const node = currList[currIdx];
        resultPts.push(node.pt);

        if (node.isIntersection) {
            if (visited.has(node)) {
                break;
            }
            visited.add(node);
            visited.add(node.partner);

            if (currList === flatP) {
                currList = flatE;
                currIdx = node.partner.flatIdx;
                dir = -1; // walk E CCW
            } else {
                currList = flatP;
                currIdx = node.partner.flatIdx;
                dir = 1; // walk P CW
            }
        }

        currIdx = (currIdx + dir + currList.length) % currList.length;
        if (currList === flatP && currIdx === startIdx) {
            break;
        }
        loopCount++;
    }

    // Clean up result: remove duplicate consecutive points
    const cleaned = [];
    for (const pt of resultPts) {
        if (cleaned.length === 0) {
            cleaned.push(pt);
        } else {
            const last = cleaned[cleaned.length - 1];
            if (Math.hypot(pt[0] - last[0], pt[1] - last[1]) > 1e-5) {
                cleaned.push(pt);
            }
        }
    }
    // Check if last point is close to first, if so pop it to avoid duplicate close
    if (cleaned.length > 1) {
        const first = cleaned[0];
        const last = cleaned[cleaned.length - 1];
        if (Math.hypot(first[0] - last[0], first[1] - last[1]) < 1e-5) {
            cleaned.pop();
        }
    }
    return cleaned;
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

function buildGrid(perimR, exR, cellSize) {
    const xs = perimR.map((p) => p[0]);
    const ys = perimR.map((p) => p[1]);
    const xMin = Math.min(...xs);
    const yMin = Math.min(...ys);
    const cols = Math.ceil((Math.max(...xs) - xMin) / cellSize);
    const rows = Math.ceil((Math.max(...ys) - yMin) / cellSize);
    const grid = Array.from({ length: rows }, (_, r) =>
        Array.from({ length: cols }, (_, c) => {
            const cx = xMin + (c + 0.5) * cellSize;
            const cy = yMin + (r + 0.5) * cellSize;
            if (!pointInPoly([cx, cy], perimR)) return 1;
            for (const ex of exR) if (pointInPoly([cx, cy], ex)) return 1;
            return 0; // unvisited
        }),
    );
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
) {
    const strips = [];
    let ltr = true; // start left-to-right from the corner of the longest edge

    for (let r = 0; r < rows; r++) {
        const cy = yMin + (r + 0.5) * cellSize;

        // Exact perimeter intersections at this y-level; inset EPS_PERIM so that
        // strip endpoints are never exactly on the perimeter boundary.
        const pxs = scanXs(perimR, cy);
        if (pxs.length < 2) continue;

        // Build free x-intervals: start from (slightly inset) perimeter segments,
        // subtract each exclusion zone expanded by EPS_EXCL.
        let segs = [];
        for (let i = 0; i + 1 < pxs.length; i += 2)
            segs.push([pxs[i] + EPS_PERIM, pxs[i + 1] - EPS_PERIM]);
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
                if (grid[r][c] === 0) grid[r][c] = 0.5;
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
        const maxCandidates = N_KNN * 4;

        for (const j of freeIdxs) {
            if (j === i) continue;
            const d = Math.hypot(
                verts[j][0] - target[0],
                verts[j][1] - target[1],
            );

            if (neighbors.length < maxCandidates) {
                neighbors.push({ j, d });
                neighbors.sort((a, b) => a.d - b.d);
            } else if (d < neighbors[maxCandidates - 1].d) {
                neighbors[maxCandidates - 1] = { j, d };
                neighbors.sort((a, b) => a.d - b.d);
            }
        }

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

    return { verts, adj };
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

// Route from → to via the Voronoi roadmap + Dijkstra (Eq. 10, §4.4.2):
//   lse = ‖p(s)−p(e)‖  if collision-free,
//         lse_Voronoi   otherwise.
// from and to are wired into the static roadmap as temporary query nodes.
function routeTransition(
    from,
    to,
    roadmap,
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

    const { verts: sv, adj: sa } = roadmap;
    const ns = sv.length;
    if (ns === 0) return [from, to]; // empty roadmap — fall back to direct

    // Augment roadmap: from=0, to=1; prebuilt roadmap nodes start at index 2
    const pts = [from, to, ...sv];
    const n = pts.length;
    const adj = [[], []];
    for (let i = 0; i < ns; i++)
        adj.push(sa[i].map(({ v, w }) => ({ v: v + 2, w })));

    // Wire from (0) and to (1) to their N_KNN nearest roadmap nodes
    for (const qi of [0, 1]) {
        const target = pts[qi];
        const dists = [];
        const maxCandidates = N_KNN * 4;

        for (let i = 0; i < ns; i++) {
            const d = Math.hypot(sv[i][0] - target[0], sv[i][1] - target[1]);
            if (dists.length < maxCandidates) {
                dists.push({ i, d });
                dists.sort((a, b) => a.d - b.d);
            } else if (d < dists[maxCandidates - 1].d) {
                dists[maxCandidates - 1] = { i, d };
                dists.sort((a, b) => a.d - b.d);
            }
        }

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
    }

    const idxPath = dijkstra(adj, 0, 1, n);
    return idxPath ? idxPath.map((i) => pts[i]) : [from, to]; // fallback: direct
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

function pushPointOutsideExclusions(pt, exData) {
    let currentPt = [pt[0], pt[1]];
    for (const ex of exData) {
        if (ex.type === "circle") {
            const dx = currentPt[0] - ex.cx;
            const dy = currentPt[1] - ex.cy;
            const dist = Math.hypot(dx, dy);
            if (dist < ex.r) {
                if (dist < 1e-9) {
                    currentPt = [ex.cx + ex.r, ex.cy];
                } else {
                    currentPt = [
                        ex.cx + (dx / dist) * ex.r,
                        ex.cy + (dy / dist) * ex.r,
                    ];
                }
            }
        } else {
            if (pointInPoly(currentPt, ex.poly)) {
                currentPt = getClosestPointOnPoly(currentPt, ex.poly);
            }
        }
    }
    return currentPt;
}

function removeSharpPeaks(pts) {
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
                result.splice(i, 1);
                changed = true;
                break;
            }
        }
        iterations++;
    }
    return result;
}

function condenseCollinearPoints(pts, tolerance = 0.01) {
    if (pts.length < 3) return pts.slice();
    const result = [pts[0]];
    for (let i = 1; i < pts.length - 1; i++) {
        const prev = result[result.length - 1];
        const curr = pts[i];
        const next = pts[i + 1];

        const cp = getClosestPointOnSegment(curr, prev, next);
        const dist = Math.hypot(curr[0] - cp[0], curr[1] - cp[1]);
        if (dist > tolerance) {
            result.push(curr);
        }
    }
    result.push(pts[pts.length - 1]);
    return result;
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
) {
    if (perimCoords.length < 3) return null;

    const oLat = perimCoords.reduce((s, c) => s + c[0], 0) / perimCoords.length;
    const oLon = perimCoords.reduce((s, c) => s + c[1], 0) / perimCoords.length;
    const xy = ([lat, lon]) => toLocal(lat, lon, oLat, oLon);

    // Project perimeter; drop closing duplicate if present
    const perim = perimCoords.map(xy);
    if (perim.length > 1) {
        const [fx, fy] = perim[0];
        const [lx, ly] = perim.at(-1);
        if (Math.hypot(fx - lx, fy - ly) < 0.01) perim.pop();
    }

    // Project + buffer each exclusion zone
    const exPolys = exclusions.map((s) => {
        if (s.type === "circle") {
            const [cx, cy] = toLocal(s.lat, s.lon, oLat, oLon);
            return circlePoly(cx, cy, s.radius + buffer, circleSegments);
        }
        const poly = s.vertices.map(xy);
        return buffer > 0 ? expandPoly(poly, buffer) : poly;
    });

    // Subtract intersecting exclusion zones from perimeter in local metric space
    let navigablePerimeter = perim;
    const isIntersecting = [];
    for (const ex of exPolys) {
        const nextPoly = subtractPolygon(navigablePerimeter, ex);
        const areaBefore = getPolygonArea(navigablePerimeter);
        const areaAfter = getPolygonArea(nextPoly);
        const changed = Math.abs(areaBefore - areaAfter) > 0.01;
        isIntersecting.push(changed);
        navigablePerimeter = nextPoly;
    }
    if (navigablePerimeter.length < 3) {
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

    let perimBoustrophedonOrig = navigablePerimeter;
    if (computedNPasses > 0) {
        perimBoustrophedonOrig = insetPoly(
            navigablePerimeter,
            (computedNPasses - 0.5) * laneWidth,
        );
    }

    // §4.2 Optimal sweep direction from MBB of convex hull (Algorithm 1) or forced sweep direction
    let angle;
    if (sweepMode === "auto") {
        angle = mbbSweepAngle(convexHull(perimBoustrophedonOrig));
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
    const perimBBox_tolerance = getBBox(perimOuterR_tolerance);

    // In sweep space, the outer perimeter already has exclusions subtracted
    const navigablePerimeterRotated = perimOuterR;

    let perimR;
    if (computedNPasses > 0) {
        perimR = rotPts(perimBoustrophedonOrig, -angle);
    } else {
        perimR = perimOuterR;
    }

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

    // Generate perimeter passes in rotated space for clean collision checking
    let perimeterPathRotated = [];
    let hasGeneratedReversedLap = false;
    if (computedNPasses > 0) {
        const poly = [...navigablePerimeterRotated];
        if (isPolygonCW(poly) !== (direction === "CW")) {
            poly.reverse();
        }
        const M = poly.length;

        // Compute unit miter offset vectors for each original vertex
        const origShiftVectors = [];
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

        const yardInsetLaps = [];
        for (let p = 0; p < computedNPasses; p++) {
            yardInsetLaps.push(insetPoly(perimOuterR, p * laneWidth));
        }

        const spacing = Math.max(laneWidth / 4, 0.5);

        const isPointFree = (pt) => {
            for (const ex of exR_tolerance) {
                if (pointInPoly(pt, ex)) return false;
            }
            return (
                perimOuterR_tolerance.length < 3 ||
                pointInPoly(pt, perimOuterR_tolerance)
            );
        };

        const exDataConstant = exclusions.map((s, idx) => {
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

        for (let p = 0; p < computedNPasses; p++) {
            const passPoints = [];
            const isReversedPass = spiralMode && reverseSpiral && p >= nPasses;

            if (isReversedPass) {
                const shift = nPasses > 0 ? 1 : 0;
                // CCW traversal: poly[M-1] -> poly[M-2] -> ... -> poly[0] -> poly[M-1]
                for (let i = M - 1; i >= 0; i--) {
                    const p1 = poly[i];
                    const p2 = poly[(i - 1 + M) % M];
                    const sv1 = origShiftVectors[i];
                    const sv2 = origShiftVectors[(i - 1 + M) % M];

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

                        if (hasGeneratedReversedLap && i === M - 1) {
                            dist = (p - shift - 1 + k / n) * laneWidth;
                            currentYardInset = insetPoly(perimOuterR, dist);
                        } else {
                            dist = (p - shift) * laneWidth;
                            currentYardInset =
                                yardInsetLaps[Math.max(0, p - shift)];
                        }

                        const offset = [
                            pt[0] + sv[0] * dist,
                            pt[1] + sv[1] * dist,
                        ];

                        let finalPt = offset;
                        if (!pointInPoly(finalPt, currentYardInset)) {
                            if (
                                currentYardInset &&
                                currentYardInset.length > 0
                            ) {
                                finalPt = getClosestPointOnPoly(
                                    finalPt,
                                    currentYardInset,
                                );
                            } else if (yardInsetLaps.length > 0) {
                                finalPt = getClosestPointOnPoly(
                                    finalPt,
                                    yardInsetLaps[yardInsetLaps.length - 1],
                                );
                            }
                        }
                        passPoints.push(finalPt);
                    }
                }
                // Push the final point poly[M-1] to complete the loop
                {
                    const pt = poly[M - 1];
                    const sv = origShiftVectors[M - 1];
                    const dist = (p - shift) * laneWidth;
                    const offset = [pt[0] + sv[0] * dist, pt[1] + sv[1] * dist];
                    let finalPt = offset;
                    const targetLapIdx = Math.max(0, p - shift);
                    if (!pointInPoly(finalPt, yardInsetLaps[targetLapIdx])) {
                        if (
                            yardInsetLaps[targetLapIdx] &&
                            yardInsetLaps[targetLapIdx].length > 0
                        ) {
                            finalPt = getClosestPointOnPoly(
                                finalPt,
                                yardInsetLaps[targetLapIdx],
                            );
                        } else if (yardInsetLaps.length > 0) {
                            finalPt = getClosestPointOnPoly(
                                finalPt,
                                yardInsetLaps[yardInsetLaps.length - 1],
                            );
                        }
                    }
                    passPoints.push(finalPt);
                }
            } else {
                // CW traversal: poly[0] -> poly[1] -> ... -> poly[M-1]
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
                            dist = (p - 1 + k / n) * laneWidth;
                            currentYardInset = insetPoly(perimOuterR, dist);
                        } else {
                            dist = p * laneWidth;
                            currentYardInset = yardInsetLaps[p];
                        }

                        const offset = [
                            pt[0] + sv[0] * dist,
                            pt[1] + sv[1] * dist,
                        ];

                        let finalPt = offset;
                        if (!pointInPoly(finalPt, currentYardInset)) {
                            if (
                                currentYardInset &&
                                currentYardInset.length > 0
                            ) {
                                finalPt = getClosestPointOnPoly(
                                    finalPt,
                                    currentYardInset,
                                );
                            } else if (yardInsetLaps.length > 0) {
                                finalPt = getClosestPointOnPoly(
                                    finalPt,
                                    yardInsetLaps[yardInsetLaps.length - 1],
                                );
                            }
                        }
                        passPoints.push(finalPt);
                    }
                }

                // Close the loop for this pass
                {
                    const pt = poly[0];
                    const sv = origShiftVectors[0];
                    const dist = p * laneWidth;
                    const offset = [pt[0] + sv[0] * dist, pt[1] + sv[1] * dist];
                    let finalPt = offset;
                    if (!pointInPoly(finalPt, yardInsetLaps[p])) {
                        if (yardInsetLaps[p] && yardInsetLaps[p].length > 0) {
                            finalPt = getClosestPointOnPoly(
                                finalPt,
                                yardInsetLaps[p],
                            );
                        } else if (yardInsetLaps.length > 0) {
                            finalPt = getClosestPointOnPoly(
                                finalPt,
                                yardInsetLaps[yardInsetLaps.length - 1],
                            );
                        }
                    }
                    passPoints.push(finalPt);
                }
            }

            // Check if this pass has any free points
            const hasFreePoint = passPoints.some((pt) => isPointFree(pt));
            if (!hasFreePoint) {
                continue;
            }

            // Push all points of this pass outside exclusions
            const pushedPassPoints = passPoints.map((pt) =>
                pushPointOutsideExclusions(pt, exDataConstant),
            );

            // Connect pushedPassPoints using routeTransition around obstacles
            const routedPassPoints = [];
            if (pushedPassPoints.length > 0) {
                routedPassPoints.push(pushedPassPoints[0]);
                for (let idx = 0; idx < pushedPassPoints.length - 1; idx++) {
                    const from = pushedPassPoints[idx];
                    const to = pushedPassPoints[idx + 1];
                    const segDist = Math.hypot(
                        to[0] - from[0],
                        to[1] - from[1],
                    );
                    if (segDist < 0.05) {
                        routedPassPoints.push(to);
                    } else if (
                        segmentFree(
                            from,
                            to,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        routedPassPoints.push(to);
                    } else {
                        const transit = routeTransition(
                            from,
                            to,
                            roadmap,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        routedPassPoints.push(...transit.slice(1));
                    }
                }
            }

            // Append routedPassPoints to perimeterPathRotated
            if (routedPassPoints.length > 0) {
                if (isReversedPass) {
                    hasGeneratedReversedLap = true;
                }
                if (perimeterPathRotated.length > 0) {
                    // Transition from previous pass's end to current pass's start
                    const prevEnd = perimeterPathRotated.at(-1);
                    const currStart = routedPassPoints[0];
                    if (
                        segmentFree(
                            prevEnd,
                            currStart,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        )
                    ) {
                        perimeterPathRotated.push(currStart);
                    } else {
                        const transit = routeTransition(
                            prevEnd,
                            currStart,
                            roadmap,
                            perimOuterR,
                            perimOuterR_tolerance,
                            exR_tolerance,
                            exRC_tolerance,
                            perimBBox,
                            perimBBox_tolerance,
                            exBBoxes_tolerance,
                            exCBBoxes_tolerance,
                        );
                        perimeterPathRotated.push(...transit.slice(1));
                    }
                    perimeterPathRotated.push(...routedPassPoints.slice(1));
                } else {
                    perimeterPathRotated.push(...routedPassPoints);
                }
            }
        }
        if (!reverseSpiral) {
            perimeterPathRotated = removeSharpPeaks(perimeterPathRotated);
        }
        perimeterPathRotated = condenseCollinearPoints(
            perimeterPathRotated,
            0.01,
        );
    }

    // §4.3 Build grid coverage map (Eq. 7) and generate boustrophedon strips
    const gridData = spiralMode ? null : buildGrid(perimR, exR, laneWidth);
    const strips = spiralMode
        ? []
        : generateStrips(gridData, laneWidth, perimR, exR);

    // Get final perimeter passes in original space (safely pushed outside obstacles)
    let safePerimeterPath = [];
    if (computedNPasses > 0) {
        safePerimeterPath = rotPts(perimeterPathRotated, angle);
    }

    if (!strips.length) {
        if (computedNPasses > 0) {
            const finalPathMetric = safePerimeterPath;
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
                        perimOuterR,
                        perimOuterR_tolerance,
                        exR_tolerance,
                        exRC_tolerance,
                        perimBBox,
                        perimBBox_tolerance,
                        exBBoxes_tolerance,
                        exCBBoxes_tolerance,
                    );
                    const cost = pathLength(transit);
                    if (cost < bestCost) {
                        bestCost = cost;
                        firstStrip = strip;
                        bestOrient = orient;
                        bestTransit = transit;
                    }
                }
            }
            if (firstStrip) {
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
                        perimOuterR,
                        perimOuterR_tolerance,
                        exR_tolerance,
                        exRC_tolerance,
                        perimBBox,
                        perimBBox_tolerance,
                        exBBoxes_tolerance,
                        exCBBoxes_tolerance,
                    );
                    const cost = pathLength(transit);

                    if (cost < bestCost) {
                        bestCost = cost;
                        bestStrip = strip;
                        bestOrient = orient;
                        bestTransit = transit;
                    }
                }
            }

            if (!bestStrip) break;

            unvisited.delete(bestStrip);
            fullPath.push(...bestTransit.slice(1));
            fullPath.push(bestOrient.end);
            currentEnd = bestOrient.end;
        }
    }

    // Rotate back to original frame and convert to lat/lon
    let finalPathMetric = [];
    if (computedNPasses > 0) {
        // Transition smoothly from end of perimeter passes to start of boustrophedon sweep
        const startTransitRotated = perimeterPathRotated.at(-1);
        const endTransitRotated = fullPath[0];
        const transitRotated = routeTransition(
            startTransitRotated,
            endTransitRotated,
            roadmap,
            perimOuterR,
            perimOuterR_tolerance,
            exR_tolerance,
            exRC_tolerance,
            perimBBox,
            perimBBox_tolerance,
            exBBoxes_tolerance,
            exCBBoxes_tolerance,
        );
        const transit = rotPts(transitRotated, angle);

        const boustrophedonPath = rotPts(fullPath, angle);

        finalPathMetric = [
            ...safePerimeterPath,
            ...transit.slice(1),
            ...boustrophedonPath.slice(1),
        ];
    } else {
        finalPathMetric = rotPts(fullPath, angle);
    }

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

    return {
        path,
        count: path.length,
        totalDistM,
        sweepDistM,
        coveredAreaSqM,
    };
}

window.generateCoveragePath = generateCoveragePath;
