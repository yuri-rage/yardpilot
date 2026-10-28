// Test harness for yardpilot planner.js
// Usage: node harness.js [--noclipper] [--only=name] [--dump=name]
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

const PLANNER =
    process.env.PLANNER_PATH || path.resolve(__dirname, "../assets/planner.js");
const args = process.argv.slice(2);
const NOCLIP = args.includes("--noclipper");
const ONLY = (args.find((a) => a.startsWith("--only=")) || "").slice(7);
const DUMP = (args.find((a) => a.startsWith("--dump=")) || "").slice(7);

function loadPlanner(withClipper) {
    const ctx = { console, Math, Number, Array, Set, Map, window: {} };
    if (withClipper) ctx.ClipperLib = require("clipper-lib");
    vm.createContext(ctx);
    vm.runInContext(fs.readFileSync(PLANNER, "utf8"), ctx);
    return ctx;
}
const P = loadPlanner(!NOCLIP);

const O = [33.3, -96.8];
const toLL = ([x, y]) => P.fromLocal(x, y, O[0], O[1]);
const toXY = ([la, lo]) => P.toLocal(la, lo, O[0], O[1]);

// ── geometry helpers (independent of planner) ──
function pip([px, py], poly) {
    let c = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i],
            [xj, yj] = poly[j];
        if (
            yi > py !== yj > py &&
            px < ((xj - xi) * (py - yi)) / (yj - yi) + xi
        )
            c = !c;
    }
    return c;
}
function segDist(p, a, b) {
    const dx = b[0] - a[0],
        dy = b[1] - a[1];
    const L = dx * dx + dy * dy;
    let t = L ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L : 0;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}
function edgeDist(p, poly) {
    let m = Infinity;
    for (let i = 0; i < poly.length; i++)
        m = Math.min(m, segDist(p, poly[i], poly[(i + 1) % poly.length]));
    return m;
}
const circ = (cx, cy, r, n = 256) =>
    Array.from({ length: n }, (_, i) => [
        cx + r * Math.cos((2 * Math.PI * i) / n),
        cy + r * Math.sin((2 * Math.PI * i) / n),
    ]);
const rot = (pts, deg) => {
    const a = (deg * Math.PI) / 180,
        c = Math.cos(a),
        s = Math.sin(a);
    return pts.map(([x, y]) => [x * c - y * s, x * s + y * c]);
};

// ── shapes (metres) ──
const SHAPES = {
    rect: [
        [0, 0],
        [40, 0],
        [40, 25],
        [0, 25],
    ],
    rot23: rot(
        [
            [0, 0],
            [40, 0],
            [40, 25],
            [0, 25],
        ],
        23,
    ),
    L: [
        [0, 0],
        [40, 0],
        [40, 12],
        [15, 12],
        [15, 30],
        [0, 30],
    ],
    dumbbell: [
        [0, 0],
        [20, 0],
        [20, 8.5],
        [30, 8.5],
        [30, 0],
        [50, 0],
        [50, 20],
        [30, 20],
        [30, 11.5],
        [20, 11.5],
        [20, 20],
        [0, 20],
    ],
    sawtooth: [
        [0, 0],
        [10, 14],
        [20, 0],
        [30, 14],
        [40, 0],
        [40, 25],
        [0, 25],
    ],
    blob: Array.from({ length: 120 }, (_, i) => {
        const a = (2 * Math.PI * i) / 120;
        const r = 15 + 3 * Math.sin(3 * a) + 1.5 * Math.cos(5 * a);
        return [1.4 * r * Math.cos(a), r * Math.sin(a)];
    }),
    wedge: [
        [0, 0],
        [45, 0],
        [45, 4],
    ], // sharp 5° tip
    finger: [
        [0, 0],
        [40, 0],
        [40, 25],
        [21.5, 25],
        [21.5, 35],
        [18.5, 35],
        [18.5, 25],
        [0, 25],
    ], // 3 m wide, 10 m long strip of lawn
    notch: [
        [0, 0],
        [18.5, 0],
        [18.5, 8],
        [21.5, 8],
        [21.5, 0],
        [40, 0],
        [40, 25],
        [0, 25],
    ], // 3 m wide, 8 m deep inlet
};
// exclusions: {type:'circle', c:[x,y], r} or {type:'poly', v:[[x,y]...]}
const EXCL = {
    none: [],
    circ: [{ type: "circle", c: [20, 12.5], r: 2 }],
    edge: [
        {
            type: "poly",
            v: [
                [30, 18],
                [36, 18],
                [36, 28],
                [30, 28],
            ],
        },
    ],
    split: [
        {
            type: "poly",
            v: [
                [24, -3],
                [27, -3],
                [27, 28],
                [24, 28],
            ],
        },
    ],
    split_circ: [
        {
            type: "poly",
            v: [
                [24, -3],
                [27, -3],
                [27, 28],
                [24, 28],
            ],
        },
        { type: "circle", c: [20, 12.5], r: 2 },
    ],
    pair: [
        { type: "circle", c: [15, 12.5], r: 1.5 },
        { type: "circle", c: [19.2, 12.5], r: 1.5 },
    ],
};

function toPlannerExcl(ex) {
    return ex.map((e) =>
        e.type === "circle"
            ? {
                  type: "circle",
                  lat: toLL(e.c)[0],
                  lon: toLL(e.c)[1],
                  radius: e.r,
              }
            : { type: "polygon", vertices: e.v.map(toLL) },
    );
}
function exclPolys(ex, extra) {
    // geometric truth: circle -> fine polygon; poly -> raw (buffered poly = Minkowski approx via clipper)
    const CL = require("clipper-lib");
    return ex.map((e) => {
        if (e.type === "circle") return circ(e.c[0], e.c[1], e.r + extra);
        if (extra <= 0) return e.v;
        const co = new CL.ClipperOffset();
        co.AddPath(
            e.v.map(([x, y]) => ({
                X: Math.round(x * 1e4),
                Y: Math.round(y * 1e4),
            })),
            CL.JoinType.jtRound,
            CL.EndType.etClosedPolygon,
        );
        co.ArcTolerance = 10;
        const s = new CL.Paths();
        co.Execute(s, extra * 1e4);
        return s[0].map((p) => [p.X / 1e4, p.Y / 1e4]);
    });
}

// ── metrics ──
function analyze(perim, ex, opt, pathsInput) {
    const pathsXY = Array.isArray(pathsInput[0]?.[0])
        ? pathsInput
        : [pathsInput];
    const lane = opt.lane,
        buf = opt.buffer,
        tol = opt.tol;
    const exRaw = exclPolys(ex, 0);
    const exBuf = exclPolys(ex, buf);
    const exHard = exclPolys(ex, Math.max(0, buf - tol)); // what the planner promises to respect for transits

    // boundary excursions: sample along every segment
    let maxOut = 0,
        outLen = 0,
        intoRaw = 0,
        intoBufLen = 0,
        maxIntoBuf = 0;
    const badSegs = [];
    for (const pathXY of pathsXY) {
        for (let i = 1; i < pathXY.length; i++) {
            const a = pathXY[i - 1],
                b = pathXY[i];
            const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
            const n = Math.max(1, Math.ceil(L / 0.05));
            let segOut = 0,
                segRaw = false,
                segBuf = 0;
            for (let k = 0; k <= n; k++) {
                const p = [
                    a[0] + ((b[0] - a[0]) * k) / n,
                    a[1] + ((b[1] - a[1]) * k) / n,
                ];
                if (!pip(p, perim)) {
                    const d = edgeDist(p, perim);
                    segOut = Math.max(segOut, d);
                    if (d > 0.05) outLen += L / n;
                }
                for (let j = 0; j < ex.length; j++) {
                    if (pip(p, exRaw[j])) segRaw = true;
                    if (pip(p, exHard[j])) {
                        const d = edgeDist(p, exHard[j]);
                        segBuf = Math.max(segBuf, d);
                        if (d > 0.05) intoBufLen += L / n;
                    }
                }
            }
            maxOut = Math.max(maxOut, segOut);
            maxIntoBuf = Math.max(maxIntoBuf, segBuf);
            if (segRaw) intoRaw++;
            if (segOut > 0.05 || segRaw || segBuf > 0.05)
                badSegs.push({
                    i,
                    a,
                    b,
                    L: +L.toFixed(2),
                    out: +segOut.toFixed(2),
                    raw: segRaw,
                    buf: +segBuf.toFixed(2),
                });
        }
    }

    // coverage grid: target = inside perim, outside buffered exclusions
    const g = 0.05,
        r = lane / 2;
    const xs = perim.map((p) => p[0]),
        ys = perim.map((p) => p[1]);
    const x0 = Math.min(...xs),
        y0 = Math.min(...ys);
    const W = Math.ceil((Math.max(...xs) - x0) / g),
        H = Math.ceil((Math.max(...ys) - y0) / g);
    const target = new Uint8Array(W * H),
        cov = new Uint8Array(W * H);
    let nT = 0;
    for (let j = 0; j < H; j++)
        for (let i = 0; i < W; i++) {
            const p = [x0 + (i + 0.5) * g, y0 + (j + 0.5) * g];
            if (pip(p, perim) && !exBuf.some((e) => pip(p, e))) {
                target[j * W + i] = 1;
                nT++;
            }
        }
    for (const pathXY of pathsXY) {
        for (let s = 1; s < pathXY.length; s++) {
            const a = pathXY[s - 1],
                b = pathXY[s];
            const i0 = Math.max(
                0,
                Math.floor((Math.min(a[0], b[0]) - r - x0) / g),
            );
            const i1 = Math.min(
                W - 1,
                Math.ceil((Math.max(a[0], b[0]) + r - x0) / g),
            );
            const j0 = Math.max(
                0,
                Math.floor((Math.min(a[1], b[1]) - r - y0) / g),
            );
            const j1 = Math.min(
                H - 1,
                Math.ceil((Math.max(a[1], b[1]) + r - y0) / g),
            );
            for (let j = j0; j <= j1; j++)
                for (let i = i0; i <= i1; i++) {
                    const k = j * W + i;
                    if (cov[k]) continue;
                    if (
                        segDist(
                            [x0 + (i + 0.5) * g, y0 + (j + 0.5) * g],
                            a,
                            b,
                        ) <= r
                    )
                        cov[k] = 1;
                }
        }
    }
    let nU = 0;
    for (let k = 0; k < W * H; k++) if (target[k] && !cov[k]) nU++;
    // largest uncovered connected blob
    const seen = new Uint8Array(W * H);
    let maxBlob = 0,
        blobs = 0;
    const blobList = [];
    for (let k = 0; k < W * H; k++) {
        if (!target[k] || cov[k] || seen[k]) continue;
        let sz = 0,
            sx = 0,
            sy = 0;
        const st = [k];
        seen[k] = 1;
        while (st.length) {
            const c = st.pop();
            sz++;
            sx += c % W;
            sy += Math.floor(c / W);
            const ci = c % W,
                cj = Math.floor(c / W);
            for (const [di, dj] of [
                [1, 0],
                [-1, 0],
                [0, 1],
                [0, -1],
            ]) {
                const ni = ci + di,
                    nj = cj + dj;
                if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
                const nk = nj * W + ni;
                if (target[nk] && !cov[nk] && !seen[nk]) {
                    seen[nk] = 1;
                    st.push(nk);
                }
            }
        }
        if (sz * g * g >= 0.05) {
            blobs++;
            blobList.push({
                area: +(sz * g * g).toFixed(2),
                at: [
                    +(x0 + (sx / sz) * g).toFixed(1),
                    +(y0 + (sy / sz) * g).toFixed(1),
                ],
            });
        }
        maxBlob = Math.max(maxBlob, sz);
    }
    blobList.sort((a, b) => b.area - a.area);

    // clustering and waypoints across all paths
    const d = [];
    let clusters = 0;
    let reversals = 0;
    let nearDup = 0;
    let totalWp = 0;

    for (const pathXY of pathsXY) {
        totalWp += pathXY.length;
        for (let i = 1; i < pathXY.length; i++)
            d.push(
                Math.hypot(
                    pathXY[i][0] - pathXY[i - 1][0],
                    pathXY[i][1] - pathXY[i - 1][1],
                ),
            );

        // tight clusters: windows of >=4 consecutive waypoints spanning < lane/2
        for (let i = 0; i + 3 < pathXY.length; ) {
            let j = i + 1;
            while (
                j < pathXY.length &&
                Math.hypot(
                    pathXY[j][0] - pathXY[i][0],
                    pathXY[j][1] - pathXY[i][1],
                ) <
                    lane / 2
            )
                j++;
            if (j - i >= 4) {
                clusters++;
                i = j;
            } else i++;
        }
        // sharp reversals (>150°) on short legs
        for (let i = 1; i + 1 < pathXY.length; i++) {
            const v1 = [
                pathXY[i][0] - pathXY[i - 1][0],
                pathXY[i][1] - pathXY[i - 1][1],
            ];
            const v2 = [
                pathXY[i + 1][0] - pathXY[i][0],
                pathXY[i + 1][1] - pathXY[i][1],
            ];
            const l1 = Math.hypot(...v1),
                l2 = Math.hypot(...v2);
            if (l1 < 1e-6 || l2 < 1e-6) continue;
            if (
                (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2) <
                    Math.cos((150 * Math.PI) / 180) &&
                Math.min(l1, l2) < lane
            )
                reversals++;
        }

        // near-duplicate, non-adjacent waypoints
        {
            const h = new Map(),
                cs = 0.05;
            pathXY.forEach((p, i) => {
                const kx = Math.floor(p[0] / cs),
                    ky = Math.floor(p[1] / cs);
                for (let dx = -1; dx <= 1; dx++)
                    for (let dy = -1; dy <= 1; dy++)
                        for (const j of h.get(`${kx + dx},${ky + dy}`) || [])
                            if (
                                i - j >= 2 &&
                                Math.hypot(
                                    p[0] - pathXY[j][0],
                                    p[1] - pathXY[j][1],
                                ) < cs
                            )
                                nearDup++;
                const k = `${kx},${ky}`;
                if (!h.has(k)) h.set(k, []);
                h.get(k).push(i);
            });
        }
    }

    const lt = (t) => d.filter((x) => x < t).length;

    return {
        nearDupPairs: nearDup,
        wp: totalWp,
        coverPct: +((100 * (nT - nU)) / nT).toFixed(2),
        uncovered_m2: +(nU * g * g).toFixed(1),
        gaps: blobs,
        biggestGap_m2: +(maxBlob * g * g).toFixed(2),
        topGaps: blobList.slice(0, 3),
        maxOutsidePerim_m: +maxOut.toFixed(2),
        lenOutsidePerim_m: +outLen.toFixed(1),
        segsIntoRawExcl: intoRaw,
        maxIntoBuffer_m: +maxIntoBuf.toFixed(2),
        lenIntoBuffer_m: +intoBufLen.toFixed(1),
        legs_lt5cm: lt(0.05),
        legs_lt10pctLane: lt(0.1 * lane),
        legs_lt25pctLane: lt(0.25 * lane),
        clusters4in_halfLane: clusters,
        shortReversals: reversals,
        badSegs,
    };
}

const MAIN_LENGTHS = {
    "rect/none/p2": 1052,
    "rect/circ/p2": 1076,
    "rot23/none/p2": 1052,
    "rot23/circ/p2": 1035,
    "L/none/p2": 828,
    "L/circ/p2": 869,
    "dumbbell/none/p2": 1014,
    "dumbbell/circ/p2": 914,
    "sawtooth/none/p2": 904,
    "sawtooth/circ/p2": 847,
    "blob/none/p2": 1092,
    "blob/circ/p2": 1063,
    "wedge/none/p2": 157,
    "wedge/circ/p2": 203,
    "finger/none/p2": 1124,
    "finger/circ/p2": 1103,
    "notch/none/p2": 1064,
    "notch/circ/p2": 1050,
    "rect/edge/p2": 1018,
    "rect/split/p2": 977,
    "rect/split_circ/p2": 996,
    "rect/pair/p2": 1050,
    "rect/circ/p0": 998,
    "rect/circ/p1": 1035,
    "rect/circ/p3": 1062,
    "finger/none/p1": 1078,
    "finger/none/p3": 1121,
    "notch/none/p1": 1084,
    "notch/none/p3": 1061,
    "dumbbell/none/p4": 935,
    "rect/circ/spiral": 919,
    "blob/none/spiral": 932,
    "rect/circ/combine0.5": 1076,
    "blob/circ/combine0.5": 1063,
    "rect/circ/combine1.3": 1040,
    "rect/circ/skip1": 1120,
    "rot23/circ/angle0": 1028,
    "rect/pair/lane0.5": 2237,
};

function run(name, shape, exName, opt) {
    const perim = SHAPES[shape];
    const ex = EXCL[exName];
    const t0 = Date.now();
    let res;
    try {
        res = P.generateCoveragePath(
            perim.map(toLL),
            toPlannerExcl(ex),
            opt.lane,
            opt.buffer,
            opt.passes,
            "CW",
            opt.tol,
            opt.skip || 0,
            opt.angle == null ? "auto" : "custom",
            opt.angle || 0,
            64,
            !!opt.spiral,
            !!opt.rev,
            opt.headland || 0,
            opt.combine || 0,
        );
    } catch (e) {
        return { name, error: String(e) };
    }
    const ms = Date.now() - t0;
    if (!res) return { name, error: "null result" };
    const pieces =
        res.pieces && res.pieces.length > 0 ? res.pieces : [{ path: res.path }];
    const pathsXY = pieces.map((p) => p.path.map(toXY));
    let pathLen = 0;
    for (const pXY of pathsXY) {
        for (let i = 0; i < pXY.length - 1; i++) {
            pathLen += Math.hypot(
                pXY[i + 1][0] - pXY[i][0],
                pXY[i + 1][1] - pXY[i][1],
            );
        }
    }
    const pathLen_m = +pathLen.toFixed(1);
    const mainLen = MAIN_LENGTHS[name];
    const lenRatio = mainLen ? +(pathLen_m / mainLen).toFixed(2) : null;
    const m = analyze(perim, ex, opt, pathsXY);
    if (DUMP && name === DUMP)
        fs.writeFileSync(
            path.join(__dirname, `dump_${name.replace(/\//g, "_")}.json`),
            JSON.stringify({
                perim,
                ex,
                exBuf: exclPolys(ex, opt.buffer),
                pathXY: pathsXY[0],
                pathsXY,
                opt,
                badSegs: m.badSegs,
            }),
        );
    return {
        name,
        ms,
        pathLen_m,
        lenRatio,
        ...m,
        warnings: res.warnings || [],
        _path: pathsXY[0],
        _paths: pathsXY,
    };
}

const base = { lane: 1.2, buffer: 1, tol: 0.3, passes: 2 };
const cases = [];
for (const shape of Object.keys(SHAPES))
    for (const exName of ["none", "circ"])
        cases.push([`${shape}/${exName}/p2`, shape, exName, base]);
for (const exName of ["edge", "split", "split_circ", "pair"])
    cases.push([`rect/${exName}/p2`, "rect", exName, base]);
for (const passes of [0, 1, 3])
    cases.push([`rect/circ/p${passes}`, "rect", "circ", { ...base, passes }]);
for (const passes of [1, 2, 3])
    cases.push([
        `finger/none/p${passes}`,
        "finger",
        "none",
        { ...base, passes },
    ]);
for (const passes of [1, 2, 3])
    cases.push([`notch/none/p${passes}`, "notch", "none", { ...base, passes }]);
cases.push(["dumbbell/none/p4", "dumbbell", "none", { ...base, passes: 4 }]);
cases.push(["rect/circ/spiral", "rect", "circ", { ...base, spiral: true }]);
cases.push(["blob/none/spiral", "blob", "none", { ...base, spiral: true }]);
cases.push(["rect/circ/combine0.5", "rect", "circ", { ...base, combine: 0.5 }]);
cases.push(["blob/circ/combine0.5", "blob", "circ", { ...base, combine: 0.5 }]);
cases.push(["rect/circ/combine1.3", "rect", "circ", { ...base, combine: 1.3 }]);
cases.push(["rect/circ/skip1", "rect", "circ", { ...base, skip: 1 }]);
cases.push(["rot23/circ/angle0", "rot23", "circ", { ...base, angle: 0 }]);
cases.push([
    "rect/pair/lane0.5",
    "rect",
    "pair",
    { ...base, lane: 0.5, buffer: 0.3, tol: 0.1 },
]);

const FUZZ = +(args.find((a) => a.startsWith("--fuzz=")) || "=0").split("=")[1];
if (FUZZ) {
    let seed = 12345;
    const rnd = () => {
        seed = (seed * 1103515245 + 12345) % 2147483648;
        return seed / 2147483648;
    };
    const U = (a, b) => a + (b - a) * rnd();
    const rows = [];
    for (let t = 0; t < FUZZ; t++) {
        const nv = Math.floor(U(6, 18)),
            R = U(12, 25);
        const perim = Array.from({ length: nv }, (_, i) => {
            const a = (2 * Math.PI * (i + U(-0.3, 0.3))) / nv,
                r = R * U(0.45, 1);
            return [r * Math.cos(a), r * Math.sin(a)];
        });
        const ex = [];
        const nex = Math.floor(U(0, 4));
        for (let k = 0; k < nex; k++) {
            const a = U(0, 2 * Math.PI),
                r = R * U(0, 0.9);
            const c = [r * Math.cos(a), r * Math.sin(a)];
            if (rnd() < 0.6) ex.push({ type: "circle", c, r: U(0.4, 2.5) });
            else {
                const s = U(1, 4),
                    q = U(0, Math.PI);
                ex.push({
                    type: "poly",
                    v: rot(
                        [
                            [-s, -s / 2],
                            [s, -s / 2],
                            [s, s / 2],
                            [-s, s / 2],
                        ],
                        (q * 180) / Math.PI,
                    ).map(([x, y]) => [x + c[0], y + c[1]]),
                });
            }
        }
        const lane = U(0.4, 1.5);
        const opt = {
            lane,
            buffer: U(0.5, 1) * lane + 0.1,
            tol: U(0, 0.4) * lane,
            passes: 1 + Math.floor(U(0, 3)),
            spiral: rnd() < 0.2,
        };
        SHAPES.__f = perim;
        EXCL.__f = ex;
        const name = `fuzz${t}`;
        if (ONLY && !name.includes(ONLY)) continue;
        const r = run(name, "__f", "__f", opt);
        if (r.error) {
            rows.push({ name, error: r.error });
            continue;
        }
        const { badSegs, topGaps, _path, ...brief } = r;
        const outLeg = badSegs.filter((b) => b.out > 0.05).length,
            rawLeg = badSegs.filter((b) => b.raw).length,
            bufLeg = badSegs.filter((b) => b.buf > 0.05).length;
        rows.push({
            ...brief,
            outLeg,
            rawLeg,
            bufLeg,
            nex,
            spiral: opt.spiral,
            passes: opt.passes,
            lane: +lane.toFixed(2),
        });
        if (ONLY && name.includes(ONLY)) {
            console.log(
                JSON.stringify(brief),
                badSegs?.length
                    ? `bad:${badSegs.length} e.g.${JSON.stringify(badSegs.slice(0, 2))}`
                    : "",
                "warnings:",
                r.warnings,
            );
        }
        if (
            DUMP === "all" &&
            (outLeg ||
                rawLeg ||
                r.coverPct < 97 ||
                r.clusters4in_halfLane >= 6 ||
                r.shortReversals >= 2)
        ) {
            fs.writeFileSync(
                path.join(__dirname, `fz_${name}.json`),
                JSON.stringify({
                    perim,
                    ex,
                    exBuf: exclPolys(ex, opt.buffer),
                    pathXY: r._path,
                    opt,
                    badSegs,
                }),
            );
        }
    }
    const ok = rows.filter((r) => !r.error);
    const frac = (f) => `${ok.filter(f).length}/${ok.length}`;
    const med = (a) => {
        const s = a.slice().sort((x, y) => x - y);
        return s[Math.floor(s.length / 2)];
    };
    console.log(
        "errors:",
        rows.filter((r) => r.error).length,
        rows.filter((r) => r.error).slice(0, 3),
    );
    console.log(
        "runs with centerline >5cm outside perimeter:",
        frac((r) => r.outLeg > 0),
        " max:",
        Math.max(...ok.map((r) => r.maxOutsidePerim_m)),
    );
    console.log(
        "runs with a leg entering a raw obstacle:",
        frac((r) => r.rawLeg > 0),
    );
    console.log(
        "runs with a leg >5cm inside (buffer - tolerance):",
        frac((r) => r.bufLeg > 0),
    );
    console.log(
        "coverage median / p10 / min:",
        med(ok.map((r) => r.coverPct)),
        ok.map((r) => r.coverPct).sort((a, b) => a - b)[
            Math.floor(ok.length / 10)
        ],
        Math.min(...ok.map((r) => r.coverPct)),
    );
    console.log(
        "runs <99% coverage:",
        frac((r) => r.coverPct < 99),
        " <95%:",
        frac((r) => r.coverPct < 95),
    );
    console.log(
        "runs with legs < 5cm:",
        frac((r) => r.legs_lt5cm > 0),
        " with 4+ wp inside lane/2:",
        frac((r) => r.clusters4in_halfLane > 0),
        " with short reversals:",
        frac((r) => r.shortReversals > 0),
        " with near-dup revisits:",
        frac((r) => r.nearDupPairs > 0),
    );
    const by = (k) => {
        const g = {};
        for (const r of ok) {
            const key = r[k];
            if (!g[key]) {
                g[key] = [];
            }
            g[key].push(r);
        }
        return Object.fromEntries(
            Object.entries(g).map(([kk, v]) => [
                kk,
                {
                    n: v.length,
                    viol: v.filter((r) => r.outLeg || r.rawLeg).length,
                    medCov: med(v.map((r) => r.coverPct)),
                },
            ]),
        );
    };
    console.log("by #exclusions:", JSON.stringify(by("nex")));
    console.log("by spiral:", JSON.stringify(by("spiral")));
    if (args.includes("--write-json") || process.env.WRITE_JSON) {
        fs.writeFileSync(
            path.join(__dirname, NOCLIP ? "fuzz_noclip.json" : "fuzz.json"),
            JSON.stringify(rows, null, 1),
        );
    }
    const errCount = rows.filter((r) => r.error).length;
    const outCount = ok.filter((r) => r.outLeg > 0).length;
    const rawCount = ok.filter((r) => r.rawLeg > 0).length;
    const bufCount = ok.filter((r) => r.bufLeg > 0).length;
    const medCoverage = med(ok.map((r) => r.coverPct));
    const p10Coverage = ok.map((r) => r.coverPct).sort((a, b) => a - b)[
        Math.floor(ok.length / 10)
    ];

    const medMs = med(ok.map((r) => r.ms));
    const maxMs = Math.max(...ok.map((r) => r.ms));
    const medWp = med(ok.map((r) => r.wp));
    const maxWp = Math.max(...ok.map((r) => r.wp));

    const pass =
        errCount === 0 &&
        outCount === 0 &&
        rawCount === 0 &&
        bufCount === 0 &&
        medCoverage >= 99.5 &&
        p10Coverage >= 98.0 &&
        medMs <= 1500 &&
        maxMs <= 15000 &&
        medWp <= 300 &&
        maxWp <= 1500;

    if (!pass) {
        console.error(
            "FAIL: Fuzz suite thresholds failed:",
            JSON.stringify({
                errCount,
                outCount,
                rawCount,
                bufCount,
                medCoverage,
                p10Coverage,
                medMs,
                maxMs,
                medWp,
                maxWp,
            }),
        );
        process.exit(1);
    }
    console.log("PASS: All fuzz suite thresholds satisfied.");
    process.exit(0);
}

const out = [];
for (const [name, ...rest] of cases) {
    if (ONLY && !name.includes(ONLY)) continue;
    const r = run(name, ...rest);
    out.push(r);
    const { badSegs, topGaps, _path, ...brief } = r;
    console.log(
        JSON.stringify(brief),
        topGaps ? `gaps:${JSON.stringify(topGaps)}` : "",
        badSegs?.length
            ? `bad:${badSegs.length} e.g.${JSON.stringify(badSegs.slice(0, 2))}`
            : "",
    );
}
if (args.includes("--write-json") || process.env.WRITE_JSON) {
    fs.writeFileSync(
        path.join(__dirname, NOCLIP ? "results_noclip.json" : "results.json"),
        JSON.stringify(out, null, 1),
    );
}

const errCount = out.filter((r) => r.error).length;
const rawCount = out.filter((r) => r.segsIntoRawExcl > 0).length;
const outCount = out.filter((r) => r.lenOutsidePerim_m > 0.05).length;
const bufCount = out.filter((r) => r.maxIntoBuffer_m > 0.05).length;
const lowCovCount = out.filter((r) => r.coverPct < 95).length;
const warnCount = out.filter((r) => r.warnings && r.warnings.length > 0).length;

const maxMs = Math.max(...out.map((r) => r.ms));
const maxWp = Math.max(...out.map((r) => r.wp));

const excessLenCases = out.filter((r) => {
    const baseline = MAIN_LENGTHS[r.name];
    return baseline && r.pathLen_m > 1.15 * baseline;
});
const missingBaselineCases = ONLY
    ? []
    : out.filter((r) => typeof MAIN_LENGTHS[r.name] !== "number");

const pass =
    errCount === 0 &&
    rawCount === 0 &&
    outCount === 0 &&
    bufCount === 0 &&
    lowCovCount === 0 &&
    warnCount === 0 &&
    excessLenCases.length === 0 &&
    missingBaselineCases.length === 0 &&
    maxMs <= 10000 &&
    maxWp <= 1200;

if (!pass) {
    console.error(
        "FAIL: Fixed scenarios threshold check failed:",
        JSON.stringify({
            errCount,
            rawCount,
            outCount,
            bufCount,
            lowCovCount,
            warnCount,
            excessLenCount: excessLenCases.length,
            excessLenCases: excessLenCases.map((r) => ({
                name: r.name,
                pathLen_m: r.pathLen_m,
                baseline: MAIN_LENGTHS[r.name],
                ratio: r.lenRatio,
            })),
            missingBaselineCount: missingBaselineCases.length,
            missingBaselineCases: missingBaselineCases.map((r) => r.name),
            maxMs,
            maxWp,
        }),
    );
    process.exit(1);
}
console.log("PASS: All fixed scenarios passed.");
