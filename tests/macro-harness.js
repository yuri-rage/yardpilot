// Test harness for yardpilot macros.js
// Usage: node tests/macro-harness.js
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

const MACROS = path.resolve(__dirname, "../assets/macros.js");
const ctx = { console, Math, Number, Array, Set, Map, JSON, Object, String };
vm.createContext(ctx);
vm.runInContext(
    `${fs.readFileSync(MACROS, "utf8")}\n;globalThis.__M = { MACRO_COMMANDS, MACRO_VEHICLES };`,
    ctx,
);
const M = { ...ctx, ...ctx.__M };

let failures = 0;
let passes = 0;
function check(name, cond, detail) {
    if (cond) {
        passes++;
    } else {
        failures++;
        console.error(`FAIL: ${name}${detail ? ` — ${detail}` : ""}`);
    }
}
const eq = (a, b) => JSON.stringify(a) === JSON.stringify(b);

// Helpers
const pathN = (n) =>
    Array.from({ length: n }, (_, i) => [33.3 + i * 1e-5, -96.8 - i * 1e-5]);
// Summarize mission body (skip header + home) as "WPk" / command names
function summarize(lines) {
    let wp = 0;
    return lines.slice(2).map((l) => {
        const c = l.split("\t");
        if (c[3] === "16") return `WP${++wp}`;
        const name = M.MACRO_COMMANDS[c[3]].name.replace(/^DO_SET_/, "");
        return `${name}:${c[4]}:${c[5]}`;
    });
}
const T = (position, index, command, p1, p2) => {
    const t = M.macroNewTask(command, position, index);
    if (p1 !== undefined) t.params[0] = p1;
    if (p2 !== undefined) t.params[1] = p2;
    return t;
};

// ── 1. Plain export unchanged (no macro) ──
{
    const p = pathN(3);
    const { lines, macroCount } = M.buildMissionLines(p, null, 50);
    const expected = [
        "QGC WPL 110",
        `0\t1\t0\t16\t0\t0\t0\t0\t${p[0][0].toFixed(7)}\t${p[0][1].toFixed(7)}\t50.000000\t1`,
        ...p.map(
            ([la, lo], i) =>
                `${i + 1}\t0\t3\t16\t0\t0\t0\t0\t${la.toFixed(7)}\t${lo.toFixed(7)}\t50.000000\t1`,
        ),
    ];
    check("plain export identical", eq(lines, expected));
    check("plain export macroCount 0", macroCount === 0);
}

// ── 2. Worked example from spec ──
{
    const macro = {
        name: "Mower PTO",
        vehicle: "rover",
        tasks: [
            T("before", 1, 183, 9, 1700),
            T("after", 1, 181, 0, 1),
            T("then", null, 183, 9, 1900),
            T("after", "last", 181, 0, 0),
            T("then", null, 183, 9, 1100),
        ],
    };
    const { lines, macroCount } = M.buildMissionLines(pathN(4), macro, 0);
    check(
        "worked example order",
        eq(summarize(lines), [
            "SERVO:9:1700",
            "WP1",
            "RELAY:0:1",
            "SERVO:9:1900",
            "WP2",
            "WP3",
            "WP4",
            "RELAY:0:0",
            "SERVO:9:1100",
        ]),
        JSON.stringify(summarize(lines)),
    );
    check("worked example macroCount", macroCount === 5);
    const seqs = lines.slice(1).map((l) => Number(l.split("\t")[0]));
    check(
        "seq numbers contiguous",
        eq(
            seqs,
            seqs.map((_, i) => i),
        ),
    );
}

// ── 3. Shared gap: After N precedes Before N+1 regardless of list order ──
{
    const macro = {
        vehicle: "rover",
        tasks: [
            T("before", 2, 181, 1, 1),
            T("then", null, 181, 2, 1),
            T("after", 1, 181, 3, 1),
            T("after", 1, 181, 4, 1),
        ],
    };
    const s = summarize(M.buildMissionLines(pathN(3), macro, 0).lines);
    check(
        "shared gap ordering",
        eq(s, [
            "WP1",
            "RELAY:3:1",
            "RELAY:4:1",
            "RELAY:1:1",
            "RELAY:2:1",
            "WP2",
            "WP3",
        ]),
        JSON.stringify(s),
    );
}

// ── 4. Out-of-range skip (and Then children follow their anchor) ──
{
    const macro = {
        vehicle: "rover",
        tasks: [
            T("after", 40, 181, 0, 1),
            T("then", null, 183, 9, 1900),
            T("after", 2, 181, 5, 1),
        ],
    };
    const r = M.buildMissionLines(pathN(3), macro, 0);
    check(
        "out of range skipped",
        eq(summarize(r.lines), ["WP1", "WP2", "RELAY:5:1", "WP3"]),
    );
    check(
        "out of range issues for anchor + then",
        eq(
            r.rangeIssues.map((x) => x.task),
            [0, 1],
        ),
    );
    check("out of range macroCount", r.macroCount === 1);
}

// ── 5. Unsupported command for vehicle skipped and flagged ──
{
    const macro = {
        vehicle: "rover",
        tasks: [T("before", 1, 223, 1), T("then", null, 181, 0, 1)],
    };
    const issues = M.validateMacro(macro);
    check(
        "unsupported flagged",
        issues.length === 1 && issues[0].task === 0,
        JSON.stringify(issues),
    );
    const s = summarize(M.buildMissionLines(pathN(2), macro, 0).lines);
    check(
        "unsupported skipped, then still emitted",
        eq(s, ["RELAY:0:1", "WP1", "WP2"]),
        JSON.stringify(s),
    );
    macro.vehicle = "copter";
    check("supported on copter", M.validateMacro(macro).length === 0);
}

// ── 6. Then at top / invalid index flagged ──
{
    const issues = M.validateMacro({
        vehicle: "rover",
        tasks: [
            T("then", null, 181),
            T("after", "abc", 181),
            T("after", 0, 181),
        ],
    });
    check(
        "then-at-top and invalid indices flagged",
        eq(
            issues.map((x) => x.task),
            [0, 1, 2],
        ),
        JSON.stringify(issues),
    );
    check("parse index 'Last'", M.macroParseIndex(" Last ") === "last");
    check("parse index '12'", M.macroParseIndex("12") === 12);
    check("parse index '1.5'", M.macroParseIndex("1.5") === null);
}

// ── 7. Location commands: blank lat/lon → 0, blank alt → sidebar altitude ──
{
    const t = T("after", 1, 19, 30);
    const { lines } = M.buildMissionLines(
        pathN(1),
        { vehicle: "rover", tasks: [t] },
        12.5,
    );
    const c = lines[3].split("\t");
    check(
        "loiter_time fields",
        c[3] === "19" &&
            c[4] === "30" &&
            c[8] === "0.0000000" &&
            c[9] === "0.0000000" &&
            c[10] === "12.500000",
        lines[3],
    );
    t.params[4] = 33.123;
    t.params[5] = -96.5;
    t.params[6] = 3;
    const c2 = M.buildMissionLines(
        pathN(1),
        { vehicle: "rover", tasks: [t] },
        12.5,
    ).lines[3].split("\t");
    check(
        "explicit lat/lon/alt",
        c2[8] === "33.1230000" &&
            c2[9] === "-96.5000000" &&
            c2[10] === "3.000000",
    );
}

// ── 8. Metric storage values exported verbatim (rounded to 6 dp) ──
{
    const t = T("before", 1, 178, 1, 0.44704 * 3); // 3 mph stored as m/s
    const c = M.buildMissionLines(
        pathN(1),
        { vehicle: "rover", tasks: [t] },
        0,
    ).lines[2].split("\t");
    check("speed exported in m/s", c[5] === "1.34112", c[5]);
}

// ── 9. Per-piece expansion: 'Last' relative to each piece ──
{
    const macro = { vehicle: "rover", tasks: [T("after", "last", 181, 0, 0)] };
    const a = summarize(M.buildMissionLines(pathN(2), macro, 0).lines);
    const b = summarize(M.buildMissionLines(pathN(5), macro, 0).lines);
    check("piece A last", eq(a, ["WP1", "WP2", "RELAY:0:0"]));
    check("piece B last", eq(b.slice(-2), ["WP5", "RELAY:0:0"]));
}

// ── 10. JSON round-trip and rejection ──
{
    const macro = {
        name: "Test",
        vehicle: "plane",
        tasks: [T("before", 1, 183, 9, 1700), T("then", 7, 112, 5)],
    };
    const json = JSON.stringify(M.macroToJson(macro));
    const back = M.parseMacroJson(json);
    check(
        "round-trip name/vehicle",
        back.name === "Test" && back.vehicle === "plane",
    );
    check("round-trip then index nulled", back.tasks[1].index === null);
    check("round-trip params", eq(back.tasks[0].params, macro.tasks[0].params));
    const bad = [
        "not json",
        JSON.stringify({ foo: 1 }),
        JSON.stringify({
            format: "yardpilot-macro",
            version: 99,
            vehicle: "rover",
            tasks: [],
        }),
        JSON.stringify({
            format: "yardpilot-macro",
            version: 1,
            vehicle: "boat",
            tasks: [],
        }),
        JSON.stringify({
            format: "yardpilot-macro",
            version: 1,
            vehicle: "rover",
        }),
    ];
    for (const b of bad) {
        let threw = false;
        try {
            M.parseMacroJson(b);
        } catch (_e) {
            threw = true;
        }
        check(`reject ${b.slice(0, 40)}`, threw);
    }
    const unknown = M.parseMacroJson(
        JSON.stringify({
            format: "yardpilot-macro",
            version: 1,
            vehicle: "rover",
            tasks: [
                {
                    position: "after",
                    index: "LAST",
                    command: 99999,
                    params: [],
                },
            ],
        }),
    );
    check("unknown command kept", unknown.tasks[0].command === 99999);
    check("unknown command flagged", M.validateMacro(unknown).length === 1);
    check("LAST normalized", unknown.tasks[0].index === "last");
}

// ── 11. Edge cases: empty path, bogus position anchor, null task rejection ──
{
    const emptyRes = M.buildMissionLines([], null, 0);
    check("empty path returns empty lines", eq(emptyRes.lines, []));
    check("empty path returns 0 count", emptyRes.macroCount === 0);

    const bogusAnchors = M.macroResolveAnchors([
        { position: "bogus", index: 2, command: 181, params: [] },
        { position: "then", index: null, command: 181, params: [] },
    ]);
    check(
        "bogus position resets anchor to null",
        bogusAnchors[0] === null && bogusAnchors[1] === null,
    );

    let threwNullTask = false;
    try {
        M.parseMacroJson(
            JSON.stringify({
                format: "yardpilot-macro",
                version: 1,
                vehicle: "rover",
                tasks: [null],
            }),
        );
    } catch (_e) {
        threwNullTask = true;
    }
    check("rejects null task entry", threwNullTask);

    const navDelayDef = M.MACRO_COMMANDS[93];
    check(
        "NAV_DELAY hour/min/sec default to -1",
        navDelayDef.params[1][3] === -1 &&
            navDelayDef.params[2][3] === -1 &&
            navDelayDef.params[3][3] === -1,
    );
}

// ── 12. Command table sanity ──
for (const [id, def] of Object.entries(M.MACRO_COMMANDS)) {
    const idx = def.params.map((p) => p[0]);
    const dup = new Set(idx).size !== idx.length;
    const clash =
        (def.loc && idx.some((i) => i >= 5)) || (def.alt && idx.includes(7));
    check(
        `command ${id} param indices`,
        !dup && !clash && /^[rcp]+$/.test(def.v),
    );
}
for (const v of Object.keys(M.MACRO_VEHICLES)) {
    const g = M.macroCommandsForVehicle(v);
    check(
        `${v} has servo+relay`,
        g.DO.some((c) => c.id === 183) && g.DO.some((c) => c.id === 181),
    );
}

console.log(`${passes} passed, ${failures} failed`);
if (failures) {
    console.error("FAIL: Macro tests failed.");
    process.exit(1);
}
console.log("PASS: All macro tests passed.");
