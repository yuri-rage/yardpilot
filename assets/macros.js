// ── Mission macros ────────────────────────────────────────
// Pure (DOM-free) macro logic: per-vehicle ArduPilot mission command tables,
// macro validation, JSON (de)serialization, and expansion of a coverage path
// into QGC WPL 110 mission lines with macro items inserted.
//
// Macro shape:
//   {
//     name: string,
//     vehicle: "rover" | "copter" | "plane",
//     tasks: [{
//       position: "before" | "after" | "then",
//       index: number | "last" | null,   // path waypoint 1..N (ignored for "then")
//       command: number,                 // MAV_CMD id
//       params: [p1..p7],                // metric values; null = blank
//     }]
//   }

const MACRO_FORMAT = "yardpilot-macro";
const MACRO_VERSION = 1;

const MACRO_VEHICLES = {
    rover: "ArduPilot Rover",
    copter: "ArduPilot Copter",
    plane: "ArduPilot Plane",
};

// Vehicle code letters used in the command table: r = rover, c = copter, p = plane
const MACRO_VEHICLE_CODES = { rover: "r", copter: "c", plane: "p" };

// Param definition: [paramIndex (1-7), label, unit, default]
//   unit: "dist" | "speed" | "alt" are unit-converted in the UI (stored metric);
//         any other string is a literal unit suffix shown in the label ("" = none).
// loc: true → param 5/6 are latitude/longitude (blank = 0 = current position),
//             param 7 is altitude (blank = sidebar altitude).
// alt: true → param 7 is altitude only (blank = sidebar altitude).
const MACRO_COMMANDS = {
    // ── Navigation ──
    16: {
        name: "NAV_WAYPOINT",
        desc: "Navigate to a location, optionally holding there.",
        v: "rcp",
        loc: true,
        params: [
            [1, "Hold", "s", 0],
            [2, "Accept radius", "dist", 0],
        ],
    },
    17: {
        name: "NAV_LOITER_UNLIM",
        desc: "Loiter at a location indefinitely.",
        v: "rcp",
        loc: true,
        params: [[3, "Radius", "dist", 0]],
    },
    18: {
        name: "NAV_LOITER_TURNS",
        desc: "Circle a location for a number of turns.",
        v: "cp",
        loc: true,
        params: [
            [1, "Turns", "", 1],
            [3, "Radius", "dist", 0],
            [4, "Xtrack exit (0/1)", "", 0],
        ],
    },
    19: {
        name: "NAV_LOITER_TIME",
        desc: "Loiter at a location for a set time.",
        v: "rcp",
        loc: true,
        params: [
            [1, "Time", "s", 10],
            [3, "Radius", "dist", 0],
        ],
    },
    20: {
        name: "NAV_RETURN_TO_LAUNCH",
        desc: "Return to the home location.",
        v: "rcp",
        params: [],
    },
    21: {
        name: "NAV_LAND",
        desc: "Land at a location (blank lat/lon = current position).",
        v: "cp",
        loc: true,
        params: [
            [1, "Abort alt", "alt", 0],
            [2, "Precision land (0-2)", "", 0],
        ],
    },
    22: {
        name: "NAV_TAKEOFF",
        desc: "Take off to the given altitude.",
        v: "cp",
        alt: true,
        params: [[1, "Min pitch", "deg", 0]],
    },
    30: {
        name: "NAV_CONTINUE_AND_CHANGE_ALT",
        desc: "Continue on the current course while climbing/descending to an altitude.",
        v: "p",
        alt: true,
        params: [[1, "Mode (0 any, 1 climb, 2 descend)", "", 0]],
    },
    31: {
        name: "NAV_LOITER_TO_ALT",
        desc: "Loiter at a location until reaching an altitude.",
        v: "p",
        loc: true,
        params: [
            [2, "Radius", "dist", 0],
            [4, "Xtrack exit (0/1)", "", 0],
        ],
    },
    82: {
        name: "NAV_SPLINE_WAYPOINT",
        desc: "Navigate to a location along a spline path.",
        v: "c",
        loc: true,
        params: [[1, "Hold", "s", 0]],
    },
    83: {
        name: "NAV_ALTITUDE_WAIT",
        desc: "Wait (e.g. under a balloon) until an altitude or descent rate is reached.",
        v: "p",
        params: [
            [1, "Altitude", "alt", 0],
            [2, "Descent rate", "speed", 0],
            [3, "Wiggle time", "s", 0],
        ],
    },
    84: {
        name: "NAV_VTOL_TAKEOFF",
        desc: "VTOL takeoff to the given altitude.",
        v: "p",
        alt: true,
        params: [],
    },
    85: {
        name: "NAV_VTOL_LAND",
        desc: "VTOL land at a location (blank lat/lon = current position).",
        v: "p",
        loc: true,
        params: [[1, "Options", "", 0]],
    },
    92: {
        name: "NAV_GUIDED_ENABLE",
        desc: "Hand control to an external (guided) controller.",
        v: "rc",
        params: [[1, "Enable (0/1)", "", 1]],
    },
    93: {
        name: "NAV_DELAY",
        desc: "Delay the mission for a number of seconds or until a UTC time (delay -1 to use time).",
        v: "rcp",
        params: [
            [1, "Delay", "s", 5],
            [2, "Hour (UTC)", "", -1],
            [3, "Minute (UTC)", "", -1],
            [4, "Second (UTC)", "", -1],
        ],
    },
    94: {
        name: "NAV_PAYLOAD_PLACE",
        desc: "Descend at a location and release a payload.",
        v: "cp",
        loc: true,
        params: [[1, "Max descent", "dist", 0]],
    },
    213: {
        name: "NAV_SET_YAW_SPEED",
        desc: "Drive at a heading and speed.",
        v: "r",
        params: [
            [1, "Heading", "deg", 0],
            [2, "Speed", "speed", 0],
            [3, "Relative (0/1)", "", 0],
        ],
    },
    42702: {
        name: "NAV_SCRIPT_TIME",
        desc: "Hand control to a Lua script for up to a timeout.",
        v: "rcp",
        params: [
            [1, "Command", "", 0],
            [2, "Timeout", "s", 0],
            [3, "Arg 1", "", 0],
            [4, "Arg 2", "", 0],
            [5, "Arg 3", "", 0],
            [6, "Arg 4", "", 0],
        ],
    },
    42703: {
        name: "NAV_ATTITUDE_TIME",
        desc: "Hold an attitude and climb rate for a time.",
        v: "c",
        params: [
            [1, "Time", "s", 0],
            [2, "Roll", "deg", 0],
            [3, "Pitch", "deg", 0],
            [4, "Yaw", "deg", 0],
            [5, "Climb rate", "speed", 0],
        ],
    },

    // ── Conditions ──
    112: {
        name: "CONDITION_DELAY",
        desc: "Delay the next DO commands for a number of seconds.",
        v: "rcp",
        params: [[1, "Delay", "s", 5]],
    },
    114: {
        name: "CONDITION_DISTANCE",
        desc: "Delay the next DO commands until within a distance of the next waypoint.",
        v: "rcp",
        params: [[1, "Distance", "dist", 0]],
    },
    115: {
        name: "CONDITION_YAW",
        desc: "Point the vehicle at a heading.",
        v: "c",
        params: [
            [1, "Angle", "deg", 0],
            [2, "Rate", "deg/s", 0],
            [3, "Direction (-1 CCW, 1 CW)", "", 1],
            [4, "Relative (0/1)", "", 0],
        ],
    },

    // ── DO commands ──
    177: {
        name: "DO_JUMP",
        desc: "Jump to a mission item (final exported seq number) a number of times. Consider JUMP_TAG instead to avoid fragile absolute sequence numbers.",
        v: "rcp",
        params: [
            [1, "Seq # (or use JUMP_TAG)", "", 1],
            [2, "Repeat", "", -1],
        ],
    },
    178: {
        name: "DO_CHANGE_SPEED",
        desc: "Change target speed and/or throttle.",
        v: "rcp",
        params: [
            [1, "Type (0 air, 1 ground)", "", 1],
            [2, "Speed", "speed", 1],
            [3, "Throttle (-1 no change)", "%", -1],
        ],
    },
    179: {
        name: "DO_SET_HOME",
        desc: "Set home to the current position or a location.",
        v: "rcp",
        loc: true,
        params: [[1, "Use current (0/1)", "", 1]],
    },
    181: {
        name: "DO_SET_RELAY",
        desc: "Set a relay on or off.",
        v: "rcp",
        params: [
            [1, "Relay #", "", 0],
            [2, "State (0 off, 1 on)", "", 1],
        ],
    },
    182: {
        name: "DO_REPEAT_RELAY",
        desc: "Toggle a relay a number of times.",
        v: "rcp",
        params: [
            [1, "Relay #", "", 0],
            [2, "Count", "", 1],
            [3, "Cycle time", "s", 1],
        ],
    },
    183: {
        name: "DO_SET_SERVO",
        desc: "Set a servo output to a PWM value.",
        v: "rcp",
        params: [
            [1, "Servo #", "", 9],
            [2, "PWM", "µs", 1500],
        ],
    },
    184: {
        name: "DO_REPEAT_SERVO",
        desc: "Cycle a servo between a PWM value and its trim a number of times.",
        v: "rcp",
        params: [
            [1, "Servo #", "", 9],
            [2, "PWM", "µs", 1500],
            [3, "Count", "", 1],
            [4, "Cycle time", "s", 1],
        ],
    },
    188: {
        name: "DO_RETURN_PATH_START",
        desc: "Mark the start of a return path for RTL.",
        v: "rcp",
        params: [],
    },
    189: {
        name: "DO_LAND_START",
        desc: "Mark the start of a landing sequence.",
        v: "rcp",
        loc: true,
        params: [],
    },
    191: {
        name: "DO_GO_AROUND",
        desc: "Abort a landing and climb to an altitude.",
        v: "p",
        params: [[1, "Altitude", "alt", 0]],
    },
    194: {
        name: "DO_SET_REVERSE",
        desc: "Drive in reverse (1) or forward (0).",
        v: "r",
        params: [[1, "Reverse (0/1)", "", 1]],
    },
    195: {
        name: "DO_SET_ROI_LOCATION",
        desc: "Point the camera/gimbal at a location.",
        v: "rcp",
        loc: true,
        params: [[1, "Gimbal #", "", 0]],
    },
    197: {
        name: "DO_SET_ROI_NONE",
        desc: "Cancel any region of interest.",
        v: "rcp",
        params: [[1, "Gimbal #", "", 0]],
    },
    201: {
        name: "DO_SET_ROI",
        desc: "Point the camera/gimbal at a location (legacy).",
        v: "rcp",
        loc: true,
        params: [],
    },
    202: {
        name: "DO_DIGICAM_CONFIGURE",
        desc: "Configure an onboard camera.",
        v: "rcp",
        params: [
            [1, "Mode", "", 0],
            [2, "Shutter speed", "", 0],
            [3, "Aperture", "", 0],
            [4, "ISO", "", 0],
            [5, "Exposure type", "", 0],
            [6, "Command id", "", 0],
            [7, "Engine cut-off", "", 0],
        ],
    },
    203: {
        name: "DO_DIGICAM_CONTROL",
        desc: "Trigger or control an onboard camera.",
        v: "rcp",
        params: [
            [1, "Session", "", 0],
            [2, "Zoom pos", "", 0],
            [3, "Zoom step", "", 0],
            [4, "Focus lock", "", 0],
            [5, "Shutter cmd", "", 1],
            [6, "Command id", "", 0],
        ],
    },
    205: {
        name: "DO_MOUNT_CONTROL",
        desc: "Set the camera mount angles.",
        v: "rcp",
        params: [
            [1, "Pitch", "deg", 0],
            [2, "Roll", "deg", 0],
            [3, "Yaw", "deg", 0],
            [7, "Mode", "", 2],
        ],
    },
    206: {
        name: "DO_SET_CAM_TRIGG_DIST",
        desc: "Trigger the camera every N distance (0 to stop).",
        v: "rcp",
        params: [
            [1, "Distance", "dist", 0],
            [3, "Trigger once (0/1)", "", 0],
        ],
    },
    207: {
        name: "DO_FENCE_ENABLE",
        desc: "Disable (0), enable (1), or disable floor only (2) the geofence.",
        v: "rcp",
        params: [[1, "Enable (0-2)", "", 1]],
    },
    208: {
        name: "DO_PARACHUTE",
        desc: "Disable (0), enable (1), or release (2) the parachute.",
        v: "cp",
        params: [[1, "Action (0-2)", "", 1]],
    },
    210: {
        name: "DO_INVERTED_FLIGHT",
        desc: "Fly inverted (1) or normal (0).",
        v: "p",
        params: [[1, "Inverted (0/1)", "", 0]],
    },
    211: {
        name: "DO_GRIPPER",
        desc: "Release (0) or grab (1) with a gripper.",
        v: "rcp",
        params: [
            [1, "Gripper #", "", 1],
            [2, "Action (0 release, 1 grab)", "", 0],
        ],
    },
    212: {
        name: "DO_AUTOTUNE_ENABLE",
        desc: "Enable (1) or disable (0) autotune.",
        v: "p",
        params: [[1, "Enable (0/1)", "", 1]],
    },
    215: {
        name: "DO_SET_RESUME_REPEAT_DIST",
        desc: "Rewind distance used when resuming an interrupted mission.",
        v: "rcp",
        params: [[1, "Distance", "dist", 0]],
    },
    216: {
        name: "DO_SPRAYER",
        desc: "Turn the sprayer on (1) or off (0).",
        v: "rcp",
        params: [[1, "Enable (0/1)", "", 1]],
    },
    217: {
        name: "DO_SEND_SCRIPT_MESSAGE",
        desc: "Send a message to a Lua script.",
        v: "rcp",
        params: [
            [1, "Id", "", 0],
            [2, "P1", "", 0],
            [3, "P2", "", 0],
            [4, "P3", "", 0],
        ],
    },
    218: {
        name: "DO_AUX_FUNCTION",
        desc: "Trigger an RCx_OPTION auxiliary function.",
        v: "rcp",
        params: [
            [1, "Aux function #", "", 0],
            [2, "Switch (0 low, 1 mid, 2 high)", "", 2],
        ],
    },
    222: {
        name: "DO_GUIDED_LIMITS",
        desc: "Set limits for a following NAV_GUIDED_ENABLE.",
        v: "c",
        params: [
            [1, "Timeout", "s", 0],
            [2, "Min alt", "alt", 0],
            [3, "Max alt", "alt", 0],
            [4, "Max horiz dist", "dist", 0],
        ],
    },
    223: {
        name: "DO_ENGINE_CONTROL",
        desc: "Start (1) or stop (0) an internal combustion engine.",
        v: "cp",
        params: [
            [1, "Start (0/1)", "", 1],
            [2, "Cold start (0/1)", "", 0],
            [3, "Height delay", "alt", 0],
            [4, "Options", "", 0],
        ],
    },
    531: {
        name: "SET_CAMERA_ZOOM",
        desc: "Set camera zoom.",
        v: "rcp",
        params: [
            [1, "Zoom type", "", 1],
            [2, "Zoom value", "", 0],
        ],
    },
    532: {
        name: "SET_CAMERA_FOCUS",
        desc: "Set camera focus.",
        v: "rcp",
        params: [
            [1, "Focus type", "", 1],
            [2, "Focus value", "", 0],
        ],
    },
    600: {
        name: "JUMP_TAG",
        desc: "Label this point in the mission for DO_JUMP_TAG.",
        v: "rcp",
        params: [[1, "Tag", "", 1]],
    },
    601: {
        name: "DO_JUMP_TAG",
        desc: "Jump to a JUMP_TAG a number of times.",
        v: "rcp",
        params: [
            [1, "Tag", "", 1],
            [2, "Repeat", "", -1],
        ],
    },
    1000: {
        name: "DO_GIMBAL_MANAGER_PITCHYAW",
        desc: "Set gimbal pitch and yaw angles/rates.",
        v: "rcp",
        params: [
            [1, "Pitch", "deg", 0],
            [2, "Yaw", "deg", 0],
            [3, "Pitch rate", "deg/s", 0],
            [4, "Yaw rate", "deg/s", 0],
            [5, "Flags", "", 0],
            [7, "Gimbal id", "", 0],
        ],
    },
    2000: {
        name: "IMAGE_START_CAPTURE",
        desc: "Start capturing images.",
        v: "rcp",
        params: [
            [1, "Camera #", "", 0],
            [2, "Interval", "s", 0],
            [3, "Total images", "", 1],
            [4, "Sequence #", "", 0],
        ],
    },
    2001: {
        name: "IMAGE_STOP_CAPTURE",
        desc: "Stop capturing images.",
        v: "rcp",
        params: [[1, "Camera #", "", 0]],
    },
    2500: {
        name: "VIDEO_START_CAPTURE",
        desc: "Start recording video.",
        v: "rcp",
        params: [[1, "Stream #", "", 0]],
    },
    2501: {
        name: "VIDEO_STOP_CAPTURE",
        desc: "Stop recording video.",
        v: "rcp",
        params: [[1, "Stream #", "", 0]],
    },
    3000: {
        name: "DO_VTOL_TRANSITION",
        desc: "Transition to multicopter (3) or fixed-wing (4) flight.",
        v: "p",
        params: [[1, "State (3 MC, 4 FW)", "", 3]],
    },
    42600: {
        name: "DO_WINCH",
        desc: "Control a winch.",
        v: "c",
        params: [
            [1, "Winch #", "", 1],
            [2, "Action (0 relax, 1 length, 2 rate)", "", 0],
            [3, "Length", "dist", 0],
            [4, "Rate", "speed", 0],
        ],
    },
};

function macroCommandSupported(cmdId, vehicle) {
    const def = MACRO_COMMANDS[cmdId];
    const code = MACRO_VEHICLE_CODES[vehicle];
    return !!(def && code && def.v.includes(code));
}

// Commands for a vehicle, grouped for the dropdown.
function macroCommandsForVehicle(vehicle) {
    const groups = { NAV: [], CONDITION: [], DO: [] };
    for (const [id, def] of Object.entries(MACRO_COMMANDS)) {
        if (!macroCommandSupported(id, vehicle)) continue;
        const group = def.name.startsWith("NAV_")
            ? "NAV"
            : def.name.startsWith("CONDITION_")
              ? "CONDITION"
              : "DO";
        groups[group].push({ id: Number(id), ...def });
    }
    for (const g of Object.values(groups)) {
        g.sort((a, b) => a.name.localeCompare(b.name));
    }
    return groups;
}

// Full ordered field list for a command (params + location/altitude fields).
// Each field: { i (1-7), label, unit, def, kind: "param" | "lat" | "lon" | "alt", placeholder?: string }
function macroCommandFields(cmdId) {
    const def = MACRO_COMMANDS[cmdId];
    if (!def) return [];
    const fields = def.params.map(([i, label, unit, d]) => {
        let placeholder;
        if (cmdId === 177 && i === 1) {
            placeholder = "seq # (or use JUMP_TAG)";
        }
        return {
            i,
            label,
            unit,
            def: d,
            kind: "param",
            placeholder,
        };
    });
    if (def.loc) {
        let latLonPlaceholder = "current";
        if (cmdId === 195 || cmdId === 201) {
            latLonPlaceholder = "0 (cancel)";
        } else if (cmdId === 189) {
            latLonPlaceholder = "optional";
        }
        fields.push({
            i: 5,
            label: "Lat",
            unit: "deg",
            def: null,
            kind: "lat",
            placeholder: latLonPlaceholder,
        });
        fields.push({
            i: 6,
            label: "Lon",
            unit: "deg",
            def: null,
            kind: "lon",
            placeholder: latLonPlaceholder,
        });
    }
    if (def.loc || def.alt) {
        fields.push({
            i: 7,
            label: "Alt",
            unit: "alt",
            def: null,
            kind: "alt",
            placeholder: "sidebar",
        });
    }
    return fields;
}

function macroDefaultParams(cmdId) {
    const params = [null, null, null, null, null, null, null];
    for (const f of macroCommandFields(cmdId)) params[f.i - 1] = f.def;
    return params;
}

function macroNewTask(cmdId, position, index) {
    return {
        position: position || "after",
        index: index === undefined ? 1 : index,
        command: cmdId,
        params: macroDefaultParams(cmdId),
    };
}

function macroEmpty(vehicle) {
    return { name: "", vehicle: vehicle || "rover", tasks: [] };
}

// Normalize a WP index value: positive integer, "last", or null (invalid).
function macroParseIndex(val) {
    if (typeof val === "number") {
        return Number.isInteger(val) && val >= 1 ? val : null;
    }
    if (typeof val !== "string") return null;
    const s = val.trim().toLowerCase();
    if (s === "last") return "last";
    if (!/^\d+$/.test(s)) return null;
    const n = Number.parseInt(s, 10);
    return n >= 1 ? n : null;
}

function macroCommandName(cmdId) {
    if (!cmdId) return "Select command...";
    return MACRO_COMMANDS[cmdId]?.name || `MAV_CMD ${cmdId}`;
}

// Resolve each task's anchor slot: { position: "before" | "after", index } or null.
// "then" rows inherit the anchor of the row above.
function macroResolveAnchors(tasks) {
    const anchors = [];
    let prev = null;
    for (const t of tasks) {
        if (t.position === "then") {
            anchors.push(prev);
        } else if (t.position === "before" || t.position === "after") {
            prev = { position: t.position, index: t.index };
            anchors.push(prev);
        } else {
            prev = null;
            anchors.push(null);
        }
    }
    return anchors;
}

// Static validation (independent of the path). Returns [{ task (0-based), message }].
function validateMacro(macro) {
    const issues = [];
    const vehicleLabel = MACRO_VEHICLES[macro.vehicle] || macro.vehicle;
    const anchors = macroResolveAnchors(macro.tasks);
    macro.tasks.forEach((t, i) => {
        if (!t.command) {
            issues.push({ task: i, message: "no command selected" });
        } else if (!MACRO_COMMANDS[t.command]) {
            issues.push({ task: i, message: `unknown command ${t.command}` });
        } else if (!macroCommandSupported(t.command, macro.vehicle)) {
            issues.push({
                task: i,
                message: `${macroCommandName(t.command)} is not supported on ${vehicleLabel}`,
            });
        }
        if (t.position === "then") {
            if (!anchors[i]) {
                issues.push({
                    task: i,
                    message: "'Then' needs a Before/After task above it",
                });
            }
        } else if (t.position !== "before" && t.position !== "after") {
            issues.push({
                task: i,
                message: `invalid position '${t.position}'`,
            });
        } else if (macroParseIndex(t.index) === null) {
            issues.push({ task: i, message: "invalid waypoint index" });
        }
    });
    return issues;
}

function fmtParam(v) {
    if (v === null || v === undefined || !Number.isFinite(Number(v)))
        return "0";
    return String(Number(Number(v).toFixed(6)));
}

// Expand a single path (array of [lat, lon]) into QGC WPL 110 lines,
// inserting macro items. macro may be null (plain export).
// Returns { lines, macroCount, rangeIssues: [{ task, message }] }.
function buildMissionLines(path, macro, altitudeM) {
    if (!path || path.length === 0) {
        return { lines: [], macroCount: 0, rangeIssues: [] };
    }
    const alt = Number(altitudeM) || 0;
    const n = path.length;
    const [hLat, hLon] = path[0];
    const wpLine = (seq, lat, lon) =>
        `${seq}\t0\t3\t16\t0\t0\t0\t0\t${lat.toFixed(7)}\t${lon.toFixed(7)}\t${alt.toFixed(6)}\t1`;

    // gap g = position between waypoint g and waypoint g+1 (gap 0 = before WP 1,
    // gap n = after the last waypoint). Within a gap, "after" items precede
    // "before" items; ties keep list order.
    const inserts = [];
    const rangeIssues = [];
    if (macro?.tasks?.length) {
        const invalid = new Set(validateMacro(macro).map((x) => x.task));
        const anchors = macroResolveAnchors(macro.tasks);
        macro.tasks.forEach((t, i) => {
            if (invalid.has(i)) return;
            const anchor = anchors[i];
            const idx = macroParseIndex(anchor.index);
            if (idx === null) return; // invalid anchor (flagged on the anchor row)
            const wp = idx === "last" ? n : idx;
            if (wp > n) {
                rangeIssues.push({
                    task: i,
                    message: `waypoint ${wp} exceeds ${n} waypoints`,
                });
                return;
            }
            const gap = anchor.position === "after" ? wp : wp - 1;
            const sub = anchor.position === "after" ? 0 : 1;
            inserts.push({ gap, sub, order: i, task: t });
        });
        inserts.sort(
            (a, b) => a.gap - b.gap || a.sub - b.sub || a.order - b.order,
        );
    }

    const macroLine = (seq, t) => {
        const def = MACRO_COMMANDS[t.command];
        const p = t.params || [];
        const cols = [];
        for (let k = 0; k < 7; k++) cols.push(fmtParam(p[k]));
        if (def.loc) {
            cols[4] = (Number(p[4]) || 0).toFixed(7);
            cols[5] = (Number(p[5]) || 0).toFixed(7);
        }
        if (def.loc || def.alt) {
            const a = p[6] === null || p[6] === undefined ? alt : Number(p[6]);
            cols[6] = (Number.isFinite(a) ? a : alt).toFixed(6);
        }
        return `${seq}\t0\t3\t${t.command}\t${cols.join("\t")}\t1`;
    };

    const lines = [
        "QGC WPL 110",
        `0\t1\t0\t16\t0\t0\t0\t0\t${hLat.toFixed(7)}\t${hLon.toFixed(7)}\t${alt.toFixed(6)}\t1`,
    ];
    let seq = 1;
    let k = 0;
    const flushGap = (gap) => {
        while (k < inserts.length && inserts[k].gap === gap) {
            lines.push(macroLine(seq++, inserts[k].task));
            k++;
        }
    };
    flushGap(0);
    for (let w = 1; w <= n; w++) {
        const [lat, lon] = path[w - 1];
        lines.push(wpLine(seq++, lat, lon));
        flushGap(w);
    }
    return { lines, macroCount: inserts.length, rangeIssues };
}

// ── JSON (de)serialization ──

function macroToJson(macro) {
    return {
        format: MACRO_FORMAT,
        version: MACRO_VERSION,
        name: macro.name || "",
        vehicle: macro.vehicle,
        tasks: macro.tasks.map((t) => ({
            position: t.position,
            index: t.position === "then" ? null : t.index,
            command: t.command,
            params: t.params.map((v) =>
                v === null || v === undefined ? null : Number(v),
            ),
        })),
    };
}

function normalizeTask(raw) {
    if (!raw || typeof raw !== "object") {
        throw new Error("Invalid task entry in macro file.");
    }
    const position = ["before", "after", "then"].includes(raw?.position)
        ? raw.position
        : String(raw?.position ?? "");
    const command = raw?.command ? Number(raw.command) : null;
    const params = [null, null, null, null, null, null, null];
    if (Array.isArray(raw?.params)) {
        for (let k = 0; k < 7; k++) {
            const v = raw.params[k];
            params[k] =
                v === null || v === undefined || !Number.isFinite(Number(v))
                    ? null
                    : Number(v);
        }
    }
    let index = raw?.index ?? null;
    if (typeof index === "string" && macroParseIndex(index) === "last") {
        index = "last";
    }
    return { position, index, command, params };
}

// Parse JSON text (or an already-parsed object) into a macro. Throws on error.
function parseMacroJson(input) {
    let obj = input;
    if (typeof input === "string") {
        try {
            obj = JSON.parse(input);
        } catch (_e) {
            throw new Error("File is not valid JSON.");
        }
    }
    if (!obj || typeof obj !== "object" || obj.format !== MACRO_FORMAT) {
        throw new Error("Not a YardPilot macro file (missing format tag).");
    }
    if (obj.version !== MACRO_VERSION) {
        throw new Error(`Unsupported macro version: ${obj.version}.`);
    }
    if (!MACRO_VEHICLES[obj.vehicle]) {
        throw new Error(`Unknown vehicle type: ${obj.vehicle}.`);
    }
    if (!Array.isArray(obj.tasks)) {
        throw new Error("Macro file has no task list.");
    }
    return {
        name: typeof obj.name === "string" ? obj.name : "",
        vehicle: obj.vehicle,
        tasks: obj.tasks.map(normalizeTask),
    };
}

if (typeof window !== "undefined") {
    window.MACRO_FORMAT = MACRO_FORMAT;
    window.MACRO_VERSION = MACRO_VERSION;
    window.MACRO_VEHICLES = MACRO_VEHICLES;
    window.MACRO_VEHICLE_CODES = MACRO_VEHICLE_CODES;
    window.MACRO_COMMANDS = MACRO_COMMANDS;
    window.macroCommandSupported = macroCommandSupported;
    window.macroCommandsForVehicle = macroCommandsForVehicle;
    window.macroCommandFields = macroCommandFields;
    window.macroDefaultParams = macroDefaultParams;
    window.macroNewTask = macroNewTask;
    window.macroEmpty = macroEmpty;
    window.macroParseIndex = macroParseIndex;
    window.macroCommandName = macroCommandName;
    window.macroResolveAnchors = macroResolveAnchors;
    window.validateMacro = validateMacro;
    window.buildMissionLines = buildMissionLines;
    window.macroToJson = macroToJson;
    window.parseMacroJson = parseMacroJson;
}
