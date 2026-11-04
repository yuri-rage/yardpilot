const map = L.map("map", {
    center: [39.8283, -98.5795],
    zoom: 5,
    zoomControl: true,
    maxZoom: 23,
});

const satellite = L.tileLayer(
    "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
    {
        attribution: "Map data &copy; Google",
        maxNativeZoom: 21,
        maxZoom: 23,
        className: "map-tile-satellite",
    },
).addTo(map);

const labels = L.tileLayer(
    "https://mt1.google.com/vt/lyrs=h&x={x}&y={y}&z={z}",
    {
        maxNativeZoom: 21,
        maxZoom: 23,
        pane: "overlayPane",
    },
).addTo(map);

const satelliteToggle = document.getElementById("layer-satellite");
const labelsToggle = document.getElementById("layer-labels");
const brightnessInput = document.getElementById("map-brightness");
const brightnessVal = document.getElementById("brightness-val");
document.getElementById("map").style.setProperty("--map-brightness", "75%");

satelliteToggle.addEventListener("change", () => {
    satelliteToggle.checked
        ? map.addLayer(satellite)
        : map.removeLayer(satellite);
    saveSettings();
});
labelsToggle.addEventListener("change", () => {
    labelsToggle.checked ? map.addLayer(labels) : map.removeLayer(labels);
    saveSettings();
});
brightnessInput.addEventListener("input", (e) => {
    const val = e.target.value;
    brightnessVal.textContent = `${val}%`;
    document
        .getElementById("map")
        .style.setProperty("--map-brightness", `${val}%`);
    saveSettings();
});

// ── Unit conversion constants ─────────────────────────────

const FT_PER_M = 3.28084;
const MPH_PER_MS = 2.23694;
const M_PER_FT = 1 / FT_PER_M;
const MS_PER_MPH = 1 / MPH_PER_MS;

// Current unit mode — 'metric' | 'imperial'
let currentUnits = "metric";

function isImperial() {
    return currentUnits === "imperial";
}

// Convert a stored metric value → display value
function toDisplay(metricVal, type) {
    if (!isImperial()) return metricVal;
    if (type === "length") return Math.round(metricVal * FT_PER_M * 100) / 100; // 2 dp in ft
    if (type === "speed") return Math.round(metricVal * MPH_PER_MS * 100) / 100; // 2 dp in mph
    return metricVal;
}

// Convert a display value → metric storage value
function toMetric(displayVal, type) {
    if (!isImperial()) return displayVal;
    if (type === "length") return displayVal * M_PER_FT;
    if (type === "speed") return displayVal * MS_PER_MPH;
    return displayVal;
}

// ── Unit-aware input helpers ──────────────────────────────

// Configuration for each unit-sensitive numeric input:
//   type          – 'length' | 'speed'
//   step_m        – step when metric
//   step_ft / step_mph – step when imperial
//   decimals_m    – display decimal places when metric
//   decimals_imp  – display decimal places when imperial
const UNIT_INPUTS = {
    //                                              step_m  step_imp  dec_m  dec_imp
    "input-lane-width": {
        type: "length",
        step_m: 0.01,
        step_imp: 0.05,
        dec_m: 2,
        dec_imp: 2,
        labelEl: null,
        metricLabel: "Lane width (m)",
        imperialLabel: "Lane width (ft)",
    },
    "input-exclusion-buffer": {
        type: "length",
        step_m: 0.1,
        step_imp: 0.5,
        dec_m: 1,
        dec_imp: 1,
        labelEl: null,
        metricLabel: "Exclusion buffer (m)",
        imperialLabel: "Exclusion buffer (ft)",
    },
    "input-transition-tolerance": {
        type: "length",
        step_m: 0.05,
        step_imp: 0.1,
        dec_m: 2,
        dec_imp: 2,
        labelEl: null,
        metricLabel: "Transition tolerance (m)",
        imperialLabel: "Transition tolerance (ft)",
    },
    "input-combine-waypoints": {
        type: "length",
        step_m: 0.01,
        step_imp: 0.05,
        dec_m: 2,
        dec_imp: 2,
        labelEl: null,
        metricLabel: "Combine waypoints within (m)",
        imperialLabel: "Combine waypoints within (ft)",
    },
    "input-headland-margin": {
        type: "length",
        step_m: 0.05,
        step_imp: 0.1,
        dec_m: 2,
        dec_imp: 2,
        labelEl: null,
        metricLabel: "Headland margin (m)",
        imperialLabel: "Headland margin (ft)",
    },
    "input-target-speed": {
        type: "speed",
        step_m: 0.1,
        step_imp: 0.1,
        dec_m: 1,
        dec_imp: 1,
        labelEl: "label-target-speed",
        metricLabel: "Target speed (m/s)",
        imperialLabel: "Target speed (mph)",
    },
    "input-altitude": {
        type: "length",
        step_m: 0.5,
        step_imp: 5,
        dec_m: 1,
        dec_imp: 0,
        labelEl: "label-altitude",
        metricLabel: "Altitude (m)",
        imperialLabel: "Altitude (ft)",
    },
};

// Read the raw metric value from a unit-sensitive input (using current unit mode)
function getMetricValue(id) {
    const cfg = UNIT_INPUTS[id];
    const raw = Number.parseFloat(document.getElementById(id).value) || 0;
    return toMetric(raw, cfg.type);
}

// Snapshot all unit-sensitive inputs as metric values from the DOM right now
function snapshotMetricValues() {
    const snap = {};
    for (const id of Object.keys(UNIT_INPUTS)) {
        snap[id] = getMetricValue(id);
    }
    return snap;
}

// Set the display value of a unit-sensitive input from a metric value
function setDisplayValue(id, metricVal) {
    const cfg = UNIT_INPUTS[id];
    const displayVal = toDisplay(metricVal, cfg.type);
    const decimals = isImperial() ? cfg.dec_imp : cfg.dec_m;
    document.getElementById(id).value = Number.parseFloat(
        displayVal.toFixed(decimals),
    );
}

// Refresh all unit-aware inputs: convert displayed values and update labels/steps.
// metricSnapshot is an optional {id: metricVal} map taken BEFORE currentUnits changed.
function applyUnitsToInputs(metricSnapshot) {
    for (const [id, cfg] of Object.entries(UNIT_INPUTS)) {
        const el = document.getElementById(id);
        // Use snapshot if provided, otherwise convert current display back to metric
        const metricVal = metricSnapshot
            ? metricSnapshot[id]
            : getStoredMetricValue(id);
        const displayVal = toDisplay(metricVal, cfg.type);
        const decimals = isImperial() ? cfg.dec_imp : cfg.dec_m;
        el.value = Number.parseFloat(displayVal.toFixed(decimals));
        el.step = isImperial() ? cfg.step_imp : cfg.step_m;

        // Update label
        const labelText = isImperial() ? cfg.imperialLabel : cfg.metricLabel;
        if (cfg.labelEl) {
            document.getElementById(cfg.labelEl).textContent = labelText;
        } else {
            const parent = el.closest("label") || el.parentElement;
            const span = parent ? parent.querySelector("span") : null;
            if (span) span.textContent = labelText;
        }
    }
    validateCombineWaypoints();
}

function validateCombineWaypoints() {
    const combineDist = getMetricValue("input-combine-waypoints");
    const laneWidth = getMetricValue("input-lane-width");
    const warningEl = document.getElementById("combine-waypoints-warning");
    if (warningEl) {
        if (combineDist > 0 && combineDist >= laneWidth) {
            warningEl.style.display = "block";
        } else {
            warningEl.style.display = "none";
        }
    }
}

// ── LocalStorage persistence ─────────────────────────────

// Settings are always stored in METRIC regardless of current unit mode.
function saveSettings() {
    const settings = {
        laneWidth: getMetricValue("input-lane-width"),
        exclusionBuffer: getMetricValue("input-exclusion-buffer"),
        transitionTolerance: getMetricValue("input-transition-tolerance"),
        headlandMargin: getMetricValue("input-headland-margin"),
        combineWaypoints: getMetricValue("input-combine-waypoints"),
        skipLanes:
            Number.parseInt(
                document.getElementById("input-skip-lanes").value,
                10,
            ) || 0,
        targetSpeed: getMetricValue("input-target-speed"),
        altitude: getMetricValue("input-altitude"),
        sweepAuto: document.getElementById("checkbox-sweep-auto").checked,
        sweepAngle: document.getElementById("input-custom-sweep-angle").value,
        perimeterPasses: document.getElementById("input-perimeter-passes")
            .value,
        perimeterDirection: document.getElementById(
            "select-perimeter-direction",
        ).value,
        circleSegments:
            Number.parseInt(
                document.getElementById("input-circle-segments").value,
                10,
            ) || 64,
        layerSatellite: satelliteToggle.checked,
        layerLabels: labelsToggle.checked,
        mapBrightness: brightnessInput.value,
        spiralMode: document.getElementById("checkbox-spiral-mode").checked,
        reverseSpiral: document.getElementById("checkbox-reverse-spiral")
            .checked,
        units: currentUnits,
    };
    localStorage.setItem("yardpilot_settings", JSON.stringify(settings));
    validateCombineWaypoints();
}

function updateSpiralModeUI() {
    const isSpiral = document.getElementById("checkbox-spiral-mode").checked;

    // Disable/enable inputs
    document.getElementById("input-skip-lanes").disabled = isSpiral;
    document.getElementById("checkbox-sweep-auto").disabled = isSpiral;
    document.getElementById("checkbox-reverse-spiral").disabled = !isSpiral;

    const sweepAuto = document.getElementById("checkbox-sweep-auto").checked;
    document.getElementById("input-custom-sweep-angle").disabled =
        isSpiral || sweepAuto;

    // Adjust opacities of rows
    document.getElementById("row-skip-lanes").style.opacity = isSpiral
        ? "0.35"
        : "1.0";
    document.getElementById("row-sweep-auto").style.opacity = isSpiral
        ? "0.35"
        : "1.0";
    customSweepAngleRow.style.opacity = isSpiral || sweepAuto ? "0.35" : "1.0";
    document.getElementById("row-reverse-spiral").style.opacity = isSpiral
        ? "1.0"
        : "0.35";

    // Perimeter passes is always active now since it specifies reversing threshold
    document.getElementById("input-perimeter-passes").disabled = false;
    document.getElementById("row-perimeter-passes").style.opacity = "1.0";
}

// Return the stored METRIC value for a unit-sensitive input (from localStorage)
function getStoredMetricValue(id) {
    const data = localStorage.getItem("yardpilot_settings");
    if (!data) {
        // Fall back to converting the current display value back to metric
        const cfg = UNIT_INPUTS[id];
        const raw = Number.parseFloat(document.getElementById(id).value) || 0;
        return toMetric(raw, cfg.type);
    }
    try {
        const settings = JSON.parse(data);
        const keyMap = {
            "input-lane-width": "laneWidth",
            "input-exclusion-buffer": "exclusionBuffer",
            "input-transition-tolerance": "transitionTolerance",
            "input-combine-waypoints": "combineWaypoints",
            "input-target-speed": "targetSpeed",
            "input-altitude": "altitude",
            "input-headland-margin": "headlandMargin",
        };
        const key = keyMap[id];
        if (key !== undefined && settings[key] !== undefined)
            return Number.parseFloat(settings[key]);
    } catch (_e) {}
    const cfg = UNIT_INPUTS[id];
    const raw = Number.parseFloat(document.getElementById(id).value) || 0;
    return toMetric(raw, cfg.type);
}

function loadSettings() {
    const data = localStorage.getItem("yardpilot_settings");
    if (!data) return;
    try {
        const settings = JSON.parse(data);

        // Restore unit mode first so conversions are correct
        if (settings.units) {
            currentUnits = settings.units;
            document
                .getElementById("units-metric")
                .classList.toggle("active", currentUnits === "metric");
            document
                .getElementById("units-imperial")
                .classList.toggle("active", currentUnits === "imperial");
        }

        // Restore metric values, display in current unit
        if (settings.laneWidth !== undefined)
            setDisplayValue("input-lane-width", settings.laneWidth);
        if (settings.exclusionBuffer !== undefined)
            setDisplayValue("input-exclusion-buffer", settings.exclusionBuffer);
        if (settings.transitionTolerance !== undefined)
            setDisplayValue(
                "input-transition-tolerance",
                settings.transitionTolerance,
            );
        if (settings.targetSpeed !== undefined)
            setDisplayValue("input-target-speed", settings.targetSpeed);
        if (settings.headlandMargin !== undefined)
            setDisplayValue("input-headland-margin", settings.headlandMargin);
        if (settings.altitude !== undefined)
            setDisplayValue("input-altitude", settings.altitude);
        if (settings.combineWaypoints !== undefined)
            setDisplayValue(
                "input-combine-waypoints",
                settings.combineWaypoints,
            );

        if (settings.skipLanes !== undefined)
            document.getElementById("input-skip-lanes").value =
                settings.skipLanes;
        if (settings.sweepAuto !== undefined) {
            const autoCheckbox = document.getElementById("checkbox-sweep-auto");
            const angleSlider = document.getElementById(
                "input-custom-sweep-angle",
            );
            const angleRow = document.getElementById("row-custom-sweep-angle");
            autoCheckbox.checked = settings.sweepAuto;
            angleSlider.disabled = settings.sweepAuto;
            angleRow.style.opacity = settings.sweepAuto ? "0.35" : "1.0";
        }
        if (settings.sweepAngle !== undefined) {
            document.getElementById("input-custom-sweep-angle").value =
                settings.sweepAngle;
            document.getElementById("sweep-angle-val").textContent =
                `${settings.sweepAngle}°`;
        }
        if (settings.perimeterPasses !== undefined)
            document.getElementById("input-perimeter-passes").value =
                settings.perimeterPasses;
        if (settings.perimeterDirection !== undefined)
            document.getElementById("select-perimeter-direction").value =
                settings.perimeterDirection;
        if (settings.circleSegments !== undefined)
            document.getElementById("input-circle-segments").value =
                settings.circleSegments;
        if (settings.spiralMode !== undefined) {
            document.getElementById("checkbox-spiral-mode").checked =
                settings.spiralMode;
        }
        if (settings.reverseSpiral !== undefined) {
            document.getElementById("checkbox-reverse-spiral").checked =
                settings.reverseSpiral;
        }
        updateSpiralModeUI();

        if (settings.layerSatellite !== undefined) {
            satelliteToggle.checked = settings.layerSatellite;
            settings.layerSatellite
                ? map.addLayer(satellite)
                : map.removeLayer(satellite);
        }
        if (settings.layerLabels !== undefined) {
            labelsToggle.checked = settings.layerLabels;
            settings.layerLabels
                ? map.addLayer(labels)
                : map.removeLayer(labels);
        }
        if (settings.mapBrightness !== undefined) {
            brightnessInput.value = settings.mapBrightness;
            brightnessVal.textContent = `${settings.mapBrightness}%`;
            document
                .getElementById("map")
                .style.setProperty(
                    "--map-brightness",
                    `${settings.mapBrightness}%`,
                );
        }

        // Apply labels and steps to all unit inputs
        applyUnitsToInputs();
        validateCombineWaypoints();
    } catch (e) {
        console.error("Failed to load settings", e);
    }
}

// ── Units toggle wiring ───────────────────────────────────

document.getElementById("units-metric").addEventListener("click", () => {
    if (currentUnits === "metric") return;
    // Snapshot metric values from DOM before changing mode
    const snap = snapshotMetricValues();
    currentUnits = "metric";
    document.getElementById("units-metric").classList.add("active");
    document.getElementById("units-imperial").classList.remove("active");
    applyUnitsToInputs(snap);
    saveSettings();
    if (lastResult) displayStats(lastResult);
    renderMacroModal();
});

document.getElementById("units-imperial").addEventListener("click", () => {
    if (currentUnits === "imperial") return;
    // Snapshot metric values from DOM before changing mode
    const snap = snapshotMetricValues();
    currentUnits = "imperial";
    document.getElementById("units-metric").classList.remove("active");
    document.getElementById("units-imperial").classList.add("active");
    applyUnitsToInputs(snap);
    saveSettings();
    if (lastResult) displayStats(lastResult);
    renderMacroModal();
});

// ── Zone state ────────────────────────────────────────────

function saveZones() {
    const data = {
        perimeter: zones.perimeter
            ? { name: zones.perimeter.name, coords: zones.perimeter.coords }
            : null,
        exclusions: zones.exclusions.map((z) => ({
            name: z.name,
            shapes: z.shapes,
        })),
    };
    localStorage.setItem("yardpilot_zones", JSON.stringify(data));
}

function loadZones() {
    const data = localStorage.getItem("yardpilot_zones");
    if (!data) return;
    try {
        const parsed = JSON.parse(data);
        if (parsed.perimeter) {
            renderPerimeter(parsed.perimeter.coords, parsed.perimeter.name);
        }
        if (parsed.exclusions?.length) {
            for (const z of zones.exclusions) {
                removeLayerGroup(z.layers);
            }
            zones.exclusions = [];

            for (const z of parsed.exclusions) {
                const layers = z.shapes.map((s) => {
                    if (s.type === "circle") {
                        return L.circle([s.lat, s.lon], {
                            radius: s.radius,
                            ...EXCLUSION_STYLE,
                        }).addTo(map);
                    }
                    return L.polygon(s.vertices, EXCLUSION_STYLE).addTo(map);
                });
                zones.exclusions.push({
                    name: z.name,
                    shapes: z.shapes,
                    layers,
                });
            }
            document.getElementById("drop-exclusion").classList.add("has-file");
            syncUI();
        }
    } catch (e) {
        console.error("Failed to load zones", e);
    }
}

// Attach change listeners to settings inputs
const settingInputs = [
    "input-lane-width",
    "input-exclusion-buffer",
    "input-transition-tolerance",
    "input-skip-lanes",
    "checkbox-sweep-auto",
    "input-custom-sweep-angle",
    "input-perimeter-passes",
    "select-perimeter-direction",
    "input-circle-segments",
    "checkbox-spiral-mode",
    "checkbox-reverse-spiral",
    "input-headland-margin",
    "input-combine-waypoints",
];
for (const id of settingInputs) {
    document.getElementById(id).addEventListener("change", saveSettings);
}

// Update slider state dynamically when checkbox is toggled
const sweepAutoCheckbox = document.getElementById("checkbox-sweep-auto");
const sweepAngleInput = document.getElementById("input-custom-sweep-angle");
const sweepAngleVal = document.getElementById("sweep-angle-val");
const customSweepAngleRow = document.getElementById("row-custom-sweep-angle");
const spiralModeCheckbox = document.getElementById("checkbox-spiral-mode");
const reverseSpiralCheckbox = document.getElementById(
    "checkbox-reverse-spiral",
);

sweepAutoCheckbox.addEventListener("change", () => {
    updateSpiralModeUI();
    saveSettings();
});

spiralModeCheckbox.addEventListener("change", () => {
    updateSpiralModeUI();
    saveSettings();
});

reverseSpiralCheckbox.addEventListener("change", () => {
    updateSpiralModeUI();
    saveSettings();
});

sweepAngleInput.addEventListener("input", (e) => {
    sweepAngleVal.textContent = `${e.target.value}°`;
    saveSettings();
});

// ── Zone state ────────────────────────────────────────────

const PERIMETER_STYLE = {
    color: "#3fb950",
    fillColor: "#3fb950",
    fillOpacity: 0.08,
    weight: 2,
};
const EXCLUSION_STYLE = {
    color: "#f85149",
    fillColor: "#f85149",
    fillOpacity: 0.2,
    weight: 2,
};
const PATH_STYLE = { color: "#58a6ff", weight: 1.5, opacity: 0.9 };

const zones = {
    perimeter: null, // { name, coords, layers }
    exclusions: [], // [{ name, shapes, layers }]
};
let pathLayer = null;
let pathWaypoints = null;
let startMarker = null;
let endMarker = null;
// Store last result so stats can be refreshed when units change
let lastResult = null;

function removeLayerGroup(layers) {
    for (const l of layers) {
        map.removeLayer(l);
    }
}

function clearPerimeter() {
    if (!zones.perimeter) return;
    removeLayerGroup(zones.perimeter.layers);
    zones.perimeter = null;
    document.getElementById("drop-perimeter").classList.remove("has-file");
    saveZones();
}

function clearExclusions() {
    for (const z of zones.exclusions) {
        removeLayerGroup(z.layers);
    }
    zones.exclusions = [];
    document.getElementById("drop-exclusion").classList.remove("has-file");
    saveZones();
}

function clearPath() {
    if (pathLayer) {
        map.removeLayer(pathLayer);
        pathLayer = null;
    }
    if (startMarker) {
        map.removeLayer(startMarker);
        startMarker = null;
    }
    if (endMarker) {
        map.removeLayer(endMarker);
        endMarker = null;
    }
    pathWaypoints = null;
    lastResult = null;
    const statsEl = document.getElementById("stats-panel");
    if (statsEl) statsEl.style.display = "none";
}

function renderPerimeter(coords, name) {
    clearPerimeter();
    clearPath();
    const poly = L.polygon(coords, PERIMETER_STYLE).addTo(map);
    zones.perimeter = { name, coords, layers: [poly] };
    map.fitBounds(poly.getBounds(), { padding: [30, 30] });
    document.getElementById("drop-perimeter").classList.add("has-file");
    syncUI();
    saveZones();
}

function renderExclusion(shapes, name) {
    clearExclusions();
    clearPath();
    const layers = shapes.map((s) => {
        if (s.type === "circle") {
            return L.circle([s.lat, s.lon], {
                radius: s.radius,
                ...EXCLUSION_STYLE,
            }).addTo(map);
        }
        return L.polygon(s.vertices, EXCLUSION_STYLE).addTo(map);
    });
    zones.exclusions.push({ name, shapes, layers });
    document.getElementById("drop-exclusion").classList.add("has-file");
    syncUI();
    saveZones();
}

function syncUI() {
    const list = document.getElementById("zone-list");
    list.innerHTML = "";

    const entries = [];
    if (zones.perimeter) {
        entries.push({
            name: zones.perimeter.name,
            color: "#3fb950",
            remove: () => {
                clearPerimeter();
                clearPath();
                syncUI();
            },
        });
    }
    for (let i = 0; i < zones.exclusions.length; i++) {
        const z = zones.exclusions[i];
        entries.push({
            name: z.name,
            color: "#f85149",
            remove: () => {
                removeLayerGroup(z.layers);
                zones.exclusions.splice(i, 1);
                clearPath();
                syncUI();
                saveZones();
            },
        });
    }

    for (const { name, color, remove } of entries) {
        const item = document.createElement("div");
        item.className = "zone-item";
        item.innerHTML = `<span class="zone-dot" style="background:${color}"></span><span class="zone-name" title="${name}">${name}</span><button class="zone-remove" title="Remove">×</button>`;
        item.querySelector(".zone-remove").addEventListener("click", remove);
        list.appendChild(item);
    }

    document.getElementById("btn-generate").disabled = !zones.perimeter;
    document.getElementById("btn-export").disabled = !pathWaypoints;
}

// ── File parsing ──────────────────────────────────────────

function parsePolyFile(text) {
    return text
        .split("\n")
        .map((l) => l.trim())
        .filter((l) => l && !l.startsWith("#"))
        .map((l) => l.split(/\s+/).map(Number))
        .filter(([lat, lon]) => !Number.isNaN(lat) && !Number.isNaN(lon));
}

function parseWaypointRows(text) {
    const lines = text
        .split("\n")
        .map((l) => l.trim())
        .filter(Boolean);
    if (!lines[0].startsWith("QGC WPL")) return null;
    return lines.slice(1).map((line) => {
        const p = line.split("\t");
        return {
            seq: +p[0],
            cmd: +p[3],
            param1: +p[4],
            lat: +p[8],
            lon: +p[9],
        };
    });
}

function waypointsToPerimeterCoords(rows) {
    return rows
        .filter((r) => r.seq > 0 && r.cmd === 16)
        .map((r) => [r.lat, r.lon]);
}

function waypointsToExclusionShapes(rows) {
    const fence = rows.filter((r) => r.seq > 0);
    const shapes = [];
    let i = 0;
    while (i < fence.length) {
        const { cmd, param1, lat, lon } = fence[i];
        if (cmd === 5004 || cmd === 5005) {
            shapes.push({ type: "circle", lat, lon, radius: param1 });
            i++;
        } else if (cmd === 5002 || cmd === 5003) {
            const count = Math.round(param1);
            const vertices = fence
                .slice(i, i + count)
                .map((r) => [r.lat, r.lon]);
            shapes.push({ type: "polygon", vertices });
            i += count;
        } else {
            i++;
        }
    }
    return shapes;
}

// ── Drop zone wiring ──────────────────────────────────────

function handleFile(file, type) {
    const reader = new FileReader();
    reader.onload = ({ target: { result: text } }) => {
        const { name } = file;
        if (type === "perimeter") {
            const coords = name.endsWith(".poly")
                ? parsePolyFile(text)
                : (() => {
                      const r = parseWaypointRows(text);
                      return r ? waypointsToPerimeterCoords(r) : null;
                  })();
            if (coords && coords.length >= 3) renderPerimeter(coords, name);
        } else {
            const rows = parseWaypointRows(text);
            if (!rows) return;
            const shapes = waypointsToExclusionShapes(rows);
            if (shapes.length) renderExclusion(shapes, name);
        }
    };
    reader.readAsText(file);
}

function setupDropZone(el, type, accept) {
    const input = Object.assign(document.createElement("input"), {
        type: "file",
        accept,
    });
    input.style.display = "none";
    document.body.appendChild(input);

    el.style.cursor = "pointer";
    el.addEventListener("click", () => input.click());
    input.addEventListener("change", () => {
        const file = input.files[0];
        if (file) handleFile(file, type);
        input.value = "";
    });
    el.addEventListener("dragover", (e) => {
        e.preventDefault();
        el.classList.add("drag-over");
    });
    el.addEventListener("dragleave", () => el.classList.remove("drag-over"));
    el.addEventListener("drop", (e) => {
        e.preventDefault();
        el.classList.remove("drag-over");
        const file = e.dataTransfer.files[0];
        if (file) handleFile(file, type);
    });
}

setupDropZone(
    document.getElementById("drop-perimeter"),
    "perimeter",
    ".poly,.waypoints",
);
setupDropZone(
    document.getElementById("drop-exclusion"),
    "exclusion",
    ".waypoints",
);

// ── Stats display (unit-aware) ────────────────────────────

function displayStats(result) {
    if (!result) return;
    lastResult = result;

    const areaSqM = result.coveredAreaAllSqM || result.coveredAreaSqM;
    const totalDistM = result.totalDistAllM || result.totalDistM;
    const macroInfo = analyzeMacro();
    const totalCount =
        (result.pieces
            ? result.pieces.reduce((s, p) => s + p.count, 0)
            : result.count) + (macroInfo.enabled ? macroInfo.count : 0);
    const sweepDistM = result.sweepDistAllM || result.sweepDistM;
    // Speed is always read back as metric from storage
    const speedMS = getStoredMetricValue("input-target-speed") || 1.5;

    if (isImperial()) {
        // Area: sq ft, and sq miles as sub
        const areaSqFt = areaSqM * 10.7639;
        const areaSqMiles = areaSqM * 3.861e-7;
        const areaVal =
            areaSqFt > 1e6
                ? `${areaSqMiles.toFixed(4)} mi²`
                : `${areaSqFt.toLocaleString(undefined, {
                      maximumFractionDigits: 0,
                  })} ft²`;
        const areaSub = `${(areaSqM * 0.000247105).toFixed(3)} acres`;
        document.getElementById("stat-area").textContent = areaVal;
        document.getElementById("stat-area-sub").textContent = areaSub;

        // Distance: miles
        const distMiles = totalDistM * 0.000621371;
        const distVal =
            distMiles >= 0.1
                ? `${distMiles.toFixed(3)} mi`
                : `${(totalDistM * FT_PER_M).toFixed(0)} ft`;
        document.getElementById("stat-distance").textContent = distVal;
        document.getElementById("stat-distance-sub").textContent =
            `${totalCount} waypoints`;

        // Duration: uses m/s internally
        const durationSec = totalDistM / speedMS;
        const durationMin = Math.floor(durationSec / 60);
        const durationSecRem = Math.round(durationSec % 60);
        const durVal =
            durationMin > 0
                ? `${durationMin}m ${durationSecRem}s`
                : `${durationSecRem}s`;
        const speedMph = speedMS * MPH_PER_MS;
        document.getElementById("stat-duration").textContent = durVal;
        document.getElementById("stat-duration-sub").textContent =
            `at ${speedMph.toFixed(1)} mph`;
    } else {
        // Metric display
        const areaVal =
            areaSqM > 10000
                ? `${(areaSqM / 10000).toFixed(2)} ha`
                : `${areaSqM.toLocaleString(undefined, {
                      maximumFractionDigits: 0,
                  })} m²`;
        const areaSub = `${(areaSqM * 0.000247105).toFixed(2)} acres`;
        document.getElementById("stat-area").textContent = areaVal;
        document.getElementById("stat-area-sub").textContent = areaSub;

        const distVal =
            totalDistM > 1000
                ? `${(totalDistM / 1000).toFixed(2)} km`
                : `${totalDistM.toFixed(0)} m`;
        document.getElementById("stat-distance").textContent = distVal;
        document.getElementById("stat-distance-sub").textContent =
            `${totalCount} waypoints`;

        const durationSec = totalDistM / speedMS;
        const durationMin = Math.floor(durationSec / 60);
        const durationSecRem = Math.round(durationSec % 60);
        const durVal =
            durationMin > 0
                ? `${durationMin}m ${durationSecRem}s`
                : `${durationSecRem}s`;
        document.getElementById("stat-duration").textContent = durVal;
        document.getElementById("stat-duration-sub").textContent =
            `at ${speedMS.toFixed(1)} m/s`;
    }

    const efficiency =
        totalDistM > 0
            ? `${((sweepDistM / totalDistM) * 100).toFixed(0)}%`
            : "0%";
    document.getElementById("stat-efficiency").textContent = efficiency;

    const warningsEl = document.getElementById("stats-warnings");
    if (warningsEl) {
        const displayWarnings = [...(result.warnings || [])];
        if (result.pieces && result.pieces.length > 1) {
            displayWarnings.unshift(
                `Split yard: ${result.pieces.length} disconnected pieces planned. Export will generate separate missions for each piece.`,
            );
        }
        if (macroInfo.enabled) displayWarnings.push(...macroInfo.messages);
        if (displayWarnings.length > 0) {
            warningsEl.replaceChildren(
                ...displayWarnings.map((w) => {
                    const item = document.createElement("div");
                    item.className = "warning-item";
                    item.textContent = `⚠️ ${w}`;
                    return item;
                }),
            );
            warningsEl.style.display = "block";
        } else {
            warningsEl.style.display = "none";
            warningsEl.replaceChildren();
        }
    }

    document.getElementById("stats-panel").style.display = "block";
}

// ── Path generation ───────────────────────────────────────

document.getElementById("btn-generate").addEventListener("click", () => {
    if (!zones.perimeter) return;

    // All values retrieved as metric regardless of display unit
    const laneWidth = getMetricValue("input-lane-width") || 1.2;
    const buffer = getMetricValue("input-exclusion-buffer") || 1.0;
    const headlandMargin = getMetricValue("input-headland-margin") || 0;
    const tolerance = getMetricValue("input-transition-tolerance") || 0;
    const skipLanes =
        Number.parseInt(
            document.getElementById("input-skip-lanes").value,
            10,
        ) || 0;
    const sweepAuto = document.getElementById("checkbox-sweep-auto").checked;
    const sweepMode = sweepAuto ? "auto" : "custom";
    const sweepAngle =
        Number.parseFloat(
            document.getElementById("input-custom-sweep-angle").value,
        ) || 0;
    const nPasses =
        Number.parseInt(
            document.getElementById("input-perimeter-passes").value,
            10,
        ) || 0;
    const direction =
        document.getElementById("select-perimeter-direction").value || "CW";
    const allShapes = zones.exclusions.flatMap((z) => z.shapes);

    const circleSegments =
        Number.parseInt(
            document.getElementById("input-circle-segments").value,
            10,
        ) || 64;

    const spiralMode = document.getElementById("checkbox-spiral-mode").checked;
    const reverseSpiral = document.getElementById(
        "checkbox-reverse-spiral",
    ).checked;

    const combineWaypoints = getMetricValue("input-combine-waypoints") || 0;

    const result = generateCoveragePath(
        zones.perimeter.coords,
        allShapes,
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
    );
    if (!result) return;

    clearPath();
    pathWaypoints = result.path;
    if (result.pieces && result.pieces.length > 1) {
        const polylineLayers = result.pieces.map((p) =>
            L.polyline(p.path, PATH_STYLE),
        );
        pathLayer = L.featureGroup(polylineLayers).addTo(map);

        const markers = [];
        result.pieces.forEach((p, idx) => {
            if (p.path.length > 0) {
                const sPt = p.path[0];
                const ePt = p.path[p.path.length - 1];
                markers.push(
                    L.circleMarker(sPt, {
                        radius: 6,
                        fillColor: "#10b981",
                        color: "#ffffff",
                        weight: 1.5,
                        fillOpacity: 1,
                    }).bindPopup(`<b>Start Piece ${idx + 1}</b>`),
                );
                markers.push(
                    L.circleMarker(ePt, {
                        radius: 6,
                        fillColor: "#ef4444",
                        color: "#ffffff",
                        weight: 1.5,
                        fillOpacity: 1,
                    }).bindPopup(`<b>End Piece ${idx + 1}</b>`),
                );
            }
        });
        startMarker = L.featureGroup(markers).addTo(map);
    } else {
        pathLayer = L.polyline(result.path, PATH_STYLE).addTo(map);

        if (result.path.length > 0) {
            const startPt = result.path[0];
            const endPt = result.path[result.path.length - 1];

            startMarker = L.circleMarker(startPt, {
                radius: 6,
                fillColor: "#10b981",
                color: "#ffffff",
                weight: 1.5,
                fillOpacity: 1,
            })
                .bindPopup("<b>Start Coverage</b>")
                .addTo(map);

            endMarker = L.circleMarker(endPt, {
                radius: 6,
                fillColor: "#ef4444",
                color: "#ffffff",
                weight: 1.5,
                fillOpacity: 1,
            })
                .bindPopup("<b>End Coverage</b>")
                .addTo(map);
        }
    }

    displayStats(result);
    syncUI();
});

// ── Waypoint export ───────────────────────────────────────

document.getElementById("btn-export").addEventListener("click", () => {
    if (!pathWaypoints) return;
    // Altitude is always exported in meters — convert from display unit to metric
    const altitudeM = getMetricValue("input-altitude") || 0.0;
    const macro = activeMacro();

    getPathPieces().forEach((piece, idx) => {
        if (!piece.path || piece.path.length === 0) return;
        const { lines } = buildMissionLines(piece.path, macro, altitudeM);
        const filename = (() => {
            if (zones.perimeter?.name) {
                const rawName = zones.perimeter.name;
                const lastDotIdx = rawName.lastIndexOf(".");
                const basename =
                    lastDotIdx !== -1
                        ? rawName.substring(0, lastDotIdx)
                        : rawName;
                return `yp_${basename}${piece.suffix}.waypoints`;
            }
            return `yardpilot${piece.suffix}.waypoints`;
        })();

        const blob = new Blob([lines.join("\n")], { type: "text/plain" });
        const url = URL.createObjectURL(blob);
        const a = Object.assign(document.createElement("a"), {
            href: url,
            download: filename,
        });
        setTimeout(() => {
            a.click();
            setTimeout(() => URL.revokeObjectURL(url), 2000);
        }, idx * 250);
    });
});

// ── Mission macro ─────────────────────────────────────────
// Command tables, validation, and mission expansion live in macros.js.
// Macro params are stored in METRIC; dist/alt/speed fields are converted for display.

const MACRO_STORAGE_KEY = "yardpilot_macro";
const macroState = { macro: macroEmpty("rover"), enabled: false };

const macroModal = document.getElementById("macro-modal");
const macroDropzone = document.getElementById("macro-dropzone");
const macroNameInput = document.getElementById("macro-name");
const macroVehicleSelect = document.getElementById("macro-vehicle");
const macroEnabledInput = document.getElementById("macro-enabled");
const macroTaskList = document.getElementById("macro-task-list");

function loadMacro() {
    const data = localStorage.getItem(MACRO_STORAGE_KEY);
    if (!data) return;
    try {
        const obj = JSON.parse(data);
        macroState.macro = parseMacroJson(obj);
        macroState.enabled = !!obj.enabled;
    } catch (e) {
        console.error("Failed to load macro", e);
    }
}

function saveMacro() {
    localStorage.setItem(
        MACRO_STORAGE_KEY,
        JSON.stringify({
            ...macroToJson(macroState.macro),
            enabled: macroState.enabled,
        }),
    );
}

// The macro applied on export, or null when disabled/empty
function activeMacro() {
    return macroState.enabled && macroState.macro.tasks.length
        ? macroState.macro
        : null;
}

// Paths that will be exported (one per disconnected piece)
function getPathPieces() {
    if (!pathWaypoints) return [];
    return lastResult?.pieces && lastResult.pieces.length > 1
        ? lastResult.pieces.map((p, idx) => ({
              path: p.path,
              suffix: `_piece${idx + 1}`,
              label: `piece ${idx + 1}`,
          }))
        : [{ path: pathWaypoints, suffix: "", label: "" }];
}

// Validate the macro and dry-run it against the current path pieces.
// Returns { enabled, count, messages, rows: Map(taskIdx → { level, messages }) }
function analyzeMacro() {
    const macro = macroState.macro;
    const rows = new Map();
    const messages = [];
    const label = (i) =>
        `Macro task ${i + 1} (${macroCommandName(macro.tasks[i].command)})`;
    const flag = (task, level, msg) => {
        const r = rows.get(task) || { level, messages: [] };
        if (level === "error") r.level = "error";
        r.messages.push(msg);
        rows.set(task, r);
        const prefix = level === "error" ? "Skipped" : "Check";
        messages.push(`${label(task)} (${prefix}): ${msg}`);
    };
    for (const issue of validateMacro(macro)) {
        flag(issue.task, "error", issue.message);
    }
    // Check if any "then" task has an anchor row that is invalid or skipped
    const anchors = macroResolveAnchors(macro.tasks);
    macro.tasks.forEach((t, i) => {
        if (t.position === "then") {
            const anchorIdx = macro.tasks.findIndex(
                (other, idx) =>
                    idx < i &&
                    anchors[idx] === anchors[i] &&
                    other.position !== "then",
            );
            if (anchorIdx !== -1 && rows.get(anchorIdx)?.level === "error") {
                const anchorCmd = macroCommandName(
                    macro.tasks[anchorIdx].command,
                );
                flag(
                    i,
                    "warn",
                    `preceding task ${anchorIdx + 1} (${anchorCmd}) has an error`,
                );
            }
        }
    });
    let count = 0;
    for (const piece of getPathPieces()) {
        if (!piece.path?.length) continue;
        const r = buildMissionLines(piece.path, macro, 0);
        count += r.macroCount;
        for (const issue of r.rangeIssues) {
            flag(
                issue.task,
                "warn",
                piece.label
                    ? `${issue.message} (${piece.label})`
                    : issue.message,
            );
        }
    }
    return { enabled: !!activeMacro(), count, messages, rows };
}

// ── Unit helpers for macro fields ──

const MACRO_UNIT_TYPES = { dist: "length", alt: "length", speed: "speed" };

function macroUnitLabel(unit) {
    if (unit === "dist" || unit === "alt") return isImperial() ? "ft" : "m";
    if (unit === "speed") return isImperial() ? "mph" : "m/s";
    return unit;
}

function macroValueToDisplay(val, unit) {
    if (val === null || val === undefined) return "";
    const type = MACRO_UNIT_TYPES[unit];
    const disp = type ? toDisplay(val, type) : val;
    return String(Number(Number(disp).toFixed(7)));
}

function macroValueFromDisplay(text, unit) {
    if (text.trim() === "") return null;
    const num = Number.parseFloat(text);
    if (!Number.isFinite(num)) return null;
    const type = MACRO_UNIT_TYPES[unit];
    return type ? toMetric(num, type) : num;
}

// ── Rendering ──

function macroEl(tag, props = {}, children = []) {
    const node = Object.assign(document.createElement(tag), props);
    for (const c of children) node.append(c);
    return node;
}

function updateMacroButton() {
    const { macro, enabled } = macroState;
    const name = macro.name.trim() || "Unnamed";
    const label = enabled ? `Macro: ${name}` : "Macro: Off";
    document.getElementById("btn-macro-label").textContent = label;
    const btn = document.getElementById("btn-macro");
    btn.classList.toggle("active", enabled);
    btn.title = enabled
        ? `Macro "${name}" (${macro.tasks.length} tasks) will be applied on export`
        : "Edit mission macro (not applied on export)";
}

function buildCommandSelect(task) {
    const select = macroEl("select", { className: "macro-cmd" });
    const placeholder = macroEl("option", {
        value: "",
        textContent: "Select command...",
        disabled: true,
        selected: !task.command,
    });
    select.append(placeholder);

    const groups = macroCommandsForVehicle(macroState.macro.vehicle);
    const groupLabels = {
        NAV: "Navigation",
        CONDITION: "Conditions",
        DO: "DO Commands",
    };
    let found = !task.command;
    for (const [key, cmds] of Object.entries(groups)) {
        if (!cmds.length) continue;
        const og = macroEl("optgroup", { label: groupLabels[key] });
        for (const c of cmds) {
            if (c.id === task.command) found = true;
            og.append(
                macroEl("option", {
                    value: String(c.id),
                    textContent: c.name,
                    title: c.desc,
                }),
            );
        }
        select.append(og);
    }
    if (!found) {
        // Keep unsupported/unknown commands visible so the row can be fixed
        select.prepend(
            macroEl("optgroup", { label: "Unsupported" }, [
                macroEl("option", {
                    value: String(task.command),
                    textContent: macroCommandName(task.command),
                }),
            ]),
        );
    }
    select.value = task.command ? String(task.command) : "";
    select.title = task.command
        ? MACRO_COMMANDS[task.command]?.desc || "Unsupported command"
        : "Select a command";
    return select;
}

function buildTaskRow(task, i) {
    const tasks = macroState.macro.tasks;
    const row = macroEl("div", { className: "macro-task" });
    row.dataset.idx = String(i);

    // Line 1: position, index, command, actions
    const pos = macroEl("select", { className: "macro-pos" });
    for (const [value, text] of [
        ["before", "Before"],
        ["after", "After"],
        ["then", "Then"],
    ]) {
        pos.append(
            macroEl("option", {
                value,
                textContent: text,
                disabled: value === "then" && i === 0,
            }),
        );
    }
    pos.value = task.position;
    pos.addEventListener("change", () => {
        if (pos.value !== "then" && macroParseIndex(task.index) === null) {
            const anchor = macroResolveAnchors(tasks)[i];
            task.index = anchor ? anchor.index : 1;
        }
        task.position = pos.value;
        onMacroChanged(true);
    });

    const index = macroEl("input", {
        className: "macro-index",
        type: "text",
        placeholder: "WP #",
        title: "Path waypoint number (1–N) or 'Last'",
        value:
            task.index === "last"
                ? "Last"
                : task.index === null
                  ? ""
                  : String(task.index),
    });
    index.setAttribute("list", "macro-index-options");
    if (task.position === "then") index.classList.add("is-hidden");
    index.addEventListener("input", () => {
        const parsed = macroParseIndex(index.value);
        task.index = parsed === null ? index.value : parsed;
        onMacroChanged();
    });

    const cmd = buildCommandSelect(task);
    cmd.addEventListener("change", () => {
        const id = Number(cmd.value);
        task.command = id;
        task.params = macroDefaultParams(id);
        onMacroChanged(true);
    });

    const actionBtn = (text, title, disabled, onClick) => {
        const b = macroEl("button", {
            type: "button",
            className: "macro-action",
            textContent: text,
            title,
            disabled,
        });
        b.addEventListener("click", onClick);
        return b;
    };
    const actions = macroEl("div", { className: "macro-actions" }, [
        actionBtn("↑", "Move up", i === 0, () => moveMacroTask(i, -1)),
        actionBtn("↓", "Move down", i === tasks.length - 1, () =>
            moveMacroTask(i, 1),
        ),
        actionBtn("✕", "Delete task", false, () => deleteMacroTask(i)),
    ]);
    actions.lastChild.classList.add("macro-action-delete");

    row.append(
        macroEl("div", { className: "macro-task-main" }, [
            macroEl("span", {
                className: "macro-task-num",
                textContent: String(i + 1),
            }),
            pos,
            index,
            cmd,
            actions,
        ]),
    );

    // Line 2: params used by this command
    const fields = macroCommandFields(task.command);
    if (fields.length) {
        const params = macroEl("div", { className: "macro-task-params" });
        for (const f of fields) {
            const unit = macroUnitLabel(f.unit);
            const input = macroEl("input", {
                type: "number",
                step: "any",
                value: macroValueToDisplay(task.params[f.i - 1], f.unit),
                placeholder:
                    f.placeholder ||
                    (f.kind === "alt"
                        ? "sidebar"
                        : f.kind === "lat" || f.kind === "lon"
                          ? "current"
                          : "0"),
            });
            if (f.kind === "lat" || f.kind === "lon") {
                input.classList.add("macro-param-wide");
            }
            input.addEventListener("input", () => {
                task.params[f.i - 1] = macroValueFromDisplay(
                    input.value,
                    f.unit,
                );
                onMacroChanged();
            });
            params.append(
                macroEl("label", { className: "macro-param" }, [
                    macroEl("span", {
                        textContent: unit ? `${f.label} (${unit})` : f.label,
                    }),
                    input,
                ]),
            );
        }
        row.append(params);
    }

    row.append(macroEl("div", { className: "macro-task-issue" }));
    return row;
}

function renderMacroTasks() {
    const { tasks } = macroState.macro;
    macroTaskList.replaceChildren(...tasks.map((t, i) => buildTaskRow(t, i)));
    document.getElementById("macro-empty").style.display = tasks.length
        ? "none"
        : "block";
    refreshMacroFlags();
}

function renderMacroModal() {
    if (!macroVehicleSelect.options.length) {
        for (const [value, text] of Object.entries(MACRO_VEHICLES)) {
            macroVehicleSelect.append(
                macroEl("option", { value, textContent: text }),
            );
        }
    }
    macroNameInput.value = macroState.macro.name;
    macroVehicleSelect.value = macroState.macro.vehicle;
    macroEnabledInput.checked = macroState.enabled;
    renderMacroTasks();
    updateMacroButton();
}

// Update row error/warning flags and the issue list without re-rendering inputs
function refreshMacroFlags() {
    const info = analyzeMacro();
    for (const row of macroTaskList.children) {
        const idx = Number(row.dataset.idx);
        const r = info.rows.get(idx);
        const task = macroState.macro.tasks[idx];
        const isUnselected = !task?.command;

        row.classList.toggle(
            "has-error",
            r?.level === "error" && !isUnselected,
        );
        row.classList.toggle("has-warn", r?.level === "warn");
        row.classList.toggle("is-unselected", isUnselected);

        if (isUnselected) {
            row.querySelector(".macro-task-issue").textContent =
                "Choose a command for this task";
        } else if (r) {
            const prefix = r.level === "error" ? "Skipped" : "Check";
            row.querySelector(".macro-task-issue").textContent =
                `⚠️ ${prefix}: ${r.messages.join("; ")}`;
        } else {
            row.querySelector(".macro-task-issue").textContent = "";
        }
    }
    const issuesEl = document.getElementById("macro-issues");
    issuesEl.replaceChildren(
        ...info.messages.map((m) =>
            macroEl("div", {
                className: "warning-item",
                textContent: `⚠️ ${m}`,
            }),
        ),
    );
    issuesEl.style.display = info.messages.length ? "block" : "none";
}

function onMacroChanged(rerender = false) {
    saveMacro();
    updateMacroButton();
    if (rerender) renderMacroTasks();
    else refreshMacroFlags();
    if (lastResult) displayStats(lastResult);
}

// ── Task list operations ──

// Run a structural edit; if any "Then" row's resolved anchor changed because
// an anchor row above it was removed or moved, convert that "Then" row into
// an explicit anchor with the slot it had before the edit (so its emitted
// position never silently shifts to a different waypoint).
function editMacroTasks(fn) {
    const tasks = macroState.macro.tasks;
    const oldAnchors = macroResolveAnchors(tasks);
    const prevAnchorMap = new Map(tasks.map((t, i) => [t, oldAnchors[i]]));
    fn(tasks);
    const newAnchors = macroResolveAnchors(tasks);
    const convertedIndices = [];

    for (let i = 0; i < tasks.length; i++) {
        const t = tasks[i];
        if (t.position !== "then") continue;
        const oldA = prevAnchorMap.get(t);
        const newA = newAnchors[i];
        // If it's now row 0, or its anchor changed/became null:
        const anchorChanged =
            !newA ||
            !oldA ||
            newA.position !== oldA.position ||
            newA.index !== oldA.index;
        if (anchorChanged) {
            const fallback = oldA || { position: "after", index: 1 };
            t.position = fallback.position;
            t.index = fallback.index;
            convertedIndices.push(i);
            // Re-resolve following anchors since t is now an explicit anchor
            const refreshed = macroResolveAnchors(tasks);
            for (let k = i; k < tasks.length; k++) newAnchors[k] = refreshed[k];
        }
    }

    onMacroChanged(true);
    for (const idx of convertedIndices) {
        const row = macroTaskList.children[idx];
        row?.classList.add("flash");
        setTimeout(() => row?.classList.remove("flash"), 1200);
    }
}

function moveMacroTask(i, delta) {
    const j = i + delta;
    editMacroTasks((tasks) => {
        if (j < 0 || j >= tasks.length) return;
        [tasks[i], tasks[j]] = [tasks[j], tasks[i]];
    });
}

function deleteMacroTask(i) {
    editMacroTasks((tasks) => tasks.splice(i, 1));
}

document.getElementById("btn-macro-add").addEventListener("click", () => {
    const tasks = macroState.macro.tasks;
    tasks.push(
        tasks.length
            ? macroNewTask(null, "then", null)
            : macroNewTask(null, "before", 1),
    );
    onMacroChanged(true);
    macroTaskList.lastElementChild?.scrollIntoView({ block: "nearest" });
});

macroNameInput.addEventListener("input", () => {
    macroState.macro.name = macroNameInput.value;
    onMacroChanged();
});

macroVehicleSelect.addEventListener("change", () => {
    macroState.macro.vehicle = macroVehicleSelect.value;
    onMacroChanged(true);
});

macroEnabledInput.addEventListener("change", () => {
    macroState.enabled = macroEnabledInput.checked;
    onMacroChanged();
});

// ── File operations ──

function handleMacroFile(file) {
    const reader = new FileReader();
    reader.onload = ({ target: { result: text } }) => {
        let loaded;
        try {
            loaded = parseMacroJson(text);
        } catch (e) {
            alert(`Could not load macro: ${e.message}`);
            return;
        }
        if (
            macroState.macro.tasks.length &&
            !confirm(
                `Replace the current macro with "${loaded.name || file.name}"?`,
            )
        ) {
            return;
        }
        macroState.macro = loaded;
        renderMacroModal();
        onMacroChanged();
    };
    reader.readAsText(file);
}

const macroFileInput = Object.assign(document.createElement("input"), {
    type: "file",
    accept: ".json,application/json",
});
macroFileInput.style.display = "none";
document.body.appendChild(macroFileInput);
macroFileInput.addEventListener("change", () => {
    const file = macroFileInput.files[0];
    if (file) handleMacroFile(file);
    macroFileInput.value = "";
});

document
    .getElementById("btn-macro-load")
    .addEventListener("click", () => macroFileInput.click());

document.getElementById("btn-macro-save").addEventListener("click", () => {
    const json = JSON.stringify(macroToJson(macroState.macro), null, 4);
    const slug = macroState.macro.name
        .trim()
        .replace(/[^\w-]+/g, "_")
        .replace(/^_+|_+$/g, "");
    const blob = new Blob([json], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = Object.assign(document.createElement("a"), {
        href: url,
        download: `${slug || "yardpilot_macro"}.json`,
    });
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
});

document.getElementById("btn-macro-clear").addEventListener("click", () => {
    const { macro } = macroState;
    if (
        (macro.tasks.length || macro.name) &&
        !confirm("Clear the macro name and all tasks?")
    ) {
        return;
    }
    macroState.macro = macroEmpty(macro.vehicle);
    renderMacroModal();
    onMacroChanged();
});

// Drag-and-drop JSON onto the modal (overlay or content)
macroModal.addEventListener("dragover", (e) => {
    e.preventDefault();
    macroDropzone.classList.add("drag-over");
});
macroModal.addEventListener("dragleave", (e) => {
    if (!macroModal.contains(e.relatedTarget)) {
        macroDropzone.classList.remove("drag-over");
    }
});
macroModal.addEventListener("drop", (e) => {
    e.preventDefault();
    macroDropzone.classList.remove("drag-over");
    const file = e.dataTransfer.files[0];
    if (file) handleMacroFile(file);
});

// Prevent browser from opening dragged files dropped outside active drop zones
window.addEventListener("dragover", (e) => {
    e.preventDefault();
    if (!e.target.closest("#macro-modal") && !e.target.closest(".drop-zone")) {
        e.dataTransfer.dropEffect = "none";
    }
});
window.addEventListener("drop", (e) => {
    e.preventDefault();
});

// ── Modal open/close ──

function closeMacroModal() {
    macroModal.classList.remove("active");
}

document.getElementById("btn-macro").addEventListener("click", () => {
    renderMacroModal();
    macroModal.classList.add("active");
});
document
    .getElementById("btn-close-macro")
    .addEventListener("click", closeMacroModal);
document
    .getElementById("btn-macro-done")
    .addEventListener("click", closeMacroModal);
macroModal.addEventListener("click", (e) => {
    if (e.target === macroModal) closeMacroModal();
});
window.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && macroModal.classList.contains("active")) {
        closeMacroModal();
    }
});

loadMacro();
updateMacroButton();

// Load persisted settings and zones on start
loadSettings();
loadZones();

document
    .getElementById("input-target-speed")
    .addEventListener("change", saveSettings);
document
    .getElementById("input-altitude")
    .addEventListener("change", saveSettings);

// ── Help Modal Handlers ───────────────────────────────────

const helpModal = document.getElementById("help-modal");
const btnHelp = document.getElementById("btn-help");
const btnClose = document.getElementById("btn-close-help");

btnHelp.addEventListener("click", () => {
    helpModal.classList.add("active");
});

btnClose.addEventListener("click", () => {
    helpModal.classList.remove("active");
});

helpModal.addEventListener("click", (e) => {
    if (e.target === helpModal) {
        helpModal.classList.remove("active");
    }
});

window.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && helpModal.classList.contains("active")) {
        helpModal.classList.remove("active");
    }
});
