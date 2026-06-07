const map = L.map('map', {
    center: [39.8283, -98.5795],
    zoom: 5,
    zoomControl: true,
    maxZoom: 23,
});

const satellite = L.tileLayer(
    'https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
    {
        attribution: 'Map data &copy; Google',
        maxNativeZoom: 21,
        maxZoom: 23,
        className: 'map-tile-satellite',
    }
).addTo(map);

const labels = L.tileLayer(
    'https://mt1.google.com/vt/lyrs=h&x={x}&y={y}&z={z}',
    {
        maxNativeZoom: 21,
        maxZoom: 23,
        pane: 'overlayPane',
    }
).addTo(map);

const satelliteToggle = document.getElementById('layer-satellite');
const labelsToggle = document.getElementById('layer-labels');
const brightnessInput = document.getElementById('map-brightness');
const brightnessVal = document.getElementById('brightness-val');
document.getElementById('map').style.setProperty('--map-brightness', '75%');

satelliteToggle.addEventListener('change', () => {
    satelliteToggle.checked ? map.addLayer(satellite) : map.removeLayer(satellite);
    saveSettings();
});
labelsToggle.addEventListener('change', () => {
    labelsToggle.checked ? map.addLayer(labels) : map.removeLayer(labels);
    saveSettings();
});
brightnessInput.addEventListener('input', (e) => {
    const val = e.target.value;
    brightnessVal.textContent = `${val}%`;
    document.getElementById('map').style.setProperty('--map-brightness', `${val}%`);
    saveSettings();
});

// ── Unit conversion constants ─────────────────────────────

const FT_PER_M   = 3.28084;
const MPH_PER_MS = 2.23694;
const M_PER_FT   = 1 / FT_PER_M;
const MS_PER_MPH = 1 / MPH_PER_MS;

// Current unit mode — 'metric' | 'imperial'
let currentUnits = 'metric';

function isImperial() { return currentUnits === 'imperial'; }

// Convert a stored metric value → display value
function toDisplay(metricVal, type) {
    if (!isImperial()) return metricVal;
    if (type === 'length') return Math.round(metricVal * FT_PER_M * 100) / 100;   // 2 dp in ft
    if (type === 'speed')  return Math.round(metricVal * MPH_PER_MS * 100) / 100; // 2 dp in mph
    return metricVal;
}

// Convert a display value → metric storage value
function toMetric(displayVal, type) {
    if (!isImperial()) return displayVal;
    if (type === 'length') return displayVal * M_PER_FT;
    if (type === 'speed')  return displayVal * MS_PER_MPH;
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
    'input-lane-width':           { type:'length', step_m:0.01, step_imp:0.05, dec_m:2, dec_imp:2, labelEl:null,                metricLabel:'Lane width (m)',          imperialLabel:'Lane width (ft)'          },
    'input-exclusion-buffer':     { type:'length', step_m:0.1,  step_imp:0.5,  dec_m:1, dec_imp:1, labelEl:null,                metricLabel:'Exclusion buffer (m)',     imperialLabel:'Exclusion buffer (ft)'     },
    'input-transition-tolerance': { type:'length', step_m:0.05, step_imp:0.1,  dec_m:2, dec_imp:2, labelEl:null,                metricLabel:'Transition tolerance (m)', imperialLabel:'Transition tolerance (ft)' },
    'input-target-speed':         { type:'speed',  step_m:0.1,  step_imp:0.1,  dec_m:1, dec_imp:1, labelEl:'label-target-speed', metricLabel:'Target speed (m/s)',       imperialLabel:'Target speed (mph)'       },
    'input-altitude':             { type:'length', step_m:0.5,  step_imp:5,    dec_m:1, dec_imp:0, labelEl:'label-altitude',    metricLabel:'Altitude (m)',             imperialLabel:'Altitude (ft)'            },
};

// Read the raw metric value from a unit-sensitive input (using current unit mode)
function getMetricValue(id) {
    const cfg = UNIT_INPUTS[id];
    const raw = parseFloat(document.getElementById(id).value) || 0;
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
    document.getElementById(id).value = parseFloat(displayVal.toFixed(decimals));
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
        const decimals   = isImperial() ? cfg.dec_imp : cfg.dec_m;
        el.value = parseFloat(displayVal.toFixed(decimals));
        el.step  = isImperial() ? cfg.step_imp : cfg.step_m;

        // Update label
        const labelText = isImperial() ? cfg.imperialLabel : cfg.metricLabel;
        if (cfg.labelEl) {
            document.getElementById(cfg.labelEl).textContent = labelText;
        } else {
            const parent = el.closest('label') || el.parentElement;
            const span = parent ? parent.querySelector('span') : null;
            if (span) span.textContent = labelText;
        }
    }
}

// ── LocalStorage persistence ─────────────────────────────

// Settings are always stored in METRIC regardless of current unit mode.
function saveSettings() {
    const settings = {
        laneWidth:           getMetricValue('input-lane-width'),
        exclusionBuffer:     getMetricValue('input-exclusion-buffer'),
        transitionTolerance: getMetricValue('input-transition-tolerance'),
        skipLanes:           parseInt(document.getElementById('input-skip-lanes').value) || 0,
        targetSpeed:         getMetricValue('input-target-speed'),
        altitude:            getMetricValue('input-altitude'),
        sweepAuto:           document.getElementById('checkbox-sweep-auto').checked,
        sweepAngle:          document.getElementById('input-custom-sweep-angle').value,
        perimeterPasses:     document.getElementById('input-perimeter-passes').value,
        perimeterDirection:  document.getElementById('select-perimeter-direction').value,
        circleSegments:      parseInt(document.getElementById('input-circle-segments').value) || 64,
        layerSatellite:      satelliteToggle.checked,
        layerLabels:         labelsToggle.checked,
        mapBrightness:       brightnessInput.value,
        units:               currentUnits,
    };
    localStorage.setItem('yardpilot_settings', JSON.stringify(settings));
}

// Return the stored METRIC value for a unit-sensitive input (from localStorage)
function getStoredMetricValue(id) {
    const data = localStorage.getItem('yardpilot_settings');
    if (!data) {
        // Fall back to converting the current display value back to metric
        const cfg = UNIT_INPUTS[id];
        const raw = parseFloat(document.getElementById(id).value) || 0;
        return toMetric(raw, cfg.type);
    }
    try {
        const settings = JSON.parse(data);
        const keyMap = {
            'input-lane-width':           'laneWidth',
            'input-exclusion-buffer':     'exclusionBuffer',
            'input-transition-tolerance': 'transitionTolerance',
            'input-target-speed':         'targetSpeed',
            'input-altitude':             'altitude',
        };
        const key = keyMap[id];
        if (key !== undefined && settings[key] !== undefined) return parseFloat(settings[key]);
    } catch(e) {}
    const cfg = UNIT_INPUTS[id];
    const raw = parseFloat(document.getElementById(id).value) || 0;
    return toMetric(raw, cfg.type);
}

function loadSettings() {
    const data = localStorage.getItem('yardpilot_settings');
    if (!data) return;
    try {
        const settings = JSON.parse(data);

        // Restore unit mode first so conversions are correct
        if (settings.units) {
            currentUnits = settings.units;
            document.getElementById('units-metric').classList.toggle('active', currentUnits === 'metric');
            document.getElementById('units-imperial').classList.toggle('active', currentUnits === 'imperial');
        }

        // Restore metric values, display in current unit
        if (settings.laneWidth           !== undefined) setDisplayValue('input-lane-width',           settings.laneWidth);
        if (settings.exclusionBuffer     !== undefined) setDisplayValue('input-exclusion-buffer',     settings.exclusionBuffer);
        if (settings.transitionTolerance !== undefined) setDisplayValue('input-transition-tolerance', settings.transitionTolerance);
        if (settings.targetSpeed         !== undefined) setDisplayValue('input-target-speed',         settings.targetSpeed);
        if (settings.altitude            !== undefined) setDisplayValue('input-altitude',             settings.altitude);

        if (settings.skipLanes !== undefined) document.getElementById('input-skip-lanes').value = settings.skipLanes;
        if (settings.sweepAuto !== undefined) {
            const autoCheckbox = document.getElementById('checkbox-sweep-auto');
            const angleSlider  = document.getElementById('input-custom-sweep-angle');
            const angleRow     = document.getElementById('row-custom-sweep-angle');
            autoCheckbox.checked      = settings.sweepAuto;
            angleSlider.disabled      = settings.sweepAuto;
            angleRow.style.opacity    = settings.sweepAuto ? '0.35' : '1.0';
        }
        if (settings.sweepAngle !== undefined) {
            document.getElementById('input-custom-sweep-angle').value = settings.sweepAngle;
            document.getElementById('sweep-angle-val').textContent = `${settings.sweepAngle}°`;
        }
        if (settings.perimeterPasses    !== undefined) document.getElementById('input-perimeter-passes').value   = settings.perimeterPasses;
        if (settings.perimeterDirection !== undefined) document.getElementById('select-perimeter-direction').value = settings.perimeterDirection;
        if (settings.circleSegments     !== undefined) document.getElementById('input-circle-segments').value     = settings.circleSegments;

        if (settings.layerSatellite !== undefined) {
            satelliteToggle.checked = settings.layerSatellite;
            settings.layerSatellite ? map.addLayer(satellite) : map.removeLayer(satellite);
        }
        if (settings.layerLabels !== undefined) {
            labelsToggle.checked = settings.layerLabels;
            settings.layerLabels ? map.addLayer(labels) : map.removeLayer(labels);
        }
        if (settings.mapBrightness !== undefined) {
            brightnessInput.value = settings.mapBrightness;
            brightnessVal.textContent = `${settings.mapBrightness}%`;
            document.getElementById('map').style.setProperty('--map-brightness', `${settings.mapBrightness}%`);
        }

        // Apply labels and steps to all unit inputs
        applyUnitsToInputs();

    } catch (e) {
        console.error('Failed to load settings', e);
    }
}

// ── Units toggle wiring ───────────────────────────────────

document.getElementById('units-metric').addEventListener('click', () => {
    if (currentUnits === 'metric') return;
    // Snapshot metric values from DOM before changing mode
    const snap = snapshotMetricValues();
    currentUnits = 'metric';
    document.getElementById('units-metric').classList.add('active');
    document.getElementById('units-imperial').classList.remove('active');
    applyUnitsToInputs(snap);
    saveSettings();
    if (lastResult) displayStats(lastResult);
});

document.getElementById('units-imperial').addEventListener('click', () => {
    if (currentUnits === 'imperial') return;
    // Snapshot metric values from DOM before changing mode
    const snap = snapshotMetricValues();
    currentUnits = 'imperial';
    document.getElementById('units-metric').classList.remove('active');
    document.getElementById('units-imperial').classList.add('active');
    applyUnitsToInputs(snap);
    saveSettings();
    if (lastResult) displayStats(lastResult);
});

// ── Zone state ────────────────────────────────────────────

function saveZones() {
    const data = {
        perimeter: zones.perimeter ? { name: zones.perimeter.name, coords: zones.perimeter.coords } : null,
        exclusions: zones.exclusions.map(z => ({ name: z.name, shapes: z.shapes }))
    };
    localStorage.setItem('yardpilot_zones', JSON.stringify(data));
}

function loadZones() {
    const data = localStorage.getItem('yardpilot_zones');
    if (!data) return;
    try {
        const parsed = JSON.parse(data);
        if (parsed.perimeter) {
            renderPerimeter(parsed.perimeter.coords, parsed.perimeter.name);
        }
        if (parsed.exclusions && parsed.exclusions.length) {
            zones.exclusions.forEach(z => removeLayerGroup(z.layers));
            zones.exclusions = [];
            
            parsed.exclusions.forEach(z => {
                const layers = z.shapes.map(s => {
                    if (s.type === 'circle') {
                        return L.circle([s.lat, s.lon], { radius: s.radius, ...EXCLUSION_STYLE }).addTo(map);
                    }
                    return L.polygon(s.vertices, EXCLUSION_STYLE).addTo(map);
                });
                zones.exclusions.push({ name: z.name, shapes: z.shapes, layers });
            });
            document.getElementById('drop-exclusion').classList.add('has-file');
            syncUI();
        }
    } catch (e) {
        console.error('Failed to load zones', e);
    }
}

// Attach change listeners to settings inputs
const settingInputs = [
    'input-lane-width',
    'input-exclusion-buffer',
    'input-transition-tolerance',
    'input-skip-lanes',
    'checkbox-sweep-auto',
    'input-custom-sweep-angle',
    'input-perimeter-passes',
    'select-perimeter-direction',
    'input-circle-segments'
];
settingInputs.forEach(id => {
    document.getElementById(id).addEventListener('change', saveSettings);
});

// Update slider state dynamically when checkbox is toggled
const sweepAutoCheckbox  = document.getElementById('checkbox-sweep-auto');
const sweepAngleInput    = document.getElementById('input-custom-sweep-angle');
const sweepAngleVal      = document.getElementById('sweep-angle-val');
const customSweepAngleRow = document.getElementById('row-custom-sweep-angle');

sweepAutoCheckbox.addEventListener('change', (e) => {
    const isAuto = e.target.checked;
    sweepAngleInput.disabled = isAuto;
    customSweepAngleRow.style.opacity = isAuto ? '0.35' : '1.0';
    saveSettings();
});

sweepAngleInput.addEventListener('input', (e) => {
    sweepAngleVal.textContent = `${e.target.value}°`;
    saveSettings();
});

// ── Zone state ────────────────────────────────────────────

const PERIMETER_STYLE = { color: '#3fb950', fillColor: '#3fb950', fillOpacity: 0.08, weight: 2 };
const EXCLUSION_STYLE = { color: '#f85149', fillColor: '#f85149', fillOpacity: 0.2,  weight: 2 };
const PATH_STYLE      = { color: '#58a6ff', weight: 1.5, opacity: 0.9 };

const zones = {
    perimeter:  null,  // { name, coords, layers }
    exclusions: [],    // [{ name, shapes, layers }]
};
let pathLayer     = null;
let pathWaypoints = null;
let startMarker   = null;
let endMarker     = null;
// Store last result so stats can be refreshed when units change
let lastResult    = null;

function removeLayerGroup(layers) { layers.forEach(l => map.removeLayer(l)); }

function clearPerimeter() {
    if (!zones.perimeter) return;
    removeLayerGroup(zones.perimeter.layers);
    zones.perimeter = null;
    document.getElementById('drop-perimeter').classList.remove('has-file');
    saveZones();
}

function clearExclusions() {
    zones.exclusions.forEach(z => removeLayerGroup(z.layers));
    zones.exclusions = [];
    document.getElementById('drop-exclusion').classList.remove('has-file');
    saveZones();
}

function clearPath() {
    if (pathLayer)   { map.removeLayer(pathLayer);   pathLayer   = null; }
    if (startMarker) { map.removeLayer(startMarker); startMarker = null; }
    if (endMarker)   { map.removeLayer(endMarker);   endMarker   = null; }
    pathWaypoints = null;
    lastResult    = null;
    const statsEl = document.getElementById('stats-panel');
    if (statsEl) statsEl.style.display = 'none';
}

function renderPerimeter(coords, name) {
    clearPerimeter();
    clearPath();
    const poly = L.polygon(coords, PERIMETER_STYLE).addTo(map);
    zones.perimeter = { name, coords, layers: [poly] };
    map.fitBounds(poly.getBounds(), { padding: [30, 30] });
    document.getElementById('drop-perimeter').classList.add('has-file');
    syncUI();
    saveZones();
}

function renderExclusion(shapes, name) {
    clearExclusions();
    clearPath();
    const layers = shapes.map(s => {
        if (s.type === 'circle') {
            return L.circle([s.lat, s.lon], { radius: s.radius, ...EXCLUSION_STYLE }).addTo(map);
        }
        return L.polygon(s.vertices, EXCLUSION_STYLE).addTo(map);
    });
    zones.exclusions.push({ name, shapes, layers });
    document.getElementById('drop-exclusion').classList.add('has-file');
    syncUI();
    saveZones();
}

function syncUI() {
    const list = document.getElementById('zone-list');
    list.innerHTML = '';

    const entries = [];
    if (zones.perimeter) {
        entries.push({
            name: zones.perimeter.name, color: '#3fb950',
            remove: () => { clearPerimeter(); clearPath(); syncUI(); },
        });
    }
    zones.exclusions.forEach((z, i) => {
        entries.push({
            name: z.name, color: '#f85149',
            remove: () => {
                removeLayerGroup(z.layers);
                zones.exclusions.splice(i, 1);
                clearPath();
                syncUI();
                saveZones();
            },
        });
    });

    entries.forEach(({ name, color, remove }) => {
        const item = document.createElement('div');
        item.className = 'zone-item';
        item.innerHTML =
            `<span class="zone-dot" style="background:${color}"></span>` +
            `<span class="zone-name" title="${name}">${name}</span>` +
            `<button class="zone-remove" title="Remove">×</button>`;
        item.querySelector('.zone-remove').addEventListener('click', remove);
        list.appendChild(item);
    });

    document.getElementById('btn-generate').disabled = !zones.perimeter;
    document.getElementById('btn-export').disabled   = !pathWaypoints;
}

// ── File parsing ──────────────────────────────────────────

function parsePolyFile(text) {
    return text.split('\n')
        .map(l => l.trim())
        .filter(l => l && !l.startsWith('#'))
        .map(l => l.split(/\s+/).map(Number))
        .filter(([lat, lon]) => !isNaN(lat) && !isNaN(lon));
}

function parseWaypointRows(text) {
    const lines = text.split('\n').map(l => l.trim()).filter(Boolean);
    if (!lines[0].startsWith('QGC WPL')) return null;
    return lines.slice(1).map(line => {
        const p = line.split('\t');
        return { seq: +p[0], cmd: +p[3], param1: +p[4], lat: +p[8], lon: +p[9] };
    });
}

function waypointsToPerimeterCoords(rows) {
    return rows.filter(r => r.seq > 0 && r.cmd === 16).map(r => [r.lat, r.lon]);
}

function waypointsToExclusionShapes(rows) {
    const fence = rows.filter(r => r.seq > 0);
    const shapes = [];
    let i = 0;
    while (i < fence.length) {
        const { cmd, param1, lat, lon } = fence[i];
        if (cmd === 5004 || cmd === 5005) {
            shapes.push({ type: 'circle', lat, lon, radius: param1 });
            i++;
        } else if (cmd === 5002 || cmd === 5003) {
            const count = Math.round(param1);
            const vertices = fence.slice(i, i + count).map(r => [r.lat, r.lon]);
            shapes.push({ type: 'polygon', vertices });
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
        if (type === 'perimeter') {
            const coords = name.endsWith('.poly')
                ? parsePolyFile(text)
                : (() => { const r = parseWaypointRows(text); return r ? waypointsToPerimeterCoords(r) : null; })();
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
    const input = Object.assign(document.createElement('input'), {
        type: 'file', accept,
    });
    input.style.display = 'none';
    document.body.appendChild(input);

    el.style.cursor = 'pointer';
    el.addEventListener('click', () => input.click());
    input.addEventListener('change', () => {
        const file = input.files[0];
        if (file) handleFile(file, type);
        input.value = '';
    });
    el.addEventListener('dragover', e => { e.preventDefault(); el.classList.add('drag-over'); });
    el.addEventListener('dragleave', () => el.classList.remove('drag-over'));
    el.addEventListener('drop', e => {
        e.preventDefault();
        el.classList.remove('drag-over');
        const file = e.dataTransfer.files[0];
        if (file) handleFile(file, type);
    });
}

setupDropZone(document.getElementById('drop-perimeter'), 'perimeter', '.poly,.waypoints');
setupDropZone(document.getElementById('drop-exclusion'), 'exclusion',  '.waypoints');

// ── Stats display (unit-aware) ────────────────────────────

function displayStats(result) {
    if (!result) return;
    lastResult = result;

    const areaSqM    = result.coveredAreaSqM;
    const totalDistM = result.totalDistM;
    // Speed is always read back as metric from storage
    const speedMS = getStoredMetricValue('input-target-speed') || 1.5;

    if (isImperial()) {
        // Area: sq ft, and sq miles as sub
        const areaSqFt    = areaSqM * 10.7639;
        const areaSqMiles = areaSqM * 3.861e-7;
        const areaVal = areaSqFt > 1e6
            ? (areaSqMiles).toFixed(4) + ' mi²'
            : areaSqFt.toLocaleString(undefined, { maximumFractionDigits: 0 }) + ' ft²';
        const areaSub = (areaSqM * 0.000247105).toFixed(3) + ' acres';
        document.getElementById('stat-area').textContent    = areaVal;
        document.getElementById('stat-area-sub').textContent = areaSub;

        // Distance: miles
        const distMiles = totalDistM * 0.000621371;
        const distVal = distMiles >= 0.1
            ? distMiles.toFixed(3) + ' mi'
            : (totalDistM * FT_PER_M).toFixed(0) + ' ft';
        document.getElementById('stat-distance').textContent    = distVal;
        document.getElementById('stat-distance-sub').textContent = `${result.count} waypoints`;

        // Duration: uses m/s internally
        const durationSec = totalDistM / speedMS;
        const durationMin = Math.floor(durationSec / 60);
        const durationSecRem = Math.round(durationSec % 60);
        const durVal = durationMin > 0 ? `${durationMin}m ${durationSecRem}s` : `${durationSecRem}s`;
        const speedMph = speedMS * MPH_PER_MS;
        document.getElementById('stat-duration').textContent    = durVal;
        document.getElementById('stat-duration-sub').textContent = `at ${speedMph.toFixed(1)} mph`;
    } else {
        // Metric display
        const areaVal = areaSqM > 10000
            ? (areaSqM / 10000).toFixed(2) + ' ha'
            : areaSqM.toLocaleString(undefined, { maximumFractionDigits: 0 }) + ' m²';
        const areaSub = (areaSqM * 0.000247105).toFixed(2) + ' acres';
        document.getElementById('stat-area').textContent    = areaVal;
        document.getElementById('stat-area-sub').textContent = areaSub;

        const distVal = totalDistM > 1000
            ? (totalDistM / 1000).toFixed(2) + ' km'
            : totalDistM.toFixed(0) + ' m';
        document.getElementById('stat-distance').textContent    = distVal;
        document.getElementById('stat-distance-sub').textContent = `${result.count} waypoints`;

        const durationSec    = totalDistM / speedMS;
        const durationMin    = Math.floor(durationSec / 60);
        const durationSecRem = Math.round(durationSec % 60);
        const durVal = durationMin > 0 ? `${durationMin}m ${durationSecRem}s` : `${durationSecRem}s`;
        document.getElementById('stat-duration').textContent    = durVal;
        document.getElementById('stat-duration-sub').textContent = `at ${speedMS.toFixed(1)} m/s`;
    }

    const efficiency = totalDistM > 0
        ? ((result.sweepDistM / totalDistM) * 100).toFixed(0) + '%'
        : '0%';
    document.getElementById('stat-efficiency').textContent = efficiency;
    document.getElementById('stats-panel').style.display = 'block';
}

// ── Path generation ───────────────────────────────────────

document.getElementById('btn-generate').addEventListener('click', () => {
    if (!zones.perimeter) return;

    // All values retrieved as metric regardless of display unit
    const laneWidth = getMetricValue('input-lane-width')           || 1.2;
    const buffer    = getMetricValue('input-exclusion-buffer')     || 1.0;
    const tolerance = getMetricValue('input-transition-tolerance') || 0;
    const skipLanes = parseInt(document.getElementById('input-skip-lanes').value) || 0;
    const sweepAuto  = document.getElementById('checkbox-sweep-auto').checked;
    const sweepMode  = sweepAuto ? 'auto' : 'custom';
    const sweepAngle = parseFloat(document.getElementById('input-custom-sweep-angle').value) || 0;
    const nPasses    = parseInt(document.getElementById('input-perimeter-passes').value) || 0;
    const direction  = document.getElementById('select-perimeter-direction').value || 'CW';
    const allShapes  = zones.exclusions.flatMap(z => z.shapes);

    const circleSegments = parseInt(document.getElementById('input-circle-segments').value) || 64;

    const result = generateCoveragePath(
        zones.perimeter.coords, allShapes,
        laneWidth, buffer, nPasses, direction, tolerance, skipLanes, sweepMode, sweepAngle, circleSegments
    );
    if (!result) return;

    clearPath();
    pathWaypoints = result.path;
    pathLayer = L.polyline(result.path, PATH_STYLE).addTo(map);

    if (result.path.length > 0) {
        const startPt = result.path[0];
        const endPt   = result.path[result.path.length - 1];

        startMarker = L.circleMarker(startPt, {
            radius: 6, fillColor: '#10b981', color: '#ffffff', weight: 1.5, fillOpacity: 1
        }).bindPopup('<b>Start Coverage</b>').addTo(map);

        endMarker = L.circleMarker(endPt, {
            radius: 6, fillColor: '#ef4444', color: '#ffffff', weight: 1.5, fillOpacity: 1
        }).bindPopup('<b>End Coverage</b>').addTo(map);
    }

    displayStats(result);
    syncUI();
});

// ── Waypoint export ───────────────────────────────────────

document.getElementById('btn-export').addEventListener('click', () => {
    if (!pathWaypoints) return;
    // Altitude is always exported in meters — convert from display unit to metric
    const altitudeM = getMetricValue('input-altitude') || 0.0;
    const [hLat, hLon] = pathWaypoints[0];
    const lines = [
        'QGC WPL 110',
        `0\t1\t0\t16\t0\t0\t0\t0\t${hLat.toFixed(7)}\t${hLon.toFixed(7)}\t${altitudeM.toFixed(6)}\t1`,
        ...pathWaypoints.map(([lat, lon], i) =>
            `${i + 1}\t0\t3\t16\t0\t0\t0\t0\t${lat.toFixed(7)}\t${lon.toFixed(7)}\t${altitudeM.toFixed(6)}\t1`
        ),
    ];
    const a = Object.assign(document.createElement('a'), {
        href: URL.createObjectURL(new Blob([lines.join('\n')], { type: 'text/plain' })),
        download: 'yardpilot.waypoints',
    });
    a.click();
    URL.revokeObjectURL(a.href);
});

// Load persisted settings and zones on start
loadSettings();
loadZones();

document.getElementById('input-target-speed').addEventListener('change', saveSettings);
document.getElementById('input-altitude').addEventListener('change', saveSettings);

// ── Help Modal Handlers ───────────────────────────────────

const helpModal = document.getElementById('help-modal');
const btnHelp   = document.getElementById('btn-help');
const btnClose  = document.getElementById('btn-close-help');

btnHelp.addEventListener('click', () => {
    helpModal.classList.add('active');
});

btnClose.addEventListener('click', () => {
    helpModal.classList.remove('active');
});

helpModal.addEventListener('click', (e) => {
    if (e.target === helpModal) {
        helpModal.classList.remove('active');
    }
});

window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && helpModal.classList.contains('active')) {
        helpModal.classList.remove('active');
    }
});

