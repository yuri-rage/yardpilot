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
);

const satelliteToggle = document.getElementById('layer-satellite');
const labelsToggle = document.getElementById('layer-labels');
const brightnessInput = document.getElementById('map-brightness');
const brightnessVal = document.getElementById('brightness-val');

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

// ── LocalStorage persistence ─────────────────────────────

function saveSettings() {
    const settings = {
        laneWidth: document.getElementById('input-lane-width').value,
        exclusionBuffer: document.getElementById('input-exclusion-buffer').value,
        transitionTolerance: document.getElementById('input-transition-tolerance').value,
        skipLanes: document.getElementById('input-skip-lanes').value,
        sweepAuto: document.getElementById('checkbox-sweep-auto').checked,
        sweepAngle: document.getElementById('input-custom-sweep-angle').value,
        perimeterPasses: document.getElementById('input-perimeter-passes').value,
        perimeterDirection: document.getElementById('select-perimeter-direction').value,
        layerSatellite: satelliteToggle.checked,
        layerLabels: labelsToggle.checked,
        mapBrightness: brightnessInput.value
    };
    localStorage.setItem('yardpilot_settings', JSON.stringify(settings));
}

function loadSettings() {
    const data = localStorage.getItem('yardpilot_settings');
    if (!data) return;
    try {
        const settings = JSON.parse(data);
        if (settings.laneWidth !== undefined) document.getElementById('input-lane-width').value = settings.laneWidth;
        if (settings.exclusionBuffer !== undefined) document.getElementById('input-exclusion-buffer').value = settings.exclusionBuffer;
        if (settings.transitionTolerance !== undefined) document.getElementById('input-transition-tolerance').value = settings.transitionTolerance;
        if (settings.skipLanes !== undefined) document.getElementById('input-skip-lanes').value = settings.skipLanes;
        if (settings.sweepAuto !== undefined) {
            const autoCheckbox = document.getElementById('checkbox-sweep-auto');
            const angleSlider = document.getElementById('input-custom-sweep-angle');
            const angleRow = document.getElementById('row-custom-sweep-angle');
            autoCheckbox.checked = settings.sweepAuto;
            angleSlider.disabled = settings.sweepAuto;
            angleRow.style.opacity = settings.sweepAuto ? '0.35' : '1.0';
        }
        if (settings.sweepAngle !== undefined) {
            document.getElementById('input-custom-sweep-angle').value = settings.sweepAngle;
            document.getElementById('sweep-angle-val').textContent = `${settings.sweepAngle}°`;
        }
        if (settings.perimeterPasses !== undefined) document.getElementById('input-perimeter-passes').value = settings.perimeterPasses;
        if (settings.perimeterDirection !== undefined) document.getElementById('select-perimeter-direction').value = settings.perimeterDirection;
        
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
    } catch (e) {
        console.error('Failed to load settings', e);
    }
}

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
    'select-perimeter-direction'
];
settingInputs.forEach(id => {
    document.getElementById(id).addEventListener('change', saveSettings);
});

// Update slider state dynamically when checkbox is toggled
const sweepAutoCheckbox = document.getElementById('checkbox-sweep-auto');
const sweepAngleInput = document.getElementById('input-custom-sweep-angle');
const sweepAngleVal = document.getElementById('sweep-angle-val');
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
    if (pathLayer) { map.removeLayer(pathLayer); pathLayer = null; }
    pathWaypoints = null;
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

// ── Path generation ───────────────────────────────────────

document.getElementById('btn-generate').addEventListener('click', () => {
    if (!zones.perimeter) return;
    const laneWidth = parseFloat(document.getElementById('input-lane-width').value)       || 2;
    const buffer    = parseFloat(document.getElementById('input-exclusion-buffer').value) || 1;
    const tolerance = parseFloat(document.getElementById('input-transition-tolerance').value) || 0;
    const skipLanes = parseInt(document.getElementById('input-skip-lanes').value)         || 0;
    const sweepAuto = document.getElementById('checkbox-sweep-auto').checked;
    const sweepMode = sweepAuto ? 'auto' : 'custom';
    const sweepAngle = parseFloat(document.getElementById('input-custom-sweep-angle').value) || 0;
    const nPasses   = parseInt(document.getElementById('input-perimeter-passes').value)   || 0;
    const direction = document.getElementById('select-perimeter-direction').value         || 'CW';
    const allShapes = zones.exclusions.flatMap(z => z.shapes);

    const result = generateCoveragePath(zones.perimeter.coords, allShapes, laneWidth, buffer, nPasses, direction, tolerance, skipLanes, sweepMode, sweepAngle);
    if (!result) return;

    clearPath();
    pathWaypoints = result.path;
    pathLayer = L.polyline(result.path, PATH_STYLE).addTo(map);
    syncUI();
});

// ── Waypoint export ───────────────────────────────────────

document.getElementById('btn-export').addEventListener('click', () => {
    if (!pathWaypoints) return;
    const [hLat, hLon] = pathWaypoints[0];
    const lines = [
        'QGC WPL 110',
        `0\t1\t0\t16\t0\t0\t0\t0\t${hLat.toFixed(7)}\t${hLon.toFixed(7)}\t0.000000\t1`,
        ...pathWaypoints.map(([lat, lon], i) =>
            `${i + 1}\t0\t3\t16\t0\t0\t0\t0\t${lat.toFixed(7)}\t${lon.toFixed(7)}\t0.000000\t1`
        ),
    ];
    const a = Object.assign(document.createElement('a'), {
        href: URL.createObjectURL(new Blob([lines.join('\n')], { type: 'text/plain' })),
        download: 'mow_path.waypoints',
    });
    a.click();
    URL.revokeObjectURL(a.href);
});

// Load persisted settings and zones on start
loadSettings();
loadZones();
