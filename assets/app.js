const map = L.map('map', {
    center: [39.8283, -98.5795],
    zoom: 5,
    zoomControl: true,
});

const satellite = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    {
        attribution: 'Imagery &copy; Esri &mdash; Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP',
        maxZoom: 19,
    }
).addTo(map);

const labels = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 19, pane: 'overlayPane' }
);

const [satelliteToggle, labelsToggle] = document.querySelectorAll('.toggle-row input');

satelliteToggle.addEventListener('change', () => {
    satelliteToggle.checked ? map.addLayer(satellite) : map.removeLayer(satellite);
});

labelsToggle.addEventListener('change', () => {
    labelsToggle.checked ? map.addLayer(labels) : map.removeLayer(labels);
});

// ── Zone state ────────────────────────────────────────────

const PERIMETER_STYLE = {
    color: '#3fb950', fillColor: '#3fb950', fillOpacity: 0.08, weight: 2,
};
const EXCLUSION_STYLE = {
    color: '#f85149', fillColor: '#f85149', fillOpacity: 0.2, weight: 2,
};

const zones = {
    perimeter: null,   // { name, layers }
    exclusions: [],    // [{ name, layers }]
};

function removeZoneLayers(zone) {
    zone.layers.forEach(l => map.removeLayer(l));
}

function clearPerimeter() {
    if (zones.perimeter) {
        removeZoneLayers(zones.perimeter);
        zones.perimeter = null;
        document.getElementById('drop-perimeter').classList.remove('has-file');
    }
}

function clearExclusions() {
    zones.exclusions.forEach(removeZoneLayers);
    zones.exclusions = [];
    document.getElementById('drop-exclusion').classList.remove('has-file');
}

function renderPerimeter(coords, name) {
    clearPerimeter();
    const poly = L.polygon(coords, PERIMETER_STYLE).addTo(map);
    zones.perimeter = { name, layers: [poly] };
    map.fitBounds(poly.getBounds(), { padding: [30, 30] });
    document.getElementById('drop-perimeter').classList.add('has-file');
    syncUI();
}

function renderExclusion(shapes, name) {
    clearExclusions();
    const layers = shapes.map(shape => {
        if (shape.type === 'circle') {
            return L.circle([shape.lat, shape.lon], { radius: shape.radius, ...EXCLUSION_STYLE }).addTo(map);
        }
        return L.polygon(shape.vertices, EXCLUSION_STYLE).addTo(map);
    });
    zones.exclusions.push({ name, layers });
    document.getElementById('drop-exclusion').classList.add('has-file');
    syncUI();
}

function syncUI() {
    const list = document.getElementById('zone-list');
    list.innerHTML = '';

    const entries = [];
    if (zones.perimeter) {
        entries.push({ name: zones.perimeter.name, color: '#3fb950', remove: () => { clearPerimeter(); syncUI(); } });
    }
    zones.exclusions.forEach((z, i) => {
        entries.push({
            name: z.name, color: '#f85149',
            remove: () => { removeZoneLayers(z); zones.exclusions.splice(i, 1); syncUI(); },
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
        return {
            seq:    parseInt(p[0]),
            cmd:    parseInt(p[3]),
            param1: parseFloat(p[4]),
            lat:    parseFloat(p[8]),
            lon:    parseFloat(p[9]),
        };
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
            // Circle inclusion/exclusion — param1 is radius in meters
            shapes.push({ type: 'circle', lat, lon, radius: param1 });
            i++;
        } else if (cmd === 5002 || cmd === 5003) {
            // Polygon inclusion/exclusion — param1 is vertex count for this polygon
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
                : (() => { const rows = parseWaypointRows(text); return rows ? waypointsToPerimeterCoords(rows) : null; })();
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
        type: 'file', accept, style: 'display:none',
    });
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
setupDropZone(document.getElementById('drop-exclusion'), 'exclusion', '.waypoints');
