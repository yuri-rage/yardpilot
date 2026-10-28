# Release Notes

## v0.3.0-beta

- planner: route around merged obstacle outlines to prevent boundary and buffer intrusions
- planner: plan disconnected yard pieces independently without crossing cut obstacles
- planner: route headlands around interior obstacles inside sub-pieces
- planner: implement offline polygon subtraction using Weiler-Atherton clipping fallback
- planner: select perimeter pass start vertex away from obstacles with entry ramp clearance
- planner: add visibility graph routing for direct collision-free transit between passes
- app: render separate polylines and distinct start/end markers for multi-piece yards
- app: export individual waypoint mission files for each disconnected yard piece
- app: stagger multi-piece mission downloads with delayed url revoke
- app: display disconnected yard summary and per-piece coverage statistics
- tests: add review harness with fixed scenarios and 150-case fuzz suites
- tests: evaluate combined piece coverage and remove split exemption
- ci: run fixed scenario tests and full fuzz suites in both Clipper and offline modes

## v0.2.1-beta

- planner: fix clustered waypoint generation and near-duplicate revisits in corner turns
- planner: refine collinear waypoint reduction and tolerance thresholds
- styles: refine dark mode UI theme and layout styling

## v0.2.0-beta

- planner: add combine waypoints simplification to reduce total path point count
- planner: fix spiral pass closing and winding transitions

## v0.1.0-beta

- planner: implement boustrophedon sweep planner with MBB optimal angle calculation
- planner: support multi-pass outer perimeter laps with spiral transition ramps
- planner: support reversible spiral coverage mode with inward and outward passes
- planner: implement configurable headland margin to inset sweep strips from boundary
- planner: integrate Voronoi roadmap collision-free transit routing
- app: interactive Leaflet map interface with drag-and-drop boundary and exclusion loading
- app: format mission export filenames with yard name and planning parameters
- app: add GitHub repository link to header navigation
- app: support QGroundControl (.waypoints) and .poly file formats for import and export
