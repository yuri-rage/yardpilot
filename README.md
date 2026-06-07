![YardPilot](assets/logo.png)

YardPilot is an advanced, client-side coverage path planner designed to calculate optimized trajectories for autonomous lawnmowers, agricultural rovers, and survey applications.

While the core trajectory planning math is completely self-contained and runs offline (falling back to custom geometry code if Clipper is unavailable), the web interface relies on public CDNs to load map tiles, external fonts, and libraries (Leaflet and ClipperLib).

### KEY FEATURES

*   **Boundary Subtraction**: Supports importing perimeter boundaries (`.poly` or `.waypoints`) and subtracting multiple internal exclusion zones (`.waypoints`) to outline the navigable area.
*   **Optimal Sweep Planning**: Computes mathematically optimal boustrophedon sweep angles based on the Minimum Bounding Box (MBB) of the field's convex hull to minimize turns (can be manually overridden).
*   **Multi-Pass Perimeter Laps**: Generates nested outer boundary passes with smooth spiral transitions.
*   **Collision-Free Routing**: Integrates a Delaunay-Voronoi roadmap and Dijkstra shortest-path search to route transit legs around exclusion zones.

### QUICK START

1.  Clone this repository to your local machine:
    ```bash
    git clone https://github.com/your-repo/yardpilot.git
    ```
2.  Open [`index.html`](index.html) directly in any modern web browser.
3.  Drag and drop your field perimeter (`.poly` or `.waypoints`) into the perimeter drop zone to start planning.
