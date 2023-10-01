file ../.venv/bin/python3
set args ../server/floorplan_png_to_navmesh.py

set breakpoint pending on

# b NavmeshBuilder::handleBuild
b generate_navmesh
