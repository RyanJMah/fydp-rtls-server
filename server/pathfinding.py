import os
import sys

from app_paths import AppPaths
import lib_pathfinding as cpp     # type: ignore

def _main():
    navmesh_path = AppPaths.get_navmesh("e7-4th-floor.nav")

    pf = cpp.Pathfinder()
    pf.load_navmesh(navmesh_path)

    print( pf.find_path( (100, 100, 0), (500, 400, 0) ) )

if __name__ == "__main__":
    _main()