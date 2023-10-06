import os
import meshio
import numpy as np
from PIL import Image
from pydelatin import Delatin

from app_paths import AppPaths
import lib_pathfinding as cpp   # type: ignore

BASE_HEIGHT = 100

def png_to_heightmap(png_path: str) -> Image:
    img = Image.open(png_path)
    img = img.convert("L")
    return img

def heightmap_to_mesh(heightmap: Image) -> meshio.Mesh:
    tin = Delatin(
            np.array(heightmap),
            width=heightmap.width,
            height=heightmap.height,
            base_height=BASE_HEIGHT,
            invert=True )
    vertices, triangles = tin.vertices, tin.triangles

    cells = [("triangle", triangles)]
    mesh = meshio.Mesh(vertices, cells)

    return mesh

def write_mesh_to_obj(mesh: meshio.Mesh, outpath: str):
    # Recast navigation uses y as the height axis, so swap y and z

    for i, p in enumerate(mesh.points):
        tmp = p[1]
        mesh.points[i][1] = p[2]
        mesh.points[i][2] = tmp

        mesh.points[i][1] += BASE_HEIGHT

    mesh.write(outpath)


def _main():
    png_path = AppPaths.get_floorplan("ssdc-testing-area.png")
    heightmap = png_to_heightmap(png_path)
    mesh = heightmap_to_mesh(heightmap)

    obj_path = AppPaths.get_obj("ssdc-testing-area.obj")
    write_mesh_to_obj(mesh, obj_path)

    navmesh_path = AppPaths.get_navmesh("ssdc-testing-area.nav")
    cpp.generate_navmesh(obj_path, navmesh_path)

if __name__ == "__main__":
    _main()
