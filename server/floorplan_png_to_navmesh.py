import os
import meshio
import numpy as np
from paths import AppPaths
from PIL import Image
from pydelatin import Delatin

def png_to_heightmap(png_path: str) -> Image:
    img = Image.open(png_path)
    img = img.convert("L")
    return img

def heightmap_to_mesh(heightmap: Image) -> meshio.Mesh:
    tin = Delatin(
            np.array(heightmap),
            width=heightmap.width,
            height=heightmap.height,
            base_height=50,
            invert=True )
    vertices, triangles = tin.vertices, tin.triangles

    cells = [("triangle", triangles)]
    mesh = meshio.Mesh(vertices, cells)

    return mesh

def write_mesh_to_obj(mesh: meshio.Mesh, outpath: str):
    # Rotate the mesh by 90 degrees in the x-axis
    rotation_matrix = np.array([
                            [1, 0,  0],
                            [0, 0, -1],
                            [0, 1,  0] ])

    mesh.points = np.dot(mesh.points, rotation_matrix.T)
    mesh.write(outpath)


def _main():
    png_path = AppPaths.get_floorplan("e7-4th-floor.png")
    heightmap = png_to_heightmap(png_path)
    mesh = heightmap_to_mesh(heightmap)

    output_path = AppPaths.get_obj("e7-4th-floor.obj")
    write_mesh_to_obj(mesh, output_path)

if __name__ == "__main__":
    _main()
