import os
import meshio
import numpy as np
from PIL import Image
from pydelatin import Delatin
from pydelatin.util import rescale_positions

THIS_DIR = os.path.dirname(os.path.abspath(__file__))

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
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    mesh.points = np.dot(mesh.points, rotation_matrix.T)

    mesh.write(outpath)



def main():
    png_path = "/Users/ryanmah/Downloads/067E7_04FLR-1.png"
    # png_path = "/Users/ryanmah/Downloads/Winkel_triple_projection_SW.jpg"

    heightmap = png_to_heightmap(png_path)

    mesh = heightmap_to_mesh(heightmap)

    output_path = os.path.join(THIS_DIR, "out.obj")
    write_mesh_to_obj(mesh, output_path)


    # scaled_heightmap = (heightmap*255).astype(np.uint8)
    # out_img = Image.fromarray(scaled_heightmap)

    # out_img.save(output_path)

if __name__ == "__main__":
    main()
