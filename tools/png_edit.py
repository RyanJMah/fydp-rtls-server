import os
import sys
from PIL import Image, ImageDraw
from dataclasses import dataclass
from typing import Tuple

import includes
from app_paths import AppPaths  # type: ignore

@dataclass
class Rectangle:
    bl_corner: Tuple[int, int]  # bottom left corner
    width:     int
    height:    int


RECTANGLES = [
    Rectangle( bl_corner=(50, 150),
               width=100,
               height=50 ),
    
    Rectangle( bl_corner=(200, 200),
               width=50,
               height=100 ),

    Rectangle( bl_corner=(300, 300),
               width=150,
               height=150 )
]

def draw_rectangle_on_image( img: Image,
                             bl_corner: Tuple[int, int],
                             width: int,
                             height: int,
                             thickness: int = 5,
                             rectangle_color: str = "black" ):
    """
    Draws a rectangle on an image with the coordinate system starting at the bottom left.

    :param img: The image to draw the rectangle on.
    :param bl_corner: A tuple (x, y) specifying the bottom left corner of the rectangle in the adjusted coordinate system.
    :param width: The width of the rectangle.
    :param height: The height of the rectangle.
    :param rectangle_color: Color of the rectangle outline.
    """

    # Get image dimensions
    img_width, img_height = img.size

    # Adjust the y-coordinate from bottom-left origin to top-left origin
    adjusted_y = img_height - bl_corner[1]

    # Calculate the rectangle coordinates in PIL coordinates
    top_left_corner = (bl_corner[0], adjusted_y - height)
    bottom_right_corner = (top_left_corner[0] + width, adjusted_y)

    # Initialize the drawing context
    draw = ImageDraw.Draw(img)

    # Draw the rectangle
    draw.rectangle( [top_left_corner, bottom_right_corner],
                    outline=rectangle_color,
                    width=thickness )


def main():
    png_filename = sys.argv[-1]

    basename = os.path.splitext(png_filename)[0].replace("_base", "")

    in_png_path = AppPaths.get_floorplan(png_filename)

    # Creating new floorplan png
    out_png_path = AppPaths.get_floorplan(f"{basename}_rect.png")

    with Image.open(in_png_path) as img:
        for rect in RECTANGLES:
            draw_rectangle_on_image( img,
                                     rect.bl_corner,
                                     rect.width,
                                     rect.height )

 
        # Save the modified image
        img.save(out_png_path)


if __name__ == "__main__":
    main()
