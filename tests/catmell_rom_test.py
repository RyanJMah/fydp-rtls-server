import time
import matplotlib.pyplot as plt

import test_paths
from catmell_rom_splines import catmull_rom_chain


if __name__ == "__main__":
    POINTS = [ (0, 1.5), (2, 2), (3, 1), (4, 0.5), (5, 1), (6, 2), (7, 3) ]  # Red points
    NUM_POINTS: int = 100  # Density of blue chain points between two red points

    start = time.time()

    chain_points: list = catmull_rom_chain(POINTS)
    # assert len(chain_points) == num_segments(POINTS) * NUM_POINTS  # 400 blue points for this example

    print(f"Time taken: {time.time() - start:.3f}s")

    print(f"Number of points: {len(chain_points)}")

    plt.plot(*zip(*chain_points), c="blue")
    plt.plot(*zip(*POINTS), c="red", linestyle="none", marker="o")
    plt.show()