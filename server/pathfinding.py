import os
import sys

THIS_DIR      = os.path.dirname(os.path.abspath(__file__))
BASE_REPO_DIR = os.path.dirname(THIS_DIR)
CPP_BUILD_DIR = os.path.join(BASE_REPO_DIR, "cpp", "build")

sys.path.append(CPP_BUILD_DIR)

from lib_pathfinding import generate_navmesh

if __name__ == "__main__":
    obj_path = os.path.join(THIS_DIR, "out.obj")
    generate_navmesh(obj_path, "./out.nav")

