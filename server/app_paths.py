import os
import sys

class AppPaths():
    # ./server
    SERVER_DIR = os.path.dirname(os.path.abspath(__file__))

    # ./
    REPO_BASE_DIR = os.path.dirname(SERVER_DIR)

    # ./cpp
    CPP_DIR = os.path.join(REPO_BASE_DIR, "cpp")

    # ./cpp/build
    CPP_BUILD_DIR = os.path.join(CPP_DIR, "build")

    # ./resources
    RESOURCES_DIR = os.path.join(REPO_BASE_DIR, "resources")

    # ./resources/floorplans
    FLOORPLANS_DIR = os.path.join(RESOURCES_DIR, "floorplans")

    # ./resources/obj
    OBJ_DIR = os.path.join(RESOURCES_DIR, "obj")

    # ./resources/navmeshes
    NAVMESHES_DIR = os.path.join(RESOURCES_DIR, "navmeshes")

    # ./tests
    TESTS_DIR = os.path.join(REPO_BASE_DIR, "tests")

    # ./tools
    TOOLS_DIR = os.path.join(REPO_BASE_DIR, "tools")

    # ./server/gl_conf.json
    GL_CONF_FILE = os.path.join(SERVER_DIR, "gl_conf.json")

    @staticmethod
    def get_floorplan(name: str) -> str:
        return os.path.join(AppPaths.FLOORPLANS_DIR, name)

    @staticmethod
    def get_obj(name: str) -> str:
        return os.path.join(AppPaths.OBJ_DIR, name)

    @staticmethod
    def get_navmesh(name: str) -> str:
        return os.path.join(AppPaths.NAVMESHES_DIR, name)

# add compiled C++ shared library to python import path
sys.path.append(AppPaths.CPP_BUILD_DIR)

