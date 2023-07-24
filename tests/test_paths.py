import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

REPO_ROOT_DIR = os.path.dirname(_THIS_DIR)
SERVER_DIR    = os.path.join(REPO_ROOT_DIR, "server")

sys.path.append(SERVER_DIR)
sys.path.append(REPO_ROOT_DIR)
