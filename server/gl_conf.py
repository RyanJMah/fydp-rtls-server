import json
from dataclasses import dataclass
from typing import List
from app_paths import AppPaths

@dataclass
class AnchorConf:
    id: int
    x: float
    y: float
    z: float

@dataclass
class GuidingLiteConf:
    broker_address: str
    broker_port: int

    anchors: List[AnchorConf]
    num_anchors: int
    

with open(AppPaths.GL_CONF_FILE, "r") as f:
    _gl_conf = json.load(f)

GL_CONF = GuidingLiteConf(
    broker_address = _gl_conf["broker_address"],
    broker_port    = _gl_conf["broker_port"],
    num_anchors    = len(_gl_conf["anchors"]),
    anchors        = [ AnchorConf( id = anchor["id"],
                                   x  = anchor["x"],
                                   y  = anchor["y"],
                                   z  = anchor["z"] ) for anchor in _gl_conf["anchors"] ],
)

