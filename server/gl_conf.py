import json
from dataclasses import dataclass
from typing import List, Dict, Any
from app_paths import AppPaths

@dataclass
class AnchorConf:
    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, d: Dict[Any, Any]):
        return cls(
            x = d["x"],
            y = d["y"],
            z = d["z"],
        )

    def get_coords(self):
        return (self.x, self.y, self.z)

@dataclass
class GuidingLiteConf:
    broker_address: str
    broker_port: int

    anchors: Dict[int, AnchorConf]
    num_anchors: int

    loc_debug_endpoint: bool
    
    @staticmethod
    def anchors_from_dict(d: Dict[Any, Any]):
        return { a["id"]: AnchorConf.from_dict(a) for a in d["anchors"] }


with open(AppPaths.GL_CONF_FILE, "r") as f:
    _gl_conf = json.load(f)

GL_CONF = GuidingLiteConf(
    broker_address     = _gl_conf["broker_address"],
    broker_port        = _gl_conf["broker_port"],
    num_anchors        = len(_gl_conf["anchors"]),
    anchors            = GuidingLiteConf.anchors_from_dict(_gl_conf),
    loc_debug_endpoint = _gl_conf["localization_service_debug_endpoint_enabled"],
)
