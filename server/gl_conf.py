import os
import commentjson as json
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Any
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
class DebugEndpoint_Conf:
    host: str
    port: int
    enabled: bool

    @classmethod
    def from_dict(cls, d: Dict[Any, Any]):
        return cls(
            host    = d["host"],
            port    = d["port"],
            enabled = d["enabled"],
        )

@dataclass
class GuidingLiteConf:
    broker_address: str
    broker_port: int

    ios_magnetometer_offset: float
    ios_magnetometer_multiplier: float

    navmesh_path: str
    real_life_to_floorplan_png_scale: float

    anchors: Dict[int, AnchorConf]

    debug_endpoint: DebugEndpoint_Conf

    debug_manual_position_control: bool

    update_period_secs: float

    num_anchors: int = field(init=False)
    anchor_coords: Dict[ int, Tuple[float, float, float] ] = field(default_factory=dict)
    
    def __post_init__(self):
        self.num_anchors   = len(self.anchors)
        self.anchor_coords = { i: a.get_coords() for i, a in self.anchors.items() }

    @staticmethod
    def anchors_from_dict(d: Dict[Any, Any]):
        return { a["id"]: AnchorConf.from_dict(a) for a in d["anchors"] }


with open(AppPaths.GL_CONF_FILE, "r") as f:
    _gl_conf = json.load(f)


# dump back as json string, removes comments
GL_CONF_JSON = json.dumps(_gl_conf)

GL_CONF = GuidingLiteConf(
    broker_address                   = _gl_conf["broker_address"],
    broker_port                      = _gl_conf["broker_port"],
    ios_magnetometer_offset          = _gl_conf["ios_magnetometer_offset"],
    ios_magnetometer_multiplier      = 1.0 if not _gl_conf["reverse_magnetometer_polarity"] else -1.0,
    navmesh_path                     = os.path.join( os.path.dirname(AppPaths.GL_CONF_FILE), _gl_conf["navmesh_relative_path"] ),
    real_life_to_floorplan_png_scale = _gl_conf["real_life_to_floorplan_png_scale"],
    update_period_secs               = 1 / _gl_conf["global_update_frequency_Hz"],
    anchors                          = GuidingLiteConf.anchors_from_dict(_gl_conf),
    debug_endpoint                   = DebugEndpoint_Conf.from_dict(_gl_conf["debug_endpoint"]),
    debug_manual_position_control    = _gl_conf["debug_manual_position_control"],
)
