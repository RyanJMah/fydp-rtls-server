from ctypes import Structure, c_uint8, c_uint32, c_int16, c_int32, c_float

ANCHOR_HEARTBEAT_TOPIC   = "gl/anchor/<aid>/heartbeat"
ANCHOR_DATA_TOPIC        = "gl/anchor/<aid>/data"
ANCHOR_CONN_REQ_TOPIC    = "gl/anchor/<aid>/conn/req"
ANCHOR_CONN_RESP_TOPIC   = "gl/anchor/<aid>/conn/resp"
ANCHOR_NI_CONFIG_TOPIC   = "gl/anchor/<aid>/conn/ni_config"
ANCHOR_CONFIG_BASE_TOPIC = "gl/anchor/<aid>/config"
ANCHOR_DFU_TOPIC         = "gl/anchor/<aid>/config"

IOS_BASE_TOPIC      = "gl/user/<uid>"
IOS_HEARTBEAT_TOPIC = "gl/user/<uid>/heartbeat"
IOS_DATA_TOPIC      = "gl/user/<uid>/data/<aid>"

SERVER_HEARTBEAT_TOPIC          = "gl/server/heartbeat"
SERVER_PATHFINDING_CONFIG_TOPIC = "gl/server/pathfinding/config"

def insert_uid(topic: str, uid: str) -> str:
    return topic.replace("<uid>", uid)

class Struct(Structure):
    def __str__(self):
        ret = "{\n"

        for field_name, field_type in self._fields_:
            ret += " "*4 + f"{field_name}: {getattr(self, field_name)}\n"

        ret += "}"
        return ret

class AnchorTelemetryData(Struct):
    _fields_ = [ ("status",      c_uint8),
                 ("timestamp",   c_uint32),
                 ("distance_mm", c_int32) ]

class IOS_TelemetryData(Struct):
    _pack_ = 1
    _fields_ = [ ("distance_m",    c_float),
                 ("azimuth_deg",   c_int16),
                 ("elevation_deg", c_int16),
                 ("los",           c_uint8) ]
