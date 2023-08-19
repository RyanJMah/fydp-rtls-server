from ctypes import Structure, c_uint8, c_uint32, c_int16, c_int32, c_float

ANCHOR_DATA_TOPIC = "gl/anchor/<id>/data"
IOS_DATA_TOPIC    = "gl/ios/<id>/data/<aid>"

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
