from ctypes import Structure, c_uint8, c_uint32, c_int16, c_int32

class Struct(Structure):
    def __str__(self):
        ret = "{\n"

        for field_name, field_type in self._fields_:
            ret += " "*4 + f"{field_name}: {getattr(self, field_name)}\n"

        ret += "}"
        return ret

class TelemetryData(Struct):
    _fields_ = [ ("status",             c_uint8),
                 ("timestamp",          c_uint32),
                 ("distance_mm",        c_int32) ]
