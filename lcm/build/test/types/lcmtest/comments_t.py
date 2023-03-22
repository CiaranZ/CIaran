"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class comments_t(object):
    __slots__ = ["field_a", "field_b", "field_c", "field_d", "field_e", "array"]

    __typenames__ = ["int8_t", "int16_t", "int32_t", "int64_t", "float", "int32_t"]

    __dimensions__ = [None, None, None, None, None, ["field_a"]]

    const_field = 5

    def __init__(self):
        self.field_a = 0
        self.field_b = 0
        self.field_c = 0
        self.field_d = 0
        self.field_e = 0.0
        self.array = []

    def encode(self):
        buf = BytesIO()
        buf.write(comments_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bhiqf", self.field_a, self.field_b, self.field_c, self.field_d, self.field_e))
        buf.write(struct.pack('>%di' % self.field_a, *self.array[:self.field_a]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != comments_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return comments_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = comments_t()
        self.field_a, self.field_b, self.field_c, self.field_d, self.field_e = struct.unpack(">bhiqf", buf.read(19))
        self.array = struct.unpack('>%di' % self.field_a, buf.read(self.field_a * 4))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if comments_t in parents: return 0
        tmphash = (0x4c8ffa86357c7d7e) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if comments_t._packed_fingerprint is None:
            comments_t._packed_fingerprint = struct.pack(">Q", comments_t._get_hash_recursive([]))
        return comments_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", comments_t._get_packed_fingerprint())[0]

