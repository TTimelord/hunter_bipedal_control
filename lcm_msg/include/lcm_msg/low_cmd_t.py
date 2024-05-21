"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class low_cmd_t(object):
    __slots__ = ["timestamp", "joint_pos", "joint_vel", "joint_torque", "ff_tau", "kp", "kd"]

    __typenames__ = ["int64_t", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, [12], [12], [12], [12], [12], [12]]

    def __init__(self):
        self.timestamp = 0
        self.joint_pos = [ 0.0 for dim0 in range(12) ]
        self.joint_vel = [ 0.0 for dim0 in range(12) ]
        self.joint_torque = [ 0.0 for dim0 in range(12) ]
        self.ff_tau = [ 0.0 for dim0 in range(12) ]
        self.kp = [ 0.0 for dim0 in range(12) ]
        self.kd = [ 0.0 for dim0 in range(12) ]

    def encode(self):
        buf = BytesIO()
        buf.write(low_cmd_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>12d', *self.joint_pos[:12]))
        buf.write(struct.pack('>12d', *self.joint_vel[:12]))
        buf.write(struct.pack('>12d', *self.joint_torque[:12]))
        buf.write(struct.pack('>12d', *self.ff_tau[:12]))
        buf.write(struct.pack('>12d', *self.kp[:12]))
        buf.write(struct.pack('>12d', *self.kd[:12]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != low_cmd_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return low_cmd_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = low_cmd_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.joint_pos = struct.unpack('>12d', buf.read(96))
        self.joint_vel = struct.unpack('>12d', buf.read(96))
        self.joint_torque = struct.unpack('>12d', buf.read(96))
        self.ff_tau = struct.unpack('>12d', buf.read(96))
        self.kp = struct.unpack('>12d', buf.read(96))
        self.kd = struct.unpack('>12d', buf.read(96))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if low_cmd_t in parents: return 0
        tmphash = (0x268d0e7269d76457) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if low_cmd_t._packed_fingerprint is None:
            low_cmd_t._packed_fingerprint = struct.pack(">Q", low_cmd_t._get_hash_recursive([]))
        return low_cmd_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", low_cmd_t._get_packed_fingerprint())[0]

