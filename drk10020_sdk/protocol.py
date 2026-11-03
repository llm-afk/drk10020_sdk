"""DRK USB panel frames. Public floating point units: V, A, W, degrees C.

The supplied USB spreadsheet's mV/mA float annotations conflict with the
vendor host implementation. Three-byte auxiliary voltages really use mV.
No RS485 register or flag definition is used here.
"""
from dataclasses import dataclass, replace
import math
import struct


class ProtocolError(RuntimeError):
    """Invalid device identity, reply or frame; raw evidence remains available."""


class WriteUncertainError(RuntimeError):
    """A control was attempted but its result is unconfirmed; never auto-retry."""


@dataclass(frozen=True)
class Settings:
    flags: int
    voltage: float
    current: float
    ovp: float
    ocp: float

    def to_bytes(self):
        body = b"\xcc\xdd\x01" + bytes([self.flags])
        body += struct.pack(">4f", self.voltage, self.current, self.ovp, self.ocp)
        body += b"\x77\x88"
        return body + bytes([sum(body) & 255])

    def to_dict(self):
        return dict(flags=self.flags, voltage=self.voltage, current=self.current,
                    ovp=self.ovp, ocp=self.ocp, enabled=bool(self.flags & 1),
                    compensation=bool(self.flags & 2), ovp_enabled=bool(self.flags & 4),
                    ocp_enabled=bool(self.flags & 8), remote_sense=bool(self.flags & 16),
                    cc_priority=bool(self.flags & 64))

    def update(self, **changes):
        limits = {"voltage": (0, 100), "current": (.001, 20)}
        for name, value in changes.items():
            if name == "flags":
                if type(value) is not int or not 0 <= value <= 255:
                    raise ValueError("flags must be an unsigned byte")
            else:
                low, high = limits[name]
                if isinstance(value, bool):
                    raise ValueError(f"{name} must be numeric")
                value = float(value)
                if not math.isfinite(value) or not low <= value <= high:
                    raise ValueError(f"{name} must be in [{low}, {high}]")
                changes[name] = value
        # Exactly the same float32 representation as subsequent readback.
        return parse_settings(replace(self, **changes).to_bytes())


def valid_frame(raw, header, tail, length):
    return (len(raw) == length and raw[:2] == header and raw[-3:-1] == tail
            and sum(raw[:-1]) & 255 == raw[-1])


def parse_settings(raw):
    if not valid_frame(raw, b"\xcc\xdd", b"\x77\x88", 23) or raw[2] != 1:
        raise ProtocolError("invalid 23-byte USB settings frame")
    values = struct.unpack_from(">4f", raw, 4)
    if not all(math.isfinite(x) and x >= 0 for x in values):
        raise ProtocolError("non-finite or negative settings value")
    return Settings(raw[3], *values)


def parse_measurement(raw):
    if not valid_frame(raw, b"\xaa\xbb", b"\x55\x66", 288):
        raise ProtocolError("invalid 288-byte USB measurement frame")
    mcu_v, mcu_i = struct.unpack_from(">2f", raw, 13)
    samples = struct.unpack_from(">66f", raw, 21)
    if not all(math.isfinite(x) for x in (mcu_v, mcu_i, *samples)):
        raise ProtocolError("non-finite measurement value")
    vs, currents, remotes = samples[:22], samples[22:44], samples[44:]
    flags, fault = raw[3], int.from_bytes(raw[4:6], "big")
    return dict(voltage=sum(vs)/22, current=sum(currents)/22,
                power=sum(v*i for v, i in zip(vs, currents))/22,
                remote_voltage=sum(remotes)/22, temperature=raw[6],
                aux_voltage=int.from_bytes(raw[7:10], "big")/1000,
                acdc_voltage=int.from_bytes(raw[10:13], "big")/1000,
                mcu_voltage=mcu_v, mcu_current=mcu_i,
                voltage_samples=vs, current_samples=currents,
                remote_voltage_samples=remotes, status=flags,
                panel_status=raw[2], fault=fault, enabled=bool(flags & 1),
                running=not bool(flags & 128), constant_voltage=bool(flags & 8),
                ovp_tripped=bool(flags & 16), ocp_tripped=bool(flags & 32),
                power_limit_w=400 if flags & 4 else 600)


@dataclass(frozen=True)
class Packet:
    kind: str
    raw: bytes
    value: object = None
    error: str | None = None


class StreamDecoder:
    """Persistent framing; never treats a serial read as a packet boundary."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while self.buffer:
            data = bytes(self.buffer)
            if data[:2] in (b"\xcc\xdd", b"\xaa\xbb"):
                settings = data[:2] == b"\xcc\xdd"
                length, tail = (23, b"\x77\x88") if settings else (288, b"\x55\x66")
                if len(data) < length:
                    break
                raw = data[:length]
                if not valid_frame(raw, data[:2], tail, length):
                    packets.append(Packet("invalid", raw, error="frame tail/checksum mismatch"))
                    del self.buffer[0]
                    continue
                del self.buffer[:length]
                try:
                    value = parse_settings(raw) if settings else parse_measurement(raw)
                    packets.append(Packet("settings" if settings else "measurement", raw, value))
                except ProtocolError as exc:
                    packets.append(Packet("invalid", raw, error=str(exc)))
                continue
            starts = [n for n in (data.find(b"\xcc\xdd"), data.find(b"\xaa\xbb")) if n >= 0]
            start = min(starts) if starts else len(data)
            newline = data.find(b"\r\n")
            if 0 <= newline < start:
                raw = data[:newline+2]
                del self.buffer[:newline+2]
                packets.append(self._text_or_noise(raw))
            elif start < len(data):
                raw = data[:start]
                del self.buffer[:start]
                packets.append(Packet("noise", raw))
            elif all(x in (9, 10, 13) or 32 <= x < 127 for x in data):
                break  # ATTBS has no terminator; flush after inter-byte idle.
            else:
                keep = 1 if data[-1] in (0xcc, 0xaa) else 0
                length = len(data)-keep
                if not length:
                    break
                packets.append(Packet("noise", data[:length]))
                del self.buffer[:length]
        return packets

    @staticmethod
    def _text_or_noise(raw):
        try:
            text = raw.decode("ascii").strip()
            if all(c in "\r\n\t" or 32 <= ord(c) < 127 for c in text):
                return Packet("text", raw, text)
        except UnicodeDecodeError:
            pass
        return Packet("noise", raw)

    def flush_idle_text(self):
        if not self.buffer or self.buffer[:1] in (b"\xcc", b"\xaa"):
            return []
        raw = bytes(self.buffer)
        if all(x in (9, 10, 13) or 32 <= x < 127 for x in raw):
            self.buffer.clear()
            return [self._text_or_noise(raw)]
        return []
