"""Simple single-device USB API; no writes at import or automatic output actions."""
from collections import deque
import threading

from .protocol import ProtocolError, WriteUncertainError
from .transport import Connection

__version__ = "0.1.0"
__all__ = ["open", "close", "enable", "disable", "send", "read", "exchange",
           "settings", "drain_events", "stats", "ProtocolError", "WriteUncertainError"]
_device = None
_retired = deque()
_lock = threading.RLock()


def _connection():
    if _device is None:
        raise RuntimeError("call drk10020.open() first")
    return _device


def open(port=None, *, timeout=1.0):
    """Validate one DRK10020 on CH340 COM ports, or an explicit port. Query only."""
    global _device
    with _lock:
        if _device is not None:
            raise RuntimeError("already open; close() first")
        if port is None:
            from serial.tools.list_ports import comports
            candidates = sorted({p.device for p in comports()
                                 if (p.vid, p.pid) == (0x1A86, 0x7523)})
        elif isinstance(port, str) and port:
            candidates = [port]
        else:
            raise ValueError("port must be a nonempty string or None")
        found, errors = [], []
        try:
            for name in candidates:
                try:
                    found.append(Connection(name, timeout))
                except Exception as exc:
                    _retired.extend(getattr(exc, "events", ()))
                    errors.append(f"{name}: {exc}")
                    if port is not None:
                        raise
            if len(found) != 1:
                names = [device.port for device in found]
                raise RuntimeError(f"expected one DRK10020, found {names}; "
                                   f"specify open(port) if multiple; probe errors: {errors}")
            _device = found.pop()
            info = dict(_device.info)
            info["initial_settings"] = dict(info["initial_settings"])
            return info
        finally:
            for device in found:
                device.close()
                _retired.extend(device.drain_events())


def close():
    """Release COM without changing output. drain_events() also works after close."""
    global _device
    with _lock:
        if _device is not None:
            device, _device = _device, None
            try:
                device.close()
            finally:
                _retired.extend(device.drain_events())


def settings():
    with _lock:
        return _connection().settings()


def enable():
    with _lock:
        return _connection().set_enabled(True)


def disable():
    with _lock:
        return _connection().set_enabled(False)


def send(voltage, current):
    with _lock:
        return _connection().send(voltage, current)


def read():
    with _lock:
        return _connection().read()


def exchange(voltage, current):
    with _lock:
        return _connection().exchange(voltage, current)


def drain_events():
    with _lock:
        events = list(_retired)
        _retired.clear()
        if _device is not None:
            events.extend(_device.drain_events())
        return events


def stats():
    with _lock:
        return _connection().stats()
