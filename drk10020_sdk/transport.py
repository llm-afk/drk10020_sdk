"""One synchronous serial owner, fixed 20 Hz maximum, no automatic writes."""
from collections import Counter, deque
import math
import re
import threading
import time
import uuid

from .protocol import ProtocolError, StreamDecoder, WriteUncertainError


class Connection:
    def __init__(self, port, timeout=1.0, *, _serial_factory=None, _clock=time):
        if not isinstance(port, str) or not port:
            raise ValueError("port must be a nonempty COM port name")
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if _serial_factory is None:
            import serial
            _serial_factory = serial.Serial
        self.port, self.timeout, self._clock = port, float(timeout), _clock
        self.connection_id = uuid.uuid4().hex
        self._events, self._replies = deque(), deque()
        self._decoder, self._counts = StreamDecoder(), Counter()
        self._sequence = self._frame_sequence = 0
        self._last_send = self._last_receive = -math.inf
        self._pending = None
        self._uncertain = self.closed = False
        self._serial = _serial_factory(port=None, baudrate=115200, bytesize=8,
                                       parity="N", stopbits=1, timeout=min(.01, timeout),
                                       write_timeout=timeout, xonxoff=False,
                                       rtscts=False, dsrdtr=False)
        self._serial.dtr = self._serial.rts = False
        self._serial.port = port
        self._record("open_attempt")
        try:
            self._serial.open()
            model = self._text(b"ATGETV\r\n")
            model = re.sub(r"^ATGETV[=:]", "", model).strip()
            if model != "DRK10020":
                raise ProtocolError(f"expected DRK10020, received {model!r}")
            firmware = self._text(b"ATVER\r\n")
            current = self.settings()
            panel = self._panel()
            self._compensation = bool(self._settings.flags & 2) if current["enabled"] else True
            self.info = dict(port=port, baudrate=115200, model=model,
                             firmware=firmware, panel=panel, initial_settings=current)
            self._record("opened")
        except BaseException as exc:
            self._serial.close()
            self.closed = True
            self._record("open_failed", error=repr(exc))
            exc.events = self.drain_events()
            raise

    def _record(self, kind, raw=b"", **extra):
        self._sequence += 1
        event = dict(connection_id=self.connection_id, sequence=self._sequence,
                     host_ns=self._clock.time_ns(),
                     monotonic_ns=self._clock.perf_counter_ns(), kind=kind,
                     raw=bytes(raw), **extra)
        self._events.append(event)
        self._counts[kind] += 1
        return event

    def _publish(self, packets):
        for packet in packets:
            extra = {}
            if packet.kind in ("settings", "measurement", "invalid"):
                self._frame_sequence += 1
                extra["receive_sequence"] = self._frame_sequence
            value = packet.value
            if packet.kind == "settings":
                value = value.to_dict()
            event = self._record("rx_"+packet.kind, packet.raw,
                                 data=value, error=packet.error, **extra)
            if packet.kind == "settings":
                event["_settings"] = packet.value
            if packet.kind in ("settings", "measurement", "text", "invalid"):
                self._replies.append(event)

    def _pump(self, wait=False):
        waiting = self._serial.in_waiting
        chunk = self._serial.read(waiting or 1) if waiting or wait else b""
        if chunk:
            self._last_receive = self._clock.perf_counter()
            self._counts["rx_bytes"] += len(chunk)
            self._record("rx_chunk", chunk)
            self._publish(self._decoder.feed(chunk))
        elif self._clock.perf_counter()-self._last_receive >= min(.08, self.timeout/3):
            self._publish(self._decoder.flush_idle_text())

    def _check(self, *, pending=False):
        if self.closed:
            raise RuntimeError("connection is closed")
        if self._pending is not None and not pending:
            raise RuntimeError("receive the pending measurement with read() first")

    def _write(self, payload, *, control=False):
        self._check(pending=True)
        # A nonblocking drain preserves unsolicited/late data before the request.
        # No input reset, flushing away bytes, or response queue replacement.
        self._pump()
        # perf_counter uses QPC on Windows Python 3.11; monotonic uses the
        # ~15.6 ms GetTickCount64 clock there and cannot enforce a 50 ms gap.
        interval = .05 if control else .1
        delay = interval - (self._clock.perf_counter()-self._last_send)
        while delay > 0:
            self._clock.sleep(delay)
            delay = interval - (self._clock.perf_counter()-self._last_send)
        # Data may have arrived during the rate wait; retain it before watermark.
        self._pump()
        watermark = self._sequence
        self._record("tx_attempt", payload)
        self._last_send = self._clock.perf_counter()
        try:
            written = self._serial.write(payload)
            self._counts["tx_bytes"] += written
            self._record("tx_done", payload[:written], written=written)
            if written != len(payload):
                raise OSError(f"partial write: {written}/{len(payload)} bytes")
        except BaseException as exc:
            self._record("tx_error", payload, error=repr(exc))
            raise
        return watermark

    def _wait(self, kind, watermark):
        deadline = self._clock.perf_counter()+self.timeout
        bad = False
        while True:
            while self._replies:
                event = self._replies.popleft()
                # Complete evidence remains in _events even when not a reply.
                if event["sequence"] <= watermark:
                    continue
                bad |= event["kind"] == "rx_invalid"
                if event["kind"] == "rx_"+kind:
                    return event
            if self._clock.perf_counter() >= deadline:
                break
            self._pump(wait=True)
        self._record("timeout", expected=kind)
        if bad:
            raise ProtocolError(f"corrupt reply; no complete valid {kind} before timeout")
        raise TimeoutError(f"no complete {kind} reply before timeout")

    def _text(self, command):
        self._check()
        return self._wait("text", self._write(command))["data"]

    def _panel(self):
        panel = self._text(b"ATTBS\r\n")
        match = re.search(r"(?:^|,)Mast_Slove=([01])(?:,|$)", panel)
        if match is None:
            raise ProtocolError(f"unexpected ATTBS response: {panel!r}")
        self._master = match[1] == "0"
        return panel

    def settings(self):
        """Read current setpoints/output flags with ATRCON, never a control frame."""
        self._check()
        event = self._wait("settings", self._write(b"ATRCON\r\n"))
        self._settings = event["_settings"]
        return dict(event["data"])

    def _begin_control(self, desired):
        self._pending = (desired, None, threading.get_ident())
        try:
            watermark = self._write(desired.to_bytes(), control=True)
            self._pending = (desired, watermark, threading.get_ident())
        except BaseException as exc:
            self._pending = None
            self._uncertain = True
            raise WriteUncertainError("control transmission unconfirmed; no automatic retry") from exc

    def send(self, voltage, current):
        """Send V and current limit A once, keeping cached output/protection flags."""
        self._check()
        if self._uncertain:
            raise WriteUncertainError("previous control uncertain; inspect settings and reopen")
        if not self._master:
            raise ProtocolError("USB control requires master mode Mast_Slove=0")
        desired = self._settings.update(voltage=voltage, current=current)
        self._begin_control(desired)

    def read(self):
        """Receive the pending measurement; this method never transmits."""
        self._check(pending=True)
        if self._pending is None:
            raise RuntimeError("no pending measurement; call send(voltage, current) first")
        desired, watermark, owner = self._pending
        if threading.get_ident() != owner:
            raise RuntimeError("send() and read() must have the same thread owner")
        try:
            event = self._wait("measurement", watermark)
            self._settings = desired  # requested cache, not proof of device readback
            return dict(event["data"], raw=event["raw"],
                        receive_sequence=event["receive_sequence"],
                        host_ns=event["host_ns"], monotonic_ns=event["monotonic_ns"],
                        connection_id=self.connection_id)
        except BaseException as exc:
            self._uncertain = True
            raise WriteUncertainError("control sent; measurement unconfirmed; no automatic retry") from exc
        finally:
            self._pending = None

    def exchange(self, voltage, current):
        self.send(voltage, current)
        return self.read()

    def set_enabled(self, enabled):
        self._check()
        if type(enabled) is not bool:
            raise ValueError("enabled must be bool")
        if enabled and self._uncertain:
            raise WriteUncertainError("previous control uncertain; reopen before enable")
        self._panel()
        if not self._master:
            raise ProtocolError("USB control requires master mode Mast_Slove=0")
        current = self.settings()
        if current["enabled"]:
            self._compensation = current["compensation"]
        flags = self._settings.flags & ~3
        if enabled:
            flags |= 3 if self._compensation else 1
        desired = self._settings.update(flags=flags)
        try:
            # Explicit disable is allowed after uncertainty; never auto-issued.
            self._begin_control(desired)
            self.read()
            actual = self.settings()
            if self._settings != desired:
                raise ProtocolError(f"settings readback mismatch: {actual!r}")
        except BaseException as exc:
            self._uncertain = True
            if isinstance(exc, WriteUncertainError):
                raise
            raise WriteUncertainError("output switch readback unconfirmed; inspect panel") from exc
        self._uncertain = False
        return actual

    def drain_events(self):
        """Transfer all queued RX chunks/frames, invalid/noise and TX evidence."""
        events = [dict(event) for event in self._events]
        self._events.clear()
        for event in events:
            event.pop("_settings", None)
        return events

    def stats(self):
        return dict(self._counts, pending_events=len(self._events),
                    buffered_bytes=len(self._decoder.buffer), uncertain=self._uncertain,
                    closed=self.closed, connection_id=self.connection_id)

    def close(self):
        """Close serial only; preserve queued evidence and never send OFF."""
        if self.closed:
            return
        try:
            self._pump()
        finally:
            self._serial.close()
            self.closed = True
            if self._decoder.buffer:
                self._record("incomplete", bytes(self._decoder.buffer))
                self._decoder.buffer.clear()
            if self._pending is not None:
                self._uncertain = True
                self._record("pending_unconfirmed")
            self._record("closed")
