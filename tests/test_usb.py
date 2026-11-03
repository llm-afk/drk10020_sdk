"""Offline protocol/serial-contract tests; no physical ports are opened."""
import math
import struct
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import drk10020_sdk
from drk10020_sdk.protocol import (ProtocolError, Settings, StreamDecoder,
                              WriteUncertainError, parse_measurement, parse_settings)
from drk10020_sdk.transport import Connection


def measurement(settings=None, *, fault=0, samples=None):
    settings = settings or Settings(0, 5., .1, 110., 23.)
    flags = 8 | (settings.flags & 1)
    body = b"\xaa\xbb\x00" + bytes([flags]) + fault.to_bytes(2, "big") + b"\x23"
    body += (12000).to_bytes(3, "big") + (130000).to_bytes(3, "big")
    body += struct.pack(">2f", settings.voltage if settings.flags & 1 else 0., 0.)
    values = samples or ([settings.voltage if settings.flags & 1 else 0.]*22
                         + [0.]*22 + [0.]*22)
    body += struct.pack(">66f", *values) + b"\x55\x66"
    return body + bytes([sum(body) & 255])


class Clock:
    def __init__(self): self.now = 1.
    def monotonic(self): return self.now
    def monotonic_ns(self): return round(self.now*1e9)
    def perf_counter(self): return self.now
    def perf_counter_ns(self): return round(self.now*1e9)
    def time_ns(self): return 1700000000000000000+self.monotonic_ns()
    def sleep(self, delay): self.now += delay


class FakeSerial:
    def __init__(self, clock, **kwargs):
        self.clock, self.kwargs = clock, kwargs
        self.port = None
        self.is_open = False
        self.buffer = bytearray()
        self.writes, self.sent_times, self.returned = [], [], bytearray()
        self.current = Settings(0, 32., 1., 110., 23.)
        self.model, self.master = "DRK10020", 0
        self.drop = self.partial = self.corrupt = self.mismatch = False
        self.extra = b""
        self.chunk = 10000
        self.fault = 0

    def open(self): self.is_open = True
    def close(self): self.is_open = False
    @property
    def in_waiting(self): return len(self.buffer)
    def read(self, length):
        if not self.buffer:
            self.clock.sleep(.01)
            return b""
        data = bytes(self.buffer[:min(length, self.chunk)])
        del self.buffer[:len(data)]
        self.returned.extend(data)
        return data

    def write(self, raw):
        self.writes.append(raw)
        self.sent_times.append(self.clock.now)
        if raw == b"ATGETV\r\n": reply = self.model.encode()+b"\r\n"
        elif raw == b"ATVER\r\n": reply = b"DRK10020-SHOW:202607201950\r\n"
        elif raw == b"ATRCON\r\n": reply = self.current.to_bytes()
        elif raw == b"ATTBS\r\n": reply = f"Mast_Slove={self.master},ATLB=1,ATOCL=0".encode()
        else:
            desired = parse_settings(raw)
            if self.partial:
                self.partial = False
                return len(raw)-1
            if not self.mismatch: self.current = desired
            if self.drop: return len(raw)
            reply = measurement(self.current, fault=self.fault)
            if self.corrupt:
                reply = reply[:-1]+bytes([reply[-1] ^ 1])
            reply += self.extra
        self.buffer.extend(reply)
        return len(raw)


class ProtocolTests(unittest.TestCase):
    def test_known_control_bytes(self):
        # Captured old output-OFF command, independently fixed expected bytes.
        raw = bytes.fromhex("ccdd0100420000003f80000042dc000041b800007788c1")
        self.assertEqual(Settings(0, 32., 1., 110., 23.).to_bytes(), raw)
        self.assertEqual(parse_settings(raw).voltage, 32.)

    def test_every_split_and_concatenation(self):
        raw = measurement()
        for cut in range(len(raw)+1):
            decoder = StreamDecoder()
            packets = decoder.feed(raw[:cut])+decoder.feed(raw[cut:]+raw)
            self.assertEqual([p.raw for p in packets], [raw, raw])

    def test_bytewise_mixed_text_and_binary(self):
        decoder = StreamDecoder()
        raw = b"DRK10020\r\n"+Settings(0, 5., .1, 110., 23.).to_bytes()+measurement()
        packets = []
        for value in raw: packets.extend(decoder.feed(bytes([value])))
        self.assertEqual([p.kind for p in packets], ["text", "settings", "measurement"])

    def test_corruption_and_noise_are_visible(self):
        valid = measurement()
        invalid = valid[:-1]+bytes([valid[-1] ^ 1])
        packets = StreamDecoder().feed(b"\x01\x02"+invalid+valid)
        self.assertTrue(any(p.kind == "invalid" and p.raw == invalid for p in packets))
        self.assertEqual(packets[-1].raw, valid)
        self.assertTrue(any(p.kind == "noise" for p in packets))

    def test_idle_text_does_not_flush_partial_binary(self):
        decoder = StreamDecoder()
        decoder.feed(measurement()[:11])
        self.assertEqual(decoder.flush_idle_text(), [])
        self.assertEqual(len(decoder.buffer), 11)
        decoder = StreamDecoder()
        decoder.feed(b"Mast_Slove=0,ATLB=1")
        self.assertEqual(decoder.flush_idle_text()[0].value, "Mast_Slove=0,ATLB=1")

    def test_samples_scaling_and_power(self):
        values = list(range(22))+list(reversed(range(22)))+[1.]*22
        data = parse_measurement(measurement(samples=values))
        self.assertEqual(data["voltage_samples"], tuple(range(22)))
        self.assertEqual(data["voltage"], 10.5)
        self.assertEqual(data["power"], sum(i*(21-i) for i in range(22))/22)
        self.assertNotEqual(data["power"], data["voltage"]*data["current"])
        self.assertEqual((data["aux_voltage"], data["acdc_voltage"]), (12., 130.))

    def test_invalid_values_and_crc(self):
        for value in (math.nan, math.inf, -math.inf):
            raw = measurement(samples=[value]+[0.]*65)
            self.assertEqual(StreamDecoder().feed(raw)[0].kind, "invalid")
        with self.assertRaises(ProtocolError): parse_measurement(measurement()[:-1])
        with self.assertRaises(ProtocolError): parse_settings(b"x"*23)

    def test_parameter_bounds_and_float32(self):
        base = Settings(0, 5., .1, 110., 23.)
        for name, values in {"voltage": (-1, 101, math.nan, math.inf, True),
                             "current": (0, 21, math.nan, True),
                             "flags": (-1, 256, True)}.items():
            for value in values:
                with self.assertRaises(ValueError): base.update(**{name: value})
        self.assertEqual(base.update(voltage=0).voltage, 0.)
        self.assertEqual(base.update(current=.123456789),
                         parse_settings(base.update(current=.123456789).to_bytes()))

    def test_large_fifo_no_overwrite(self):
        raw = measurement()
        packets = StreamDecoder().feed(raw*10000)
        self.assertEqual(len(packets), 10000)
        self.assertTrue(all(p.raw == raw for p in packets))


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.serial = FakeSerial(self.clock)
        self.device = Connection("COM9", .2, _serial_factory=lambda **kw: self.serial,
                                 _clock=self.clock)

    def tearDown(self): self.device.close()

    def test_open_is_readonly_and_no_close_write(self):
        self.assertTrue(all(raw.startswith(b"AT") for raw in self.serial.writes))
        count = len(self.serial.writes)
        self.device.close()
        self.assertEqual(len(self.serial.writes), count)

    def test_factory_serial_parameters(self):
        clock = Clock()
        made = []
        def factory(**kwargs):
            made.append(FakeSerial(clock, **kwargs))
            return made[-1]
        device = Connection("COM9", .2, _serial_factory=factory, _clock=clock)
        self.addCleanup(device.close)
        self.assertEqual({k: made[0].kwargs[k] for k in ("baudrate", "bytesize", "parity", "stopbits")},
                         dict(baudrate=115200, bytesize=8, parity="N", stopbits=1))
        self.assertFalse(made[0].dtr or made[0].rts)

    def test_enable_disable_preserve_other_bits(self):
        self.serial.current = Settings(0x5c, 5., .1, 110., 23.)
        self.assertEqual(self.device.set_enabled(True)["flags"], 0x5f)
        self.assertEqual(self.device.set_enabled(False)["flags"], 0x5c)

    def test_disabled_exchange_does_not_enable(self):
        data = self.device.exchange(5., .1)
        self.assertFalse(data["enabled"])
        self.assertEqual(self.serial.current.flags, 0)

    def test_read_transmits_nothing(self):
        self.device.send(5., .1)
        count = len(self.serial.writes)
        data = self.device.read()
        self.assertEqual(len(self.serial.writes), count)
        self.assertEqual(len(data["raw"]), 288)
        self.assertEqual(len(data["voltage_samples"]), 22)

    def test_pending_owner_and_double_send(self):
        with self.assertRaises(RuntimeError): self.device.read()
        self.device.send(5., .1)
        with self.assertRaises(RuntimeError): self.device.send(6., .1)
        with self.assertRaises(RuntimeError): self.device.settings()
        failures = []
        def other_thread():
            try: self.device.read()
            except RuntimeError as exc: failures.append(str(exc))
        thread = threading.Thread(target=other_thread)
        thread.start(); thread.join()
        self.assertEqual(len(failures), 1)
        self.device.read()

    def test_fixed_20hz(self):
        self.device.exchange(5., .1)
        self.device.exchange(5., .1)
        self.device.exchange(5., .1)
        times = [t for raw, t in zip(self.serial.writes, self.serial.sent_times)
                 if raw.startswith(b"\xcc\xdd")]
        self.assertTrue(all(b-a >= .05-1e-10 for a,b in zip(times, times[1:])))

    def test_rate_uses_high_resolution_clock_and_rechecks_early_wakeup(self):
        self.clock.monotonic = lambda: (_ for _ in ()).throw(AssertionError("coarse clock used"))
        real_sleep = self.clock.sleep
        woke_early = []
        def early_once(delay):
            if not woke_early:
                woke_early.append(True)
                real_sleep(delay/2)
            else: real_sleep(delay)
        self.clock.sleep = early_once
        self.device.exchange(5., .1)
        self.device.exchange(5., .1)
        self.assertGreaterEqual(self.serial.sent_times[-1]-self.serial.sent_times[-2], .05-1e-10)

    def test_fault_only_delivered(self):
        self.serial.fault = 0x45
        before = len(self.serial.writes)
        data = self.device.exchange(5., .1)
        self.assertEqual(data["fault"], 0x45)
        self.assertEqual(len(self.serial.writes)-before, 1)
        self.device.exchange(5., .1)

    def test_lagged_status_is_delivered_without_inventing_confirmation(self):
        # Hardware can acknowledge new settings while still reporting an older
        # measurement. Do not synthesize status from the requested flags.
        self.serial.current = Settings(0, 5., .1, 110., 23.)
        self.device.send(5., .1)
        self.serial.buffer[:] = measurement(Settings(1, 32., 10., 110., 23.))
        sample = self.device.read()
        self.assertTrue(sample["enabled"])
        self.assertEqual(sample["voltage"], 32.)
        self.assertFalse(self.device.settings()["enabled"])

    def test_extra_frames_do_not_satisfy_next_request(self):
        extra = measurement(Settings(1, 99., .1, 110., 23.))
        self.serial.extra = extra
        first = self.device.exchange(5., .1)
        self.serial.extra = b""
        second = self.device.exchange(6., .1)
        self.assertEqual((first["voltage"], second["voltage"]), (0., 0.))
        events = self.device.drain_events()
        samples = [e for e in events if e["kind"] == "rx_measurement"]
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[1]["raw"], extra)

    def test_all_returned_bytes_and_event_sequence(self):
        self.serial.chunk = 3
        for _ in range(4): self.device.exchange(5., .1)
        events = self.device.drain_events()
        self.assertEqual(b"".join(e["raw"] for e in events if e["kind"] == "rx_chunk"),
                         bytes(self.serial.returned))
        self.assertEqual([e["sequence"] for e in events], list(range(1, len(events)+1)))
        self.assertEqual(self.device.stats()["pending_events"], 0)

    def test_partial_write_is_not_retried(self):
        self.serial.partial = True
        count = len(self.serial.writes)
        with self.assertRaises(WriteUncertainError): self.device.send(5., .1)
        self.assertEqual(len(self.serial.writes), count+1)
        with self.assertRaises(WriteUncertainError): self.device.send(5., .1)

    def test_timeout_locks_control_but_allows_explicit_off(self):
        self.serial.drop = True
        with self.assertRaises(WriteUncertainError): self.device.exchange(5., .1)
        with self.assertRaises(WriteUncertainError): self.device.set_enabled(True)
        self.assertEqual(self.device.settings()["voltage"], 5.)
        self.serial.drop = False
        self.assertFalse(self.device.set_enabled(False)["enabled"])
        self.assertFalse(self.device.stats()["uncertain"])

    def test_corrupt_feedback_retained_and_locks(self):
        self.serial.corrupt = True
        with self.assertRaises(WriteUncertainError): self.device.exchange(5., .1)
        self.assertTrue(any(e["kind"] == "rx_invalid" for e in self.device.drain_events()))

    def test_switch_mismatch_is_not_success(self):
        self.serial.mismatch = True
        with self.assertRaises(WriteUncertainError): self.device.set_enabled(True)
        self.assertTrue(self.device.stats()["uncertain"])

    def test_close_preserves_incomplete_and_pending(self):
        self.serial.drop = True
        self.device.send(5., .1)
        self.serial.buffer.extend(b"\xaa\xbb\x00")
        self.device.close()
        kinds = [e["kind"] for e in self.device.drain_events()]
        self.assertIn("incomplete", kinds)
        self.assertIn("pending_unconfirmed", kinds)

    def test_slave_control_rejected(self):
        self.device._master = False
        count = len(self.serial.writes)
        with self.assertRaises(ProtocolError): self.device.send(5., .1)
        self.assertEqual(len(self.serial.writes), count)

    def test_wrong_model_closed_and_evidence_delivered(self):
        clock = Clock(); serial = FakeSerial(clock); serial.model = "DRK10020S"
        with self.assertRaises(ProtocolError) as caught:
            Connection("COM9", .2, _serial_factory=lambda **kw: serial, _clock=clock)
        self.assertFalse(serial.is_open)
        self.assertTrue(caught.exception.events)
        self.assertTrue(all(raw.startswith(b"AT") for raw in serial.writes))


class ModuleTests(unittest.TestCase):
    def tearDown(self):
        drk10020_sdk.close(); drk10020_sdk.drain_events()

    def connect_factory(self, name, timeout):
        clock = Clock(); serial = FakeSerial(clock)
        return Connection(name, timeout, _serial_factory=lambda **kw: serial, _clock=clock)

    def test_auto_filter_and_close_tail(self):
        ports = [SimpleNamespace(device="COM9",vid=0x1a86,pid=0x7523),
                 SimpleNamespace(device="COM4",vid=0x451,pid=0xbef3)]
        with patch("serial.tools.list_ports.comports", return_value=ports), \
             patch("drk10020_sdk.Connection", side_effect=self.connect_factory) as maker:
            self.assertEqual(drk10020_sdk.open()["port"], "COM9")
            maker.assert_called_once_with("COM9", 1.)
            drk10020_sdk.exchange(5., .1)
            drk10020_sdk.close()
            self.assertTrue(drk10020_sdk.drain_events())
            self.assertEqual(drk10020_sdk.drain_events(), [])

    def test_multiple_devices_not_chosen_arbitrarily(self):
        ports = [SimpleNamespace(device=name,vid=0x1a86,pid=0x7523) for name in ("COM9","COM10")]
        made = []
        def make(name, timeout):
            device = self.connect_factory(name, timeout); made.append(device); return device
        with patch("serial.tools.list_ports.comports", return_value=ports), \
             patch("drk10020_sdk.Connection", side_effect=make):
            with self.assertRaises(RuntimeError): drk10020_sdk.open()
        self.assertTrue(all(d.closed for d in made))
        self.assertTrue(drk10020_sdk.drain_events())

    def test_no_device_no_serial_open(self):
        with patch("serial.tools.list_ports.comports", return_value=[]), \
             patch("drk10020_sdk.Connection") as maker:
            with self.assertRaises(RuntimeError): drk10020_sdk.open()
            maker.assert_not_called()

    def test_closed_evidence_survives_reopen(self):
        with patch("drk10020_sdk.Connection", side_effect=self.connect_factory):
            drk10020_sdk.open("COM9"); drk10020_sdk.close()
            drk10020_sdk.open("COM9"); drk10020_sdk.close()
        events = drk10020_sdk.drain_events()
        self.assertEqual(len({e["connection_id"] for e in events}), 2)


if __name__ == "__main__": unittest.main()
