#!/usr/bin/env python3

import argparse
import datetime
import json
import math
import queue
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox

import numpy as np
import serial
import serial.tools.list_ports
import tkintermapview


DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200

METERS_TO_FEET = 3.28084

MIN_ABS_COORDINATE = 1.0
MAP_SAMPLE_SECONDS = 1.0
MAP_MIN_DISTANCE_METERS = 2.0
MAX_TRACK_MARKERS = 2000

STREET_TILES = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Street_Map/MapServer/tile/{z}/{y}/{x}"
)
SATELLITE_TILES = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Imagery/MapServer/tile/{z}/{y}/{x}"
)


MAGIC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
PACKET_FAST = 0x01
PACKET_STATUS = 0x02
PACKET_ORIENTATION = 0x03

PACKET_ACK = 0x05
PACKET_ALERT = 0x06

PACKET_NAMES = {
    PACKET_FAST: "fast",
    PACKET_STATUS: "status",
    PACKET_ORIENTATION: "orientation",
    PACKET_ACK: "ack",
    PACKET_ALERT: "alert",
}
PACKET_LENGTHS = {
    PACKET_FAST: 40,
    PACKET_STATUS: 32,
    PACKET_ORIENTATION: 22,
    PACKET_ACK: 12,
    PACKET_ALERT: 46,
}

ALERT_SEVERITY_NAMES = {0: "INFO", 1: "WARNING", 2: "CRITICAL"}
ACK_RESULT_NAMES = {0: "applied", 1: "refused", 2: "ignored", 3: "unknown command"}

STATUS_RECORDING = 1 << 0
STATUS_5V_ACTIVE = 1 << 1
STATUS_GPS_FIX = 1 << 2
STATUS_GPS_OK = 1 << 3
STATUS_BARO_OK = 1 << 4
STATUS_IMU_OK = 1 << 5
STATUS_FLASH_OK = 1 << 6
STATUS_LORA_OK = 1 << 7


def flag(status, bit):
    return bool(status & bit)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc



PACKET_COMMAND = 0x04
COMMAND_PACKET_SIZE = 14

CMD_CLEAR_FLASH = 0x01
CMD_START_RECORDING = 0x02
CMD_STOP_RECORDING = 0x03
CMD_CALIBRATE_IMU = 0x04
CMD_CALIBRATE_BAROMETER = 0x05
CMD_5V_ON = 0x07
CMD_5V_OFF = 0x08
CMD_BUZZER_MUTE = 0x09
CMD_BUZZER_UNMUTE = 0x0A
CMD_DVR_TRIGGER = 0x0B

COMMAND_NAMES = {
    CMD_CLEAR_FLASH: "Clear flash",
    CMD_START_RECORDING: "Start recording",
    CMD_STOP_RECORDING: "Stop recording",
    CMD_CALIBRATE_IMU: "Calibrate IMU",
    CMD_CALIBRATE_BAROMETER: "Calibrate barometer",
    CMD_5V_ON: "5V ON",
    CMD_5V_OFF: "5V OFF",
    CMD_BUZZER_MUTE: "Buzzer mute",
    CMD_BUZZER_UNMUTE: "Buzzer unmute",
    CMD_DVR_TRIGGER: "DVR trigger (debug)",
}


def build_command_packet(sequence, command_id, param=0):
    buffer = bytearray(COMMAND_PACKET_SIZE)
    buffer[0:2] = MAGIC
    buffer[2] = PROTOCOL_VERSION
    buffer[3] = PACKET_COMMAND
    buffer[4:6] = sequence.to_bytes(2, "little")
    buffer[6] = command_id
    buffer[7] = 0
    buffer[8:12] = (param & 0xFFFFFFFF).to_bytes(4, "little")

    crc = crc16(buffer[:12])
    buffer[12:14] = crc.to_bytes(2, "little")

    return bytes(buffer)


def common_fields(packet):
    return {
        "sequence": int.from_bytes(packet[4:6], "little"),
        "timestamp": int.from_bytes(packet[6:10], "little"),
    }


def parse_fast(packet):
    data = common_fields(packet)
    data.update({
        "latitude": int.from_bytes(packet[10:14], "little", signed=True) / 1_000_000.0,
        "longitude": int.from_bytes(packet[14:18], "little", signed=True) / 1_000_000.0,
        "altitude_m": int.from_bytes(packet[18:20], "little", signed=True) / 100.0,
        "accel_x_g": int.from_bytes(packet[20:22], "little", signed=True) / 1000.0,
        "accel_y_g": int.from_bytes(packet[22:24], "little", signed=True) / 1000.0,
        "accel_z_g": int.from_bytes(packet[24:26], "little", signed=True) / 1000.0,
        "gyro_x_dps": int.from_bytes(packet[26:28], "little", signed=True) / 100.0,
        "gyro_y_dps": int.from_bytes(packet[28:30], "little", signed=True) / 100.0,
        "gyro_z_dps": int.from_bytes(packet[30:32], "little", signed=True) / 100.0,
        "pressure_pa": int.from_bytes(packet[32:36], "little"),
        "status": packet[36],
        "gps_satellites": packet[37],
    })
    return data


def parse_status(packet):
    data = common_fields(packet)
    data.update({
        "battery_mv": int.from_bytes(packet[10:12], "little"),
        "flash_percent": int.from_bytes(packet[12:14], "little"),
        "lora_rssi": int.from_bytes(packet[14:15], "little", signed=True),
        "lora_snr": int.from_bytes(packet[15:16], "little", signed=True),
        "gps_satellites": packet[16],
        "status": packet[17],
        "flash_bytes_written": int.from_bytes(packet[18:22], "little"),
        "flash_errors": int.from_bytes(packet[22:26], "little"),
        "gps_unix_time": int.from_bytes(packet[26:30], "little"),
    })
    return data


def parse_orientation(packet):
    data = common_fields(packet)
    calibration = packet[18]
    data.update({
        "quat_w": int.from_bytes(packet[10:12], "little", signed=True) / 10000.0,
        "quat_x": int.from_bytes(packet[12:14], "little", signed=True) / 10000.0,
        "quat_y": int.from_bytes(packet[14:16], "little", signed=True) / 10000.0,
        "quat_z": int.from_bytes(packet[16:18], "little", signed=True) / 10000.0,
        "filter_converged": ((calibration >> 6) & 3) >= 3,
    })
    return data


def parse_ack(packet):
    return {
        "sequence": int.from_bytes(packet[4:6], "little"),
        "acked_sequence": int.from_bytes(packet[6:8], "little"),
        "acked_command": packet[8],
        "result": packet[9],
    }


def parse_alert(packet):
    data = common_fields(packet)
    message = packet[12:12 + 32].split(b"\x00", 1)[0].decode("ascii", errors="replace")
    data.update({
        "code": packet[10],
        "severity": packet[11],
        "message": message,
    })
    return data


def decode_packet(packet):
    if len(packet) < 4:
        return None, None, "short"
    if packet[:2] != MAGIC:
        return None, None, "bad_magic"
    if packet[2] != PROTOCOL_VERSION:
        return None, None, "bad_version"

    packet_type = packet[3]
    kind = PACKET_NAMES.get(packet_type)
    if kind is None:
        return None, None, "unknown_type"

    if len(packet) != PACKET_LENGTHS[packet_type]:
        return kind, None, "bad_length"

    if crc16(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
        return kind, None, "bad_crc"

    if packet_type == PACKET_FAST:
        return kind, parse_fast(packet), None
    if packet_type == PACKET_STATUS:
        return kind, parse_status(packet), None
    if packet_type == PACKET_ORIENTATION:
        return kind, parse_orientation(packet), None
    if packet_type == PACKET_ACK:
        return kind, parse_ack(packet), None
    return kind, parse_alert(packet), None



@dataclass
class PacketStats:
    good_total: int = 0
    good_fast: int = 0
    good_status: int = 0
    good_orientation: int = 0
    bad_total: int = 0
    bad_crc: int = 0
    bad_length: int = 0
    bad_hex: int = 0
    bad_magic: int = 0
    bad_version: int = 0
    unknown_type: int = 0
    short: int = 0
    serial_errors: int = 0
    sequence_gaps: int = 0
    sequence_resets: int = 0
    last_error: str = ""

    def good(self, kind):
        self.good_total += 1
        if kind == "fast":
            self.good_fast += 1
        elif kind == "status":
            self.good_status += 1
        elif kind == "orientation":
            self.good_orientation += 1

    def bad(self, reason):
        self.bad_total += 1
        if reason == "bad_crc":
            self.bad_crc += 1
        elif reason == "bad_length":
            self.bad_length += 1
        elif reason == "bad_hex":
            self.bad_hex += 1
        elif reason == "bad_magic":
            self.bad_magic += 1
        elif reason == "bad_version":
            self.bad_version += 1
        elif reason == "unknown_type":
            self.unknown_type += 1
        elif reason == "short":
            self.short += 1
        self.last_error = reason



COMMAND_RETRY_INTERVAL_S = 0.4
MAX_COMMAND_ATTEMPTS = 8


class SerialReader(threading.Thread):
    def __init__(self, port, baud, events):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self.stop_event = threading.Event()
        self.serial_port = None
        self.write_queue = queue.Queue()

        self.pending_lock = threading.Lock()
        self.pending_commands = {}
        self._next_sequence = 0

    def run(self):
        try:
            self.serial_port = serial.Serial(
                self.port, self.baud, timeout=0.2, write_timeout=1.0
            )
            self.events.put(("connected", self.port))
        except serial.SerialException as exc:
            self.events.put(("reader_error", str(exc)))
            return

        buffer = bytearray()
        try:
            while not self.stop_event.is_set():
                chunk = self.serial_port.read(max(1, self.serial_port.in_waiting))
                if chunk:
                    buffer.extend(chunk)

                    while b"\n" in buffer:
                        line, _, rest = buffer.partition(b"\n")
                        buffer = bytearray(rest)
                        try:
                            self._handle_line(line)
                        except Exception as exc:
                            self.events.put(("bad", None, "bad_hex"))
                            self.events.put(("reader_error", f"line handling error: {exc}"))

                self._flush_write_queue()
                self._service_retries()
        finally:
            if self.serial_port is not None:
                self.serial_port.close()

    def _flush_write_queue(self):
        while True:
            try:
                data = self.write_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self.serial_port.write(data)
            except serial.SerialException:
                return

    def _service_retries(self):
        now = time.monotonic()
        expired = []
        with self.pending_lock:
            for sequence in list(self.pending_commands.keys()):
                pending = self.pending_commands[sequence]
                if now - pending["last_sent"] < COMMAND_RETRY_INTERVAL_S:
                    continue
                if pending["attempts"] >= MAX_COMMAND_ATTEMPTS:
                    expired.append((pending["command_id"], pending["param"]))
                    del self.pending_commands[sequence]
                    continue
                pending["attempts"] += 1
                pending["last_sent"] = now
                self.write_queue.put(pending["line"])

        for command_id, param in expired:
            self.events.put(("command_timeout", command_id, param))

    def send_command(self, command_id, param=0):
        if self.serial_port is None:
            return False

        with self.pending_lock:
            sequence = self._next_sequence
            self._next_sequence = (self._next_sequence + 1) & 0xFFFF

            packet = build_command_packet(sequence, command_id, param)
            line = ("TX:" + packet.hex().upper() + "\n").encode("ascii")

            self.pending_commands[sequence] = {
                "command_id": command_id,
                "param": param,
                "line": line,
                "attempts": 1,
                "last_sent": time.monotonic(),
            }

        self.write_queue.put(line)
        return True

    def _handle_line(self, line):
        text = line.decode("ascii", errors="ignore").strip()
        if not text:
            return
        try:
            packet = bytes.fromhex(text)
        except ValueError:
            self.events.put(("bad", None, "bad_hex"))
            return

        kind, data, error = decode_packet(packet)
        if error:
            self.events.put(("bad", kind, error))
        elif kind == "ack":
            self._handle_ack(data)
        elif kind == "alert":
            self.events.put(("alert", data["code"], data["severity"], data["message"]))
        else:
            self.events.put(("packet", kind, data))

    def _handle_ack(self, data):
        with self.pending_lock:
            pending = self.pending_commands.pop(data["acked_sequence"], None)
        if pending is not None:
            self.events.put(("command_ack", pending["command_id"], data["result"]))

    def stop(self):
        self.stop_event.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass



class MapWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.title("GPS Map")
        self.geometry("900x700")
        self.configure(bg="#202124")

        controls = tk.Frame(self, bg="#202124")
        controls.pack(fill="x", padx=8, pady=6)
        self.layer_button = tk.Button(
            controls, text="Satellite view", command=self.toggle_layer,
            bg="#3c4043", fg="white", activebackground="#5f6368"
        )
        self.layer_button.pack(side="left")
        tk.Button(
            controls, text="Clear track", command=self.clear_track,
            bg="#3c4043", fg="white", activebackground="#5f6368"
        ).pack(side="left", padx=6)
        self.layer_label = tk.Label(
            controls, text="Street map", bg="#202124", fg="#dfe1e5"
        )
        self.layer_label.pack(side="left", padx=8)

        self.hover_label = tk.Label(
            self, text="Hover over a track point for coordinates",
            anchor="w", bg="#202124", fg="#dfe1e5"
        )
        self.hover_label.pack(fill="x", padx=8, pady=(0, 5))

        self.map_widget = tkintermapview.TkinterMapView(
            self, width=900, height=650, corner_radius=0
        )
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server(STREET_TILES)
        self.map_widget.set_zoom(15)
        self.map_widget.bind("<Motion>", self.on_mouse_move)

        self.street_mode = True
        self.live_marker = None
        self.path = None
        self.points = []
        self.track_markers = []
        self.last_sample_time = None
        self.last_sample_position = None
        self.centered = False

    @staticmethod
    def valid_coordinate(data):
        lat = data["latitude"]
        lon = data["longitude"]
        return (
            flag(data["status"], STATUS_GPS_FIX)
            and math.isfinite(lat)
            and math.isfinite(lon)
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
            and abs(lat) >= MIN_ABS_COORDINATE
            and abs(lon) >= MIN_ABS_COORDINATE
        )

    def toggle_layer(self):
        try:
            if self.street_mode:
                self.map_widget.set_tile_server(SATELLITE_TILES)
                self.layer_button.config(text="Street view")
                self.layer_label.config(text="Satellite imagery")
            else:
                self.map_widget.set_tile_server(STREET_TILES)
                self.layer_button.config(text="Satellite view")
                self.layer_label.config(text="Street map")
            self.street_mode = not self.street_mode
        except Exception as exc:
            self.hover_label.config(text=f"Map layer error: {exc}")

    def clear_track(self):
        for marker in self.track_markers:
            try:
                marker.delete()
            except Exception:
                pass
        self.track_markers.clear()
        self.points.clear()
        self.last_sample_time = None
        self.last_sample_position = None
        if self.path is not None:
            try:
                self.path.delete()
            except Exception:
                pass
        self.path = None
        self.hover_label.config(text="Track cleared")

    def update_position(self, data):
        if not self.valid_coordinate(data):
            return

        lat = data["latitude"]
        lon = data["longitude"]
        marker_text = (
            f"Lat: {lat:.7f}\nLon: {lon:.7f}\n"
            f"Alt: {data['altitude_m'] * METERS_TO_FEET:.1f} ft\nSeq: {data['sequence']}"
        )

        if not self.centered:
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(17)
            self.centered = True

        if self.live_marker is None:
            self.live_marker = self.map_widget.set_marker(lat, lon, text="Rocket")
        else:
            self.live_marker.set_position(lat, lon)

        now = time.monotonic()
        sample = self.last_sample_time is None
        if not sample:
            elapsed = now - self.last_sample_time
            distance = self.distance_meters(
                self.last_sample_position[0], self.last_sample_position[1], lat, lon
            )
            sample = elapsed >= MAP_SAMPLE_SECONDS and distance >= MAP_MIN_DISTANCE_METERS

        if not sample:
            return

        self.last_sample_time = now
        self.last_sample_position = (lat, lon)
        point = {
            "lat": lat,
            "lon": lon,
            "altitude_m": data["altitude_m"],
            "timestamp": data["timestamp"],
            "sequence": data["sequence"],
        }
        self.points.append(point)
        self.add_point_marker(point, marker_text)

        positions = [(p["lat"], p["lon"]) for p in self.points]
        if len(positions) < 2:
            return

        try:
            if self.path is None:
                self.path = self.map_widget.set_path(positions)
            else:
                self.path.add_position(lat, lon)
        except (AttributeError, tk.TclError, ValueError):
            if self.path is not None:
                try:
                    self.path.delete()
                except Exception:
                    pass
            self.path = None

    def add_point_marker(self, point, text):
        try:
            marker = self.map_widget.set_marker(
                point["lat"], point["lon"], text=text,
                command=lambda _marker, p=point: self.show_point(p)
            )
        except TypeError:
            marker = self.map_widget.set_marker(point["lat"], point["lon"], text=text)
        self.track_markers.append(marker)

        while len(self.track_markers) > MAX_TRACK_MARKERS:
            old = self.track_markers.pop(0)
            try:
                old.delete()
            except Exception:
                pass

    def show_point(self, point):
        self.hover_label.config(
            text=(
                f"Lat: {point['lat']:.7f}    Lon: {point['lon']:.7f}    "
                f"Alt: {point['altitude_m'] * METERS_TO_FEET:.1f} ft    "
                f"Timestamp: {point['timestamp']} ms"
            )
        )

    @staticmethod
    def distance_meters(lat1, lon1, lat2, lon2):
        radius = 6_371_000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return radius * 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

    def on_mouse_move(self, event):
        if not self.points:
            return
        try:
            nearest = None
            nearest_distance = float("inf")
            for point in self.points:
                x, y = self.map_widget.convert_decimal_coords_to_canvas_coords(
                    point["lat"], point["lon"]
                )
                distance = math.hypot(x - event.x, y - event.y)
                if distance < nearest_distance:
                    nearest, nearest_distance = point, distance
            if nearest is not None and nearest_distance <= 18:
                self.show_point(nearest)
            elif nearest_distance > 25:
                self.hover_label.config(text="Hover over a track point for coordinates")
        except (AttributeError, TypeError, ValueError):
            pass



class OrientationWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.title("Orientation")
        self.geometry("600x650")
        self.configure(bg="#141414")
        self.canvas = tk.Canvas(
            self, width=600, height=560, bg="#141414", highlightthickness=0
        )
        self.canvas.pack(fill="both", expand=True)
        self.readout = tk.Label(
            self, text="Waiting for orientation packets...", anchor="w", justify="left",
            fg="#abb2bf", bg="#141414", font=("Consolas", 11)
        )
        self.readout.pack(fill="x", padx=12, pady=8)

    @staticmethod
    def rotation_matrix(w, x, y, z):
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-8:
            return np.eye(3)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    @staticmethod
    def euler(w, x, y, z):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def render(self, data):
        self.canvas.delete("all")
        cx, cy, scale = 300, 270, 100
        rotation = self.rotation_matrix(
            data["quat_w"], data["quat_x"], data["quat_y"], data["quat_z"]
        )

        def project(point):
            p = rotation @ np.array(point, dtype=float)
            return cx + p[0] * scale, cy - p[1] * scale

        body = [
            project((-0.35, -0.35, -1.4)), project((0.35, -0.35, -1.4)),
            project((0.35, 0.35, -1.4)), project((-0.35, 0.35, -1.4)),
            project((-0.24, -0.24, 1.2)), project((0.24, -0.24, 1.2)),
            project((0.24, 0.24, 1.2)), project((-0.24, 0.24, 1.2)),
        ]
        self.canvas.create_polygon(
            [v for p in body[:4] for v in p], fill="#33445c", outline="#c8d6ec"
        )
        self.canvas.create_polygon(
            [v for p in body[4:] for v in p], fill="#8fb8e8", outline="#c8d6ec"
        )
        for i in range(4):
            self.canvas.create_line(*body[i], *body[i + 4], fill="#c8d6ec")

        nose_base = [body[4], body[5], body[6], body[7]]
        nose = project((0, 0, 1.95))
        for point in nose_base:
            self.canvas.create_line(*point, *nose, fill="#e06c75", width=2)

        roll, pitch, yaw = self.euler(
            data["quat_w"], data["quat_x"], data["quat_y"], data["quat_z"]
        )
        self.readout.config(
            text=(
                f"Roll: {math.degrees(roll):6.1f} deg    "
                f"Pitch: {math.degrees(pitch):6.1f} deg    "
                f"Yaw: {math.degrees(yaw):6.1f} deg (relative - no magnetometer, drifts freely)\n"
                f"Orientation filter: "
                f"{'converged' if data['filter_converged'] else 'still settling'}"
            )
        )



class ControlWindow(tk.Toplevel):
    def __init__(self, master, reader):
        super().__init__(master)
        self.reader = reader
        self.title("Rocket Control")
        self.geometry("440x600")
        self.configure(bg="#282c34")

        self.recording = False

        tk.Label(
            self,
            text=(
                "Commands are sent over the LoRa uplink, best-effort\n"
                "(no ACK from the rocket) - confirm on its own NeoPixels/\n"
                "buzzer/status flags that a command actually landed."
            ),
            bg="#282c34", fg="#e5c07b", justify="left", anchor="w",
        ).pack(fill="x", padx=12, pady=(12, 6))

        self._build_power_section()
        self._build_recording_section()
        self._build_flash_section()
        self._build_calibration_section()
        self._build_buzzer_section()
        self._build_debug_section()

        tk.Label(self, text="Action log", bg="#282c34", fg="#abb2bf", anchor="w").pack(
            fill="x", padx=12, pady=(6, 0)
        )
        self.log = tk.Text(
            self, height=8, bg="#1e2127", fg="#abb2bf", insertbackground="#abb2bf",
            borderwidth=0, highlightthickness=0, state="disabled",
        )
        self.log.pack(fill="both", expand=True, padx=12, pady=(4, 12))

    @staticmethod
    def _button(parent, text, command):
        return tk.Button(
            parent, text=text, command=command,
            bg="#3c4043", fg="white", activebackground="#5f6368",
            activeforeground="white", borderwidth=0, padx=10, pady=4,
        )

    @staticmethod
    def _section(parent, title):
        frame = tk.LabelFrame(
            parent, text=title, bg="#282c34", fg="#abb2bf",
            labelanchor="nw", borderwidth=1, relief="groove",
        )
        frame.pack(fill="x", padx=12, pady=6)
        return frame

    def _log_action(self, message):
        self.log.config(state="normal")
        self.log.insert("end", f"{message}\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _send(self, command_id, param=0):
        if self.reader.send_command(command_id, param):
            return True
        self._log_action("NOT SENT - serial port not connected.")
        return False


    def _build_power_section(self):
        frame = self._section(self, "Sensor Board 5V Power")

        self._button(frame, "5V ON", self.power_5v_on).pack(
            side="left", padx=8, pady=8
        )
        self._button(frame, "5V OFF", self.power_5v_off).pack(
            side="left", padx=8, pady=8
        )

    def power_5v_on(self):
        if self._send(CMD_5V_ON):
            self._log_action("Sent: 5V ON.")

    def power_5v_off(self):
        if self._send(CMD_5V_OFF):
            self._log_action("Sent: 5V OFF.")


    def _build_buzzer_section(self):
        frame = self._section(self, "Buzzer")

        self._button(frame, "Mute", self.buzzer_mute).pack(
            side="left", padx=8, pady=8
        )
        self._button(frame, "Unmute", self.buzzer_unmute).pack(
            side="left", padx=8, pady=8
        )

    def buzzer_mute(self):
        if self._send(CMD_BUZZER_MUTE):
            self._log_action("Sent: buzzer mute.")

    def buzzer_unmute(self):
        if self._send(CMD_BUZZER_UNMUTE):
            self._log_action("Sent: buzzer unmute.")


    def _build_recording_section(self):
        frame = self._section(self, "Recording")

        self.record_button = self._button(
            frame, "Start Recording", self.toggle_recording
        )
        self.record_button.pack(side="left", padx=8, pady=8)

        self.record_status = tk.Label(
            frame, text="Stopped", bg="#282c34", fg="#e06c75"
        )
        self.record_status.pack(side="left", padx=8)

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            self.record_button.config(text="Stop Recording")
            self.record_status.config(text="Recording", fg="#98c379")
            if self._send(CMD_START_RECORDING):
                self._log_action("Sent: start recording.")
        else:
            self.record_button.config(text="Start Recording")
            self.record_status.config(text="Stopped", fg="#e06c75")
            if self._send(CMD_STOP_RECORDING):
                self._log_action("Sent: stop recording.")


    def _build_flash_section(self):
        frame = self._section(self, "Flash Storage")
        self._button(frame, "Clear Flash", self.clear_flash).pack(
            side="left", padx=8, pady=8
        )

    def clear_flash(self):
        if not messagebox.askyesno(
            "Clear Flash",
            "Clear all recorded flight data on the rocket?\n\n"
            "This is sent over the LoRa uplink, best-effort (no ACK) - "
            "confirm on the rocket's own feedback that it landed.",
            parent=self,
        ):
            return
        if self._send(CMD_CLEAR_FLASH):
            self._log_action("Sent: clear flash.")


    def _build_calibration_section(self):
        frame = self._section(self, "Calibration")
        self._button(frame, "Calibrate IMU (upright)", self.calibrate_imu).pack(
            side="left", padx=8, pady=8
        )
        self._button(
            frame, "Calibrate Barometer (zero)", self.calibrate_barometer
        ).pack(side="left", padx=8, pady=8)

    def calibrate_imu(self):
        if not messagebox.askyesno(
            "Calibrate IMU",
            "Hold the rocket in its upright/nose-up launch orientation, "
            "then confirm - the rocket will treat however it's sitting "
            "right now as \"level\" from here on.",
            parent=self,
        ):
            return
        if self._send(CMD_CALIBRATE_IMU):
            self._log_action("Sent: calibrate IMU (upright zero).")

    def calibrate_barometer(self):
        if self._send(CMD_CALIBRATE_BAROMETER):
            self._log_action("Sent: calibrate barometer (zero altitude).")


    def _build_debug_section(self):
        frame = self._section(self, "Debug")
        self._button(frame, "Trigger DVR", self.debug_trigger_dvr).pack(
            side="left", padx=8, pady=8
        )

    def debug_trigger_dvr(self):
        if self._send(CMD_DVR_TRIGGER):
            self._log_action("Sent: DVR trigger (debug).")



class RawDataWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Rocket Telemetry - Raw Data")
        self.geometry("920x480")
        self.configure(bg="#282c34")

        self.recording_banner = tk.Label(
            self, text="NOT RECORDING", bg="#e06c75", fg="#1e2127",
            font=("Consolas", 14, "bold"), pady=6,
        )
        self.recording_banner.pack(fill="x", side="top")
        self._recording = False
        self._banner_flash_on = True

        columns = tk.Frame(self, bg="#282c34")
        columns.pack(fill="both", expand=True)
        columns.columnconfigure(0, weight=1)
        columns.columnconfigure(1, weight=1)
        columns.rowconfigure(0, weight=1)

        self.text_left = self._make_text_column(columns)
        self.text_left.grid(row=0, column=0, sticky="nsew")
        self.text_right = self._make_text_column(columns)
        self.text_right.grid(row=0, column=1, sticky="nsew")

        self._flash_recording_banner()

    @staticmethod
    def _make_text_column(parent):
        text = tk.Text(
            parent, bg="#282c34", fg="#abb2bf", insertbackground="#abb2bf",
            font=("Consolas", 11), borderwidth=0, highlightthickness=0,
            padx=14, pady=12, wrap="none",
        )
        text.tag_configure("head", foreground="#e5c07b", font=("Consolas", 11, "bold"))
        text.tag_configure("key", foreground="#61afef")
        text.tag_configure("val", foreground="#98c379")
        text.tag_configure("bad", foreground="#e06c75")
        text.tag_configure("unit", foreground="#5c6370")
        text.config(state="disabled")
        return text

    def set_recording(self, recording):
        self._recording = recording

    def _flash_recording_banner(self):
        if self._recording:
            self.recording_banner.config(text="RECORDING", bg="#2d3b2d", fg="#98c379")
        else:
            self._banner_flash_on = not self._banner_flash_on
            if self._banner_flash_on:
                self.recording_banner.config(text="NOT RECORDING", bg="#e06c75", fg="#1e2127")
            else:
                self.recording_banner.config(text="NOT RECORDING", bg="#1e2127", fg="#e06c75")
        self.after(500, self._flash_recording_banner)

    def row(self, text_widget, label, value, unit=""):
        text_widget.insert("end", f" {label:<22}", "key")
        text_widget.insert("end", str(value), "val")
        if unit:
            text_widget.insert("end", f" {unit}", "unit")
        text_widget.insert("end", "\n")

    def flag_row(self, text_widget, label, value):
        text_widget.insert("end", f" {label:<22}", "key")
        text_widget.insert("end", "YES\n" if value else "NO\n", "val" if value else "bad")

    def update_view(self, fast, status_packet, stats, connection):
        left = self.text_left
        right = self.text_right

        left.config(state="normal")
        right.config(state="normal")
        left.delete("1.0", "end")
        right.delete("1.0", "end")

        left.insert("end", "=== PACKET HEALTH ===\n", "head")
        self.row(left, "Good packets", stats.good_total)
        self.row(left, "  Fast good", stats.good_fast)
        self.row(left, "  Status good", stats.good_status)
        self.row(left, "  Orientation good", stats.good_orientation)
        self.row(left, "Bad packets", stats.bad_total)
        self.row(left, "  Bad CRC", stats.bad_crc)
        self.row(left, "  Bad length", stats.bad_length)
        self.row(left, "  Bad hex/text", stats.bad_hex)
        self.row(left, "  Bad magic", stats.bad_magic)
        self.row(left, "  Bad version", stats.bad_version)
        self.row(left, "  Unknown type", stats.unknown_type)
        self.row(left, "  Too short", stats.short)
        self.row(left, "Sequence gaps", stats.sequence_gaps)
        self.row(left, "Sequence resets", stats.sequence_resets)
        self.row(left, "Serial connection", connection)
        if stats.last_error:
            self.row(left, "Last rejected", stats.last_error)

        left.insert("end", "\n=== FAST PACKET ===\n", "head")
        if fast:
            self.row(left, "Sequence", fast["sequence"])
            self.row(left, "Timestamp", fast["timestamp"], "ms")
            self.row(left, "Latitude", f'{fast["latitude"]:.7f}', "deg")
            self.row(left, "Longitude", f'{fast["longitude"]:.7f}', "deg")
            self.row(left, "Altitude", f'{fast["altitude_m"] * METERS_TO_FEET:.1f}', "ft")
            self.row(left, "Accel X/Y/Z", f'{fast["accel_x_g"]:.3f} / {fast["accel_y_g"]:.3f} / {fast["accel_z_g"]:.3f}', "g")
            self.row(left, "Gyro X/Y/Z", f'{fast["gyro_x_dps"]:.2f} / {fast["gyro_y_dps"]:.2f} / {fast["gyro_z_dps"]:.2f}', "deg/s")
            self.row(left, "Pressure", fast["pressure_pa"], "Pa")
            self.row(left, "GPS satellites", fast["gps_satellites"])
        else:
            left.insert("end", " waiting for data...\n", "unit")

        right.insert("end", "=== STATUS FLAGS ===\n", "head")
        status = fast["status"] if fast else (status_packet["status"] if status_packet else 0)
        self.flag_row(right, "Recording", flag(status, STATUS_RECORDING))
        self.flag_row(right, "5V active", flag(status, STATUS_5V_ACTIVE))
        self.flag_row(right, "GPS fix", flag(status, STATUS_GPS_FIX))
        self.flag_row(right, "GPS OK", flag(status, STATUS_GPS_OK))
        self.flag_row(right, "Barometer OK", flag(status, STATUS_BARO_OK))
        self.flag_row(right, "IMU OK", flag(status, STATUS_IMU_OK))
        self.flag_row(right, "Flash OK", flag(status, STATUS_FLASH_OK))
        self.flag_row(right, "LoRa OK", flag(status, STATUS_LORA_OK))

        right.insert("end", "\n=== STATUS PACKET ===\n", "head")
        if status_packet:
            self.row(right, "Battery", f'{status_packet["battery_mv"] / 1000:.3f}', "V")
            self.row(right, "Flash used", status_packet["flash_percent"], "%")
            self.row(right, "Flash written", status_packet["flash_bytes_written"], "bytes")
            self.row(right, "Flash errors", status_packet["flash_errors"])
            self.row(right, "LoRa RSSI", status_packet["lora_rssi"], "dBm")
            self.row(right, "LoRa SNR", status_packet["lora_snr"], "dB")
            gps_unix_time = status_packet["gps_unix_time"]
            if gps_unix_time:
                gps_time_text = datetime.datetime.fromtimestamp(
                    gps_unix_time, tz=datetime.timezone.utc
                ).strftime("%Y-%m-%d %H:%M:%S UTC")
            else:
                gps_time_text = "no GPS time fix yet"
            self.row(right, "GPS time", gps_time_text)
        else:
            right.insert("end", " waiting for data...\n", "unit")

        left.config(state="disabled")
        right.config(state="disabled")



WINDOW_STATE_FILE = Path.home() / ".rocket_dashboard_window_state.json"


def load_window_state():
    try:
        with open(WINDOW_STATE_FILE, "r") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_window_state(state):
    try:
        with open(WINDOW_STATE_FILE, "w") as f:
            json.dump(state, f, indent=2)
    except OSError:
        pass



class App:
    def __init__(self, port, baud):
        self.baud = baud
        self.raw_window = RawDataWindow()
        self.map_window = MapWindow(self.raw_window)
        self.orientation_window = OrientationWindow(self.raw_window)
        self.events = queue.Queue()
        self.reader = SerialReader(port, baud, self.events)
        self.control_window = ControlWindow(self.raw_window, self.reader)
        self.reader.start()

        self.windows = {
            "raw": self.raw_window,
            "map": self.map_window,
            "orientation": self.orientation_window,
            "control": self.control_window,
        }

        self.stats = PacketStats()
        self.latest_fast = None
        self.latest_status = None
        self.latest_orientation = None
        self.last_sequences = {}
        self.connection = "Opening serial port..."

        self.raw_window.protocol("WM_DELETE_WINDOW", self.close)
        self.map_window.protocol("WM_DELETE_WINDOW", self.map_window.withdraw)
        self.orientation_window.protocol("WM_DELETE_WINDOW", self.orientation_window.withdraw)
        self.control_window.protocol("WM_DELETE_WINDOW", self.control_window.withdraw)
        self._restore_window_state()
        self.poll()

    def _restore_window_state(self):
        state = load_window_state()

        for name, window in self.windows.items():
            saved = state.get("windows", {}).get(name)
            if not saved:
                continue
            geometry = saved.get("geometry")
            if geometry:
                try:
                    window.geometry(geometry)
                except tk.TclError:
                    pass
            if saved.get("visible", True):
                window.deiconify()
            else:
                window.withdraw()

        for name in state.get("stack_order", []):
            window = self.windows.get(name)
            if window is not None and window.state() != "withdrawn":
                window.lift()

    def _save_window_state(self):
        windows_state = {}
        for name, window in self.windows.items():
            try:
                windows_state[name] = {
                    "geometry": window.geometry(),
                    "visible": window.state() != "withdrawn",
                }
            except tk.TclError:
                continue

        stack_order = []
        try:
            path_to_name = {str(window): name for name, window in self.windows.items()}
            raw_order = self.raw_window.tk.call("wm", "stackorder", self.raw_window._w)
            if isinstance(raw_order, str):
                raw_order = raw_order.split()
            stack_order = [path_to_name[path] for path in raw_order if path in path_to_name]
        except tk.TclError:
            pass

        save_window_state({"windows": windows_state, "stack_order": stack_order})

    def check_sequence(self, kind, sequence):
        previous = self.last_sequences.get(kind)
        if previous is not None:
            expected = (previous + 1) & 0xFFFF
            if sequence != expected:
                gap = (sequence - expected) & 0xFFFF
                if gap < 1000:
                    self.stats.sequence_gaps += gap + 1
                else:
                    self.stats.sequence_resets += 1
        self.last_sequences[kind] = sequence

    def poll(self):
        processed = 0
        while processed < 500:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            processed += 1

            try:
                self._handle_event(event)
            except Exception as exc:
                self.connection = f"Event handling error: {exc}"

        try:
            self.raw_window.update_view(
                self.latest_fast, self.latest_status, self.stats, self.connection
            )
        except Exception as exc:
            self.connection = f"Display update error: {exc}"

        self.raw_window.after(100, self.poll)

    def _handle_event(self, event):
        if event[0] == "connected":
            self.connection = f"Connected: {event[1]} @ {self.baud}"
        elif event[0] == "reader_error":
            self.connection = f"Serial error: {event[1]}"
            self.stats.serial_errors += 1
        elif event[0] == "bad":
            self.stats.bad(event[2])
        elif event[0] == "packet":
            _, kind, data = event
            self.stats.good(kind)
            self.check_sequence(kind, data["sequence"])
            if kind == "fast":
                self.latest_fast = data
                self.map_window.update_position(data)
                self.raw_window.set_recording(flag(data["status"], STATUS_RECORDING))
            elif kind == "status":
                self.latest_status = data
                self.raw_window.set_recording(flag(data["status"], STATUS_RECORDING))
            elif kind == "orientation":
                self.latest_orientation = data
                self.orientation_window.render(data)
        elif event[0] == "command_ack":
            _, command_id, result = event
            name = COMMAND_NAMES.get(command_id, f"cmd 0x{command_id:02X}")
            result_text = ACK_RESULT_NAMES.get(result, f"result={result}")
            self.control_window._log_action(f"ACK: {name} - {result_text}")
        elif event[0] == "command_timeout":
            _, command_id, param = event
            name = COMMAND_NAMES.get(command_id, f"cmd 0x{command_id:02X}")
            self.control_window._log_action(
                f"NO ACK: {name} never acknowledged after {MAX_COMMAND_ATTEMPTS} "
                "attempts - it may not have reached the rocket."
            )
        elif event[0] == "alert":
            _, code, severity, message = event
            severity_name = ALERT_SEVERITY_NAMES.get(severity, f"severity={severity}")
            self.control_window._log_action(f"ALERT [{severity_name}] {message}")
            if severity == 2:
                self.control_window.deiconify()
                self.control_window.lift()

    def close(self):
        self._save_window_state()
        self.reader.stop()
        self.raw_window.destroy()

    def run(self):
        self.raw_window.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="Rocket telemetry ground station")
    parser.add_argument(
        "--port", default=DEFAULT_SERIAL_PORT,
        help=f"Serial port the GroundStation board is on (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD_RATE,
        help=f"Baud rate (default: {DEFAULT_BAUD_RATE})",
    )
    return parser.parse_args()


def check_port_exists(port):
    available = [p.device for p in serial.tools.list_ports.comports()]
    if port in available:
        return
    print(f"Serial port {port!r} not found.", file=sys.stderr)
    if available:
        print("Available ports:", file=sys.stderr)
        for device in available:
            print(f"  {device}", file=sys.stderr)
    else:
        print("No serial ports detected - is the GroundStation board plugged in?", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    args = parse_args()
    check_port_exists(args.port)
    App(args.port, args.baud).run()
