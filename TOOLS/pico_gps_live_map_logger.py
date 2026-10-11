import csv
import json
import mimetypes
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from string import Template

import serial
import serial.tools.list_ports

from PySide6.QtCore import QThread, QUrl, Signal
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# Requires: pip install PySide6 pyserial
#
# One-time online map preparation:
#   cd /Users/lsd/gps_logger
#   python3 prepare_offline_map.py
#
# After preparation, this logger works offline. It starts a localhost web page
# server and the local PMTiles CLI, which exposes the downloaded vector map as
# a MapLibre-compatible TileJSON/ZXY endpoint.

PROJECT_DIR = Path(__file__).resolve().parent
MAP_DIR = PROJECT_DIR / "offline_map"
VENDOR_DIR = MAP_DIR / "vendor"
TOOLS_DIR = MAP_DIR / "tools"
PMTILES_PATH = MAP_DIR / "dominican_republic.pmtiles"
TILESET_NAME = PMTILES_PATH.stem
TILE_SERVER_LOG = MAP_DIR / "pmtiles_server.log"

REQUIRED_VENDOR_FILES = (
    "maplibre-gl.js",
    "maplibre-gl.css",
)

TIME_RE = re.compile(r"\b(?P<time>\d{2}:\d{2}:\d{2}(?:\.\d+)?)\b")
LAT_RE = re.compile(
    r"\bLat(?:itude)?\s*[:=]?\s*(?P<value>[+-]?\d+(?:\.\d+)?)\s*(?P<hemisphere>[NS])\b",
    re.IGNORECASE,
)
LON_RE = re.compile(
    r"\bLon(?:gitude)?\s*[:=]?\s*(?P<value>[+-]?\d+(?:\.\d+)?)\s*(?P<hemisphere>[EW])\b",
    re.IGNORECASE,
)
HEIGHT_RE = re.compile(
    r"\b(?:Height|Hgt|Altitude|AltMSL|Alt|MSL)\s*[:=]?\s*"
    r"(?P<value>[+-]?\d+(?:\.\d+)?)\s*(?:m|meter|meters)?\b",
    re.IGNORECASE,
)


MAP_TEMPLATE = Template(r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PICO GPS Offline Live Map</title>
<link rel="stylesheet" href="/vendor/maplibre-gl.css">
<script src="/vendor/maplibre-gl.js"></script>
<style>
html, body, #map {
    width: 100%;
    height: 100%;
    margin: 0;
    padding: 0;
    overflow: hidden;
    background: #e8eee8;
    font-family: Arial, Helvetica, sans-serif;
}
.map-status {
    position: absolute;
    top: 10px;
    right: 10px;
    z-index: 10;
    padding: 7px 9px;
    border-radius: 5px;
    background: rgba(20, 29, 38, 0.84);
    color: #ffffff;
    font-size: 12px;
    pointer-events: none;
}
.map-error {
    position: absolute;
    left: 10px;
    bottom: 28px;
    z-index: 10;
    max-width: 60%;
    padding: 7px 9px;
    border-radius: 5px;
    background: rgba(160, 30, 30, 0.9);
    color: #ffffff;
    font-size: 12px;
    display: none;
}
</style>
</head>
<body>
<div id="map"></div>
<div class="map-status">Offline Dominican Republic map</div>
<div id="mapError" class="map-error"></div>
<script>
const initialRoute = $ROUTE_JSON;
const tileJsonUrl = $TILE_JSON_URL;

let route = initialRoute || [];
let followLatest = true;
let mapLoaded = false;

const mapStyle = {
    version: 8,
    sources: {
        protomaps: {
            type: "vector",
            url: tileJsonUrl,
            attribution: "Protomaps © OpenStreetMap contributors"
        }
    },
    layers: [
        {
            id: "background",
            type: "background",
            paint: {"background-color": "#e8eee8"}
        },
        {
            id: "earth",
            type: "fill",
            source: "protomaps",
            "source-layer": "earth",
            paint: {"fill-color": "#e6ecdf"}
        },
        {
            id: "landcover",
            type: "fill",
            source: "protomaps",
            "source-layer": "landcover",
            maxzoom: 8,
            paint: {"fill-color": "#d5e4cb", "fill-opacity": 0.72}
        },
        {
            id: "landuse",
            type: "fill",
            source: "protomaps",
            "source-layer": "landuse",
            paint: {
                "fill-color": [
                    "match", ["get", "kind"],
                    "park", "#cfe5c5",
                    "forest", "#cfe0c2",
                    "wood", "#cfe0c2",
                    "grass", "#dbe9cf",
                    "farmland", "#e9edd6",
                    "residential", "#e5e3d8",
                    "commercial", "#e7dfd4",
                    "industrial", "#dddad3",
                    "#e8e6dc"
                ],
                "fill-opacity": 0.72
            }
        },
        {
            id: "water",
            type: "fill",
            source: "protomaps",
            "source-layer": "water",
            paint: {"fill-color": "#a9d5e8", "fill-opacity": 0.92}
        },
        {
            id: "boundaries",
            type: "line",
            source: "protomaps",
            "source-layer": "boundaries",
            paint: {
                "line-color": "#9aa3ad",
                "line-width": ["interpolate", ["linear"], ["zoom"], 4, 0.5, 10, 1.4],
                "line-dasharray": [3, 2]
            }
        },
        {
            id: "buildings",
            type: "fill",
            source: "protomaps",
            "source-layer": "buildings",
            minzoom: 12,
            paint: {"fill-color": "#cbd0d2", "fill-opacity": 0.68}
        },
        {
            id: "roads-casing",
            type: "line",
            source: "protomaps",
            "source-layer": "roads",
            layout: {"line-cap": "round", "line-join": "round"},
            paint: {
                "line-color": "#8f969e",
                "line-opacity": 0.5,
                "line-width": ["interpolate", ["linear"], ["zoom"], 5, 0.4, 10, 1.5, 14, 5.5]
            }
        },
        {
            id: "roads",
            type: "line",
            source: "protomaps",
            "source-layer": "roads",
            layout: {"line-cap": "round", "line-join": "round"},
            paint: {
                "line-color": [
                    "match", ["get", "kind"],
                    "highway", "#f4b183",
                    "major_road", "#ffffff",
                    "medium_road", "#ffffff",
                    "minor_road", "#f7f7f7",
                    "path", "#c8b99a",
                    "#ffffff"
                ],
                "line-width": ["interpolate", ["linear"], ["zoom"], 5, 0.25, 10, 1.0, 14, 3.4]
            }
        },
        {
            id: "places",
            type: "circle",
            source: "protomaps",
            "source-layer": "places",
            minzoom: 5,
            paint: {
                "circle-color": "#59646f",
                "circle-radius": [
                    "interpolate", ["linear"], ["get", "population"],
                    0, 1.8,
                    100000, 3.0,
                    1000000, 5.0
                ],
                "circle-opacity": 0.7
            }
        }
    ]
};

const map = new maplibregl.Map({
    container: "map",
    style: mapStyle,
    center: [-70.15, 19.72],
    zoom: 7.8,
    minZoom: 5,
    maxZoom: 18,
    maxBounds: [[-73.0, 16.8], [-67.4, 20.7]],
    attributionControl: {compact: true}
});

map.addControl(new maplibregl.NavigationControl(), "top-left");
map.addControl(new maplibregl.ScaleControl({unit: "metric"}), "bottom-left");

function lineData() {
    return {
        type: "Feature",
        properties: {},
        geometry: {
            type: "LineString",
            coordinates: route.map(function(point) { return [point.lng, point.lat]; })
        }
    };
}

function endpointData() {
    const features = [];
    if (route.length) {
        features.push({
            type: "Feature",
            properties: {role: "start", time: route[0].time || ""},
            geometry: {type: "Point", coordinates: [route[0].lng, route[0].lat]}
        });
    }
    if (route.length > 1) {
        const last = route[route.length - 1];
        features.push({
            type: "Feature",
            properties: {role: "end", time: last.time || ""},
            geometry: {type: "Point", coordinates: [last.lng, last.lat]}
        });
    }
    return {type: "FeatureCollection", features: features};
}

function updateRouteLayers() {
    if (!mapLoaded) {
        return;
    }
    map.getSource("track-line").setData(lineData());
    map.getSource("track-endpoints").setData(endpointData());
}

function addTrackLayers() {
    map.addSource("track-line", {type: "geojson", data: lineData()});
    map.addSource("track-endpoints", {type: "geojson", data: endpointData()});

    map.addLayer({
        id: "track-line-casing",
        type: "line",
        source: "track-line",
        layout: {"line-cap": "round", "line-join": "round"},
        paint: {"line-color": "#111827", "line-width": 7, "line-opacity": 0.78}
    });

    map.addLayer({
        id: "track-line",
        type: "line",
        source: "track-line",
        layout: {"line-cap": "round", "line-join": "round"},
        paint: {"line-color": "#00a6ff", "line-width": 4, "line-opacity": 0.96}
    });

    map.addLayer({
        id: "track-endpoints",
        type: "circle",
        source: "track-endpoints",
        paint: {
            "circle-radius": 7,
            "circle-color": ["match", ["get", "role"], "start", "#22c55e", "#ef4444"],
            "circle-stroke-color": "#ffffff",
            "circle-stroke-width": 2
        }
    });
}

function fitRoute() {
    if (!mapLoaded || !route.length) {
        return;
    }

    if (route.length === 1) {
        map.easeTo({center: [route[0].lng, route[0].lat], zoom: 14, duration: 300});
        return;
    }

    const bounds = new maplibregl.LngLatBounds(
        [route[0].lng, route[0].lat],
        [route[0].lng, route[0].lat]
    );
    for (const point of route) {
        bounds.extend([point.lng, point.lat]);
    }
    map.fitBounds(bounds, {padding: 45, maxZoom: 15, duration: 400});
}

function addLivePoint(lat, lng, timeText, heightValue) {
    route.push({lat: lat, lng: lng, time: timeText, height: heightValue});
    updateRouteLayers();

    if (followLatest && mapLoaded) {
        const options = {center: [lng, lat], duration: 160};
        if (route.length === 1) {
            options.zoom = Math.max(map.getZoom(), 13.5);
        }
        map.easeTo(options);
    }
}

function syncRoute(nextRoute) {
    route = nextRoute || [];
    updateRouteLayers();
}

function clearRoute() {
    route = [];
    updateRouteLayers();
}

function setFollow(value) {
    followLatest = Boolean(value);
    if (followLatest && mapLoaded && route.length) {
        const last = route[route.length - 1];
        map.easeTo({center: [last.lng, last.lat], duration: 200});
    }
}

map.on("load", function() {
    mapLoaded = true;
    addTrackLayers();
    updateRouteLayers();
    if (route.length) {
        fitRoute();
    }
});

map.on("error", function(event) {
    const element = document.getElementById("mapError");
    const message = event && event.error ? event.error.message : "Unknown map error";
    element.textContent = message;
    element.style.display = "block";
});
</script>
</body>
</html>
""")


SETUP_HTML = f"""<!DOCTYPE html>
<html>
<body style="margin:0;background:#17202a;color:#eef3f8;display:grid;place-items:center;height:100vh;font-family:Arial,sans-serif;">
<div style="max-width:560px;padding:28px;line-height:1.55">
  <h1 style="margin-top:0">Offline map is not prepared</h1>
  <p>Run this once while connected to the internet:</p>
  <pre style="background:#0e151d;padding:14px;border-radius:6px">cd {PROJECT_DIR}
python3 prepare_offline_map.py</pre>
  <p>Then reopen this logger. The map will work without internet.</p>
</div>
</body>
</html>"""


class OfflineMapServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, map_html):
        super().__init__(server_address, handler_class)
        self.map_html = map_html


class OfflineMapRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "PicoOfflineMap/1.0"

    def log_message(self, format_string, *args):
        return

    def add_common_headers(self, content_length, content_type="application/octet-stream"):
        self.send_header("Content-Length", str(content_length))
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Range")
        self.send_header("Accept-Ranges", "bytes")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Range")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_HEAD(self):
        self.serve_request(send_body=False)

    def do_GET(self):
        self.serve_request(send_body=True)

    def serve_request(self, send_body):
        path = urllib.parse.urlparse(self.path).path

        if path in {"/", "/map.html"}:
            body = self.server.map_html.encode("utf-8")
            self.send_response(200)
            self.add_common_headers(len(body), "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            if send_body:
                self.wfile.write(body)
            return

        if path.startswith("/vendor/"):
            filename = Path(urllib.parse.unquote(path)).name
            file_path = VENDOR_DIR / filename
            if filename not in REQUIRED_VENDOR_FILES or not file_path.exists():
                self.send_error(404)
                return

            body = file_path.read_bytes()
            content_type = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
            self.send_response(200)
            self.add_common_headers(len(body), content_type)
            self.send_header("Cache-Control", "public, max-age=86400")
            self.end_headers()
            if send_body:
                self.wfile.write(body)
            return

        self.send_error(404)


class SerialReader(QThread):
    data_received = Signal(str)
    error = Signal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self.data_received.emit(line)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        self.wait(3000)


def parse_gps_line(line):
    lat_match = LAT_RE.search(line)
    lon_match = LON_RE.search(line)

    if not lat_match or not lon_match:
        return None

    time_match = TIME_RE.search(line)
    height_match = HEIGHT_RE.search(line)

    latitude = abs(float(lat_match.group("value")))
    if lat_match.group("hemisphere").upper() == "S":
        latitude = -latitude

    longitude = abs(float(lon_match.group("value")))
    if lon_match.group("hemisphere").upper() == "W":
        longitude = -longitude

    return {
        "time": time_match.group("time") if time_match else datetime.now().strftime("%H:%M:%S"),
        "latitude": latitude,
        "longitude": longitude,
        "height": float(height_match.group("value")) if height_match else None,
    }


def make_map_html(route_points, tile_json_url):
    route_json = json.dumps(route_points, separators=(",", ":")).replace("</", "<\\/")
    tile_json_json = json.dumps(tile_json_url)
    return MAP_TEMPLATE.substitute(
        ROUTE_JSON=route_json,
        TILE_JSON_URL=tile_json_json,
    )


def find_pmtiles_binary():
    if sys.platform.startswith("win"):
        tool_names = ("pmtiles.exe",)
    else:
        tool_names = ("pmtiles", "pmtiles.exe")

    for tool_name in tool_names:
        candidate = TOOLS_DIR / tool_name
        if candidate.is_file() and candidate.stat().st_size > 0:
            if sys.platform.startswith("win") or os.access(candidate, os.X_OK):
                return candidate

    system_binary = shutil.which("pmtiles")
    return Path(system_binary) if system_binary else None


def find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for_tile_server(process, port, timeout=10):
    tile_json_url = f"http://127.0.0.1:{port}/{TILESET_NAME}.json"
    deadline = time.monotonic() + timeout
    last_error = "the server did not respond"

    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"The PMTiles server exited with status {return_code}. "
                f"See {TILE_SERVER_LOG} for details."
            )

        try:
            request = urllib.request.Request(
                tile_json_url,
                headers={"User-Agent": "PICO-GPS-Live-Map/1.0"},
            )
            with urllib.request.urlopen(request, timeout=0.5) as response:
                if response.status == 200:
                    return
                last_error = f"HTTP {response.status}"
        except Exception as exc:
            last_error = str(exc)

        time.sleep(0.1)

    raise RuntimeError(
        f"Timed out waiting for the offline tile server ({last_error}). "
        f"See {TILE_SERVER_LOG} for details."
    )


def missing_map_files():
    missing = []
    if not PMTILES_PATH.exists() or PMTILES_PATH.stat().st_size == 0:
        missing.append(PMTILES_PATH.name)

    for filename in REQUIRED_VENDOR_FILES:
        file_path = VENDOR_DIR / filename
        if not file_path.exists() or file_path.stat().st_size == 0:
            missing.append(f"vendor/{filename}")

    if find_pmtiles_binary() is None:
        missing.append("tools/pmtiles")

    return missing


class PicoGpsLiveMapLogger(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PICO M9N Logger — Offline DR Live Map")
        self.resize(1240, 760)

        self.reader = None
        self.log_file = None
        self.csv_writer = None
        self.logging = False
        self.rows_logged = 0
        self.missing_height_warned = False
        self.route_points = []
        self.map_ready = False
        self.http_server = None
        self.http_thread = None
        self.tile_server_process = None
        self.tile_server_port = None

        self.init_ui()
        self.refresh_ports()
        self.load_offline_map()
        self.update_status("Select a port and press Connect.")

    def init_ui(self):
        root_layout = QHBoxLayout(self)
        root_layout.setContentsMargins(8, 8, 8, 8)

        splitter = QSplitter()
        root_layout.addWidget(splitter)

        left_panel = QWidget()
        left_panel.setMinimumWidth(390)
        left_panel.setMaximumWidth(480)
        left_layout = QVBoxLayout(left_panel)
        splitter.addWidget(left_panel)

        self.map_view = QWebEngineView()
        self.map_view.loadFinished.connect(self.on_map_loaded)
        splitter.addWidget(self.map_view)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        serial_row = QHBoxLayout()
        serial_row.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        serial_row.addWidget(self.port_combo, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        serial_row.addWidget(self.refresh_btn)
        left_layout.addLayout(serial_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["9600", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        baud_row.addWidget(self.baud_combo, 1)
        left_layout.addLayout(baud_row)

        connection_row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_serial)
        connection_row.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)
        connection_row.addWidget(self.disconnect_btn)
        left_layout.addLayout(connection_row)

        logging_row = QHBoxLayout()
        self.start_log_btn = QPushButton("Start Logging")
        self.start_log_btn.clicked.connect(self.start_logging)
        self.start_log_btn.setEnabled(False)
        logging_row.addWidget(self.start_log_btn)
        self.stop_log_btn = QPushButton("Stop Logging")
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.stop_log_btn.setEnabled(False)
        logging_row.addWidget(self.stop_log_btn)
        left_layout.addLayout(logging_row)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        left_layout.addWidget(self.status_label)

        self.latest_label = QLabel("Latest GPS: --")
        self.latest_label.setWordWrap(True)
        left_layout.addWidget(self.latest_label)

        left_layout.addWidget(QLabel("Offline map:"))
        self.map_status_label = QLabel()
        self.map_status_label.setWordWrap(True)
        left_layout.addWidget(self.map_status_label)

        self.reload_map_btn = QPushButton("Check / Load Offline Map")
        self.reload_map_btn.clicked.connect(self.load_offline_map)
        left_layout.addWidget(self.reload_map_btn)

        map_controls_row = QHBoxLayout()
        self.follow_checkbox = QCheckBox("Follow latest point")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.toggled.connect(self.set_follow_latest)
        map_controls_row.addWidget(self.follow_checkbox)

        self.fit_route_btn = QPushButton("Fit Route")
        self.fit_route_btn.clicked.connect(self.fit_route)
        map_controls_row.addWidget(self.fit_route_btn)

        self.clear_route_btn = QPushButton("Clear Track")
        self.clear_route_btn.clicked.connect(self.clear_track)
        map_controls_row.addWidget(self.clear_route_btn)
        left_layout.addLayout(map_controls_row)

        map_note = QLabel(
            "Map data is decoded locally from offline_map/dominican_republic.pmtiles. "
            "The map page and tiles are served from 127.0.0.1; no internet map requests are made."
        )
        map_note.setWordWrap(True)
        left_layout.addWidget(map_note)

        left_layout.addWidget(QLabel("Serial output:"))
        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.document().setMaximumBlockCount(5000)
        left_layout.addWidget(self.output, 1)

    def load_offline_map(self):
        missing = missing_map_files()
        self.map_ready = False
        self.stop_tile_server()

        if missing:
            self.map_status_label.setText(
                "Missing: " + ", ".join(missing) + ". Run prepare_offline_map.py once while online."
            )
            self.map_view.setHtml(SETUP_HTML)
            return

        try:
            self.start_map_server()
            tile_server_port = self.start_tile_server()
        except (OSError, RuntimeError) as exc:
            self.stop_tile_server()
            self.map_status_label.setText(f"Could not start the offline map: {exc}")
            self.map_view.setHtml(SETUP_HTML)
            return

        size_mb = PMTILES_PATH.stat().st_size / 1_000_000
        self.map_status_label.setText(
            f"Ready — dominican_republic.pmtiles ({size_mb:.1f} MB), loaded through localhost."
        )

        tile_json_url = f"http://127.0.0.1:{tile_server_port}/{TILESET_NAME}.json"
        self.http_server.map_html = make_map_html(self.route_points, tile_json_url)
        port = self.http_server.server_address[1]
        self.map_view.load(QUrl(f"http://127.0.0.1:{port}/map.html"))

    def start_map_server(self):
        if self.http_server:
            return

        self.http_server = OfflineMapServer(
            ("127.0.0.1", 0),
            OfflineMapRequestHandler,
            "",
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            kwargs={"poll_interval": 0.2},
            daemon=True,
        )
        self.http_thread.start()

    def start_tile_server(self):
        binary = find_pmtiles_binary()
        if binary is None:
            raise RuntimeError("The pmtiles command-line tool was not found.")

        if not self.http_server:
            raise RuntimeError("The local map page server is not running.")

        port = find_free_port()
        public_url = f"http://127.0.0.1:{port}"
        map_origin = f"http://127.0.0.1:{self.http_server.server_address[1]}"
        command = [
            str(binary),
            "serve",
            str(MAP_DIR),
            f"--port={port}",
            f"--cors={map_origin}",
            f"--public-url={public_url}",
        ]

        popen_options = {
            "cwd": str(MAP_DIR),
            "stdin": subprocess.DEVNULL,
            "stdout": None,
            "stderr": subprocess.STDOUT,
        }
        if sys.platform.startswith("win") and hasattr(subprocess, "CREATE_NO_WINDOW"):
            popen_options["creationflags"] = subprocess.CREATE_NO_WINDOW

        try:
            with TILE_SERVER_LOG.open("ab") as log_file:
                log_file.write(
                    ("\n--- PMTiles server start "
                     + datetime.now().isoformat(timespec="seconds")
                     + " ---\n"
                     + " ".join(command)
                     + "\n").encode("utf-8")
                )
                log_file.flush()
                popen_options["stdout"] = log_file
                process = subprocess.Popen(command, **popen_options)
        except OSError as exc:
            raise RuntimeError(f"Could not run {binary}: {exc}") from exc

        try:
            wait_for_tile_server(process, port)
        except Exception:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            raise

        self.tile_server_process = process
        self.tile_server_port = port
        return port

    def stop_tile_server(self):
        process = self.tile_server_process
        self.tile_server_process = None
        self.tile_server_port = None

        if not process or process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)

    def refresh_ports(self):
        current_port = self.port_combo.currentText()
        self.port_combo.clear()

        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            description = f"{port.device} — {port.description}"
            self.port_combo.addItem(description, port.device)

        if current_port:
            index = self.port_combo.findText(current_port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

        if not ports:
            self.update_status("No serial ports found. Connect the PICO and press Refresh.")

    def selected_port(self):
        return self.port_combo.currentData() or self.port_combo.currentText().split(" — ")[0]

    def connect_serial(self):
        if self.reader:
            return

        port = self.selected_port().strip()
        if not port:
            QMessageBox.warning(self, "No port", "Select a serial port first.")
            return

        try:
            baudrate = int(self.baud_combo.currentText().strip())
        except ValueError:
            QMessageBox.warning(self, "Invalid baud rate", "Enter a numeric baud rate.")
            return

        self.reader = SerialReader(port, baudrate)
        self.reader.data_received.connect(self.handle_serial_line)
        self.reader.error.connect(self.on_serial_error)
        self.reader.start()

        self.append_message(f"Connected to {port} at {baudrate} baud.")
        self.update_status("Connected. GPS fixes will appear on the map automatically.")

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.start_log_btn.setEnabled(True)
        self.refresh_btn.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)

    def disconnect_serial(self):
        if self.logging:
            self.stop_logging()

        if self.reader:
            self.reader.stop()
            self.reader = None
            self.append_message("Disconnected.")

        self.update_status("Disconnected.")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.start_log_btn.setEnabled(False)
        self.stop_log_btn.setEnabled(False)
        self.refresh_btn.setEnabled(True)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)

    def start_logging(self):
        if not self.reader:
            QMessageBox.warning(self, "Not connected", "Connect to the PICO before logging.")
            return

        default_name = datetime.now().strftime("pico_m9n_log_%Y%m%d_%H%M%S.csv")
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save GPS Log",
            str(Path.home() / default_name),
            "CSV Files (*.csv)",
        )

        if not filename:
            return

        if not filename.lower().endswith(".csv"):
            filename += ".csv"

        try:
            self.log_file = open(filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(["time", "latitude_deg", "longitude_deg", "height_m"])
            self.log_file.flush()
        except OSError as exc:
            self.log_file = None
            self.csv_writer = None
            QMessageBox.critical(self, "Cannot open file", str(exc))
            return

        self.logging = True
        self.rows_logged = 0
        self.missing_height_warned = False

        self.start_log_btn.setEnabled(False)
        self.stop_log_btn.setEnabled(True)
        self.append_message(f"Logging started: {filename}")
        self.update_status(f"Logging to {Path(filename).name} — 0 rows")

    def stop_logging(self):
        if not self.logging:
            return

        self.logging = False

        if self.log_file:
            self.log_file.flush()
            self.log_file.close()

        filename = self.log_file.name
        self.log_file = None
        self.csv_writer = None

        self.start_log_btn.setEnabled(self.reader is not None)
        self.stop_log_btn.setEnabled(False)
        self.append_message(f"Logging stopped. Saved {self.rows_logged} rows to {filename}")
        self.update_status(f"Logging stopped — {self.rows_logged} rows saved.")

    def handle_serial_line(self, line):
        self.append_message(line)
        parsed = parse_gps_line(line)

        if not parsed:
            return

        self.add_track_point(parsed)
        height_text = "--" if parsed["height"] is None else f"{parsed['height']:.1f} m"
        self.latest_label.setText(
            f"Latest GPS: {parsed['time']} | "
            f"{parsed['latitude']:.7f}, {parsed['longitude']:.7f} | "
            f"height {height_text}"
        )

        if not self.logging or not self.csv_writer:
            return

        self.csv_writer.writerow(
            [
                parsed["time"],
                f"{parsed['latitude']:.7f}",
                f"{parsed['longitude']:.7f}",
                "" if parsed["height"] is None else f"{parsed['height']:.2f}",
            ]
        )
        self.log_file.flush()
        self.rows_logged += 1

        if parsed["height"] is None and not self.missing_height_warned:
            self.missing_height_warned = True
            self.append_message(
                "[NOTE] Coordinates are being logged, but no height field was found in the serial text."
            )

        self.update_status(
            f"Logging to {Path(self.log_file.name).name} — {self.rows_logged} rows"
        )

    def add_track_point(self, parsed):
        point = {
            "lat": parsed["latitude"],
            "lng": parsed["longitude"],
            "time": parsed["time"],
            "height": parsed["height"],
        }
        self.route_points.append(point)

        if self.map_ready:
            self.set_follow_latest(self.follow_checkbox.isChecked())
            script = "addLivePoint(%s, %s, %s, %s);" % (
                json.dumps(point["lat"]),
                json.dumps(point["lng"]),
                json.dumps(point["time"]),
                json.dumps(point["height"]),
            )
            self.map_view.page().runJavaScript(script)

    def on_map_loaded(self, ok):
        tile_server_running = (
            self.tile_server_process is not None
            and self.tile_server_process.poll() is None
        )
        self.map_ready = bool(
            ok
            and tile_server_running
            and self.http_server is not None
            and bool(self.http_server.map_html)
        )

        if not self.map_ready:
            return

        self.set_follow_latest(self.follow_checkbox.isChecked())
        self.sync_route()

    def sync_route(self):
        if not self.map_ready:
            return
        route_json = json.dumps(self.route_points, separators=(",", ":"))
        self.map_view.page().runJavaScript(f"syncRoute({route_json});")

    def set_follow_latest(self, enabled):
        if self.map_ready:
            self.map_view.page().runJavaScript(f"setFollow({str(bool(enabled)).lower()});")

    def fit_route(self):
        if self.map_ready:
            self.map_view.page().runJavaScript("fitRoute();")

    def clear_track(self):
        self.route_points = []
        if self.map_ready:
            self.map_view.page().runJavaScript("clearRoute();")
        self.latest_label.setText("Latest GPS: --")

    def on_serial_error(self, message):
        self.append_message(f"[ERROR] {message}")
        self.disconnect_serial()

    def append_message(self, message):
        self.output.append(message)
        scrollbar = self.output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def update_status(self, message):
        self.status_label.setText(f"Status: {message}")

    def closeEvent(self, event):
        if self.logging:
            self.stop_logging()

        if self.reader:
            self.reader.stop()
            self.reader = None

        self.map_ready = False
        self.stop_tile_server()

        if self.http_server:
            self.http_server.shutdown()
            self.http_server.server_close()
            self.http_server = None

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PicoGpsLiveMapLogger()
    window.show()
    sys.exit(app.exec())
