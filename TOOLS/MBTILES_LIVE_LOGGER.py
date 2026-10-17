# MBTILES_LIVE_LOGGER.py
#
# Offline MBTiles map + live serial logger (PySide6)
#
# Map:    fully offline raster .mbtiles (SQLite, z/x/y PNG/JPEG tiles)
#         pan with left-drag, wheel zooms toward cursor, tiles load in a
#         worker thread so the GUI never blocks on the DB.
#
# Logger: reads your 5-line telemetry block over serial:
#   Q:-0.936 -0.021  0.006 -0.351        <- quaternions (w x y z)
#   A: 0.072  0.045  1.044 -0.351        <- acceleration (x y z, norm)
#   Lat 19.7003479  Lon -71.4378668  hMSL 6492 mm  fix=3  sats=6
#   temp : 24.320
#   press: 101194.045
#
# A sample is committed to CSV + the live map track when the 'press' line
# closes the block. hMSL is converted mm -> m in the CSV.
#
# Requires: pip install PySide6 pyserial

import sys
import math
import csv
import re
import sqlite3
import os
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

from PySide6.QtWidgets import (
    QApplication, QWidget, QGraphicsView, QGraphicsScene,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox, QComboBox, QTextEdit
)
from PySide6.QtCore import Qt, QThread, Signal, QMutex, QWaitCondition, QRectF, QPointF
from PySide6.QtGui import QPixmap, QPainter, QPen, QColor, QWheelEvent, QMouseEvent, QPolygonF

TILE = 256
MAX_CACHE = 600
MAX_TRACK_POINTS = 20000      # ring-buffer cap for the live track


# ---------------- packet regexes ----------------
NUM = r"[+-]?\d+(?:\.\d+)?"
Q_RE = re.compile(r"^\s*Q\s*[:=]?\s*(%s)\s+(%s)\s+(%s)\s+(%s)" % (NUM, NUM, NUM, NUM))
A_RE = re.compile(r"^\s*A\s*[:=]?\s*(%s)\s+(%s)\s+(%s)(?:\s+(%s))?" % (NUM, NUM, NUM, NUM))
LATLON_RE = re.compile(
    r"Lat\s*[:=]?\s*(?P<lat>%s).*?Lon\s*[:=]?\s*(?P<lon>%s).*?"
    r"hMSL\s*[:=]?\s*(?P<hmsl>%s)\s*mm.*?fix\s*=\s*(?P<fix>\d+).*?"
    r"sats\s*=\s*(?P<sats>\d+)" % (NUM, NUM, NUM),
    re.IGNORECASE)
TEMP_RE = re.compile(r"temp\s*[:=]?\s*(?P<value>%s)" % NUM, re.IGNORECASE)
PRESS_RE = re.compile(r"press\s*[:=]?\s*(?P<value>%s)" % NUM, re.IGNORECASE)

CSV_HEADER = ["time_pc", "qw", "qx", "qy", "qz",
              "ax", "ay", "az",
              "lat_deg", "lon_deg", "hmsl_m",
              "fix", "sats", "temp_c", "press_pa"]


# ---------------- slippy map math ----------------
def lonlat_to_global_px(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * TILE * n
    lat = max(min(lat, 85.05112878), -85.05112878)
    latrad = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(latrad)) / math.pi) / 2.0 * TILE * n
    return x, y


def global_px_to_lonlat(x, y, z):
    n = 2 ** z
    lon = x / (TILE * n) * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / (TILE * n)))))
    return lon, lat


# ---------------- MBTiles worker ----------------
class TileWorker(QThread):
    tile_ready = Signal(int, int, int, QPixmap)

    def __init__(self, db_path):
        super().__init__()
        self.db_path = db_path
        self.con = None
        self.pending = []
        self.mutex = QMutex()
        self.cond = QWaitCondition()
        self.running = True

    def request(self, z, x, y):
        self.mutex.lock()
        self.pending.append((z, x, y))
        self.cond.wakeOne()
        self.mutex.unlock()

    def run(self):
        self.con = sqlite3.connect("file:%s?mode=ro" % self.db_path, uri=True)
        while self.running:
            self.mutex.lock()
            while self.running and not self.pending:
                self.cond.wait(self.mutex)
            batch = self.pending[:64]
            self.pending = self.pending[64:]
            self.mutex.unlock()

            for (z, x, y) in batch:
                row = self.con.execute(
                    "SELECT tile_data FROM tiles "
                    "WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                    (z, x, 2 ** z - 1 - y)).fetchone()   # MBTiles rows are TMS: flip y
                if row:
                    pm = QPixmap()
                    pm.loadFromData(row[0])
                    if not pm.isNull():
                        self.tile_ready.emit(z, x, y, pm)
        if self.con:
            self.con.close()

    def stop(self):
        self.mutex.lock()
        self.running = False
        self.cond.wakeAll()
        self.mutex.unlock()
        self.wait(2000)


# ---------------- map widget with live track ----------------
class MapView(QGraphicsView):
    coords_changed = Signal(float, float)

    def __init__(self):
        super().__init__()
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self.worker = None
        self.cache = {}
        self.requested = set()

        self.z = 3
        self.min_zoom = 0
        self.max_zoom = 14
        self.bounds = None

        self.track = []          # list of (lon, lat), oldest first
        self.latest = None       # (lon, lat) newest fix

        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QColor(232, 232, 230))
        self._scene.setSceneRect(QRectF(0, 0, TILE << self.z, TILE << self.z))

    # ---- source ----
    def set_source(self, db_path):
        con = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
        meta = dict(con.execute("SELECT name, value FROM metadata"))
        con.close()

        self.min_zoom = int(float(meta.get("minzoom", 0)))
        self.max_zoom = int(float(meta.get("maxzoom", 14)))
        if "bounds" in meta:
            w, s, e, n = [float(v) for v in meta["bounds"].split(",")]
            self.bounds = (w, s, e, n)

        if self.worker:
            self.worker.stop()
        self.worker = TileWorker(db_path)
        self.worker.tile_ready.connect(self.on_tile)
        self.worker.start()

        self.cache.clear()
        self.requested.clear()
        self.go_home()

    def on_tile(self, z, x, y, pm):
        self.cache[(z, x, y)] = pm
        self.viewport().update()

    def go_home(self):
        if self.bounds:
            w, s, e, n = self.bounds
            lon, lat = (w + e) / 2.0, (s + n) / 2.0
            self.z = self._fit_zoom(w, s, e, n)
        else:
            lon, lat = -69.9, 18.5
            self.z = 8
        self._apply_zoom()
        self.centerOn(*lonlat_to_global_px(lon, lat, self.z))
        self.viewport().update()

    def _fit_zoom(self, w, s, e, n):
        vw = max(self.width(), 800)
        vh = max(self.height(), 600)
        zx = math.log2(vw * 0.9 * 360.0 / (max(e - w, 1e-6) * TILE))
        zy = math.log2(vh * 0.9 * 360.0 / (max(n - s, 1e-6) * TILE))
        return max(self.min_zoom, min(self.max_zoom, int(min(zx, zy))))

    # ---- zoom ----
    def set_zoom(self, z):
        z = max(self.min_zoom, min(self.max_zoom, z))
        if z != self.z:
            self.z = z
            self._apply_zoom()

    def _apply_zoom(self):
        world = TILE << self.z
        self._scene.setSceneRect(QRectF(0, 0, world, world))

    def wheelEvent(self, event: QWheelEvent):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        old_z = self.z
        anchor = self.mapToScene(event.position().toPoint())
        self.set_zoom(old_z + (1 if delta > 0 else -1))
        if self.z != old_z:
            factor = float(2 ** (self.z - old_z))
            self.centerOn(anchor.x() * factor, anchor.y() * factor)
            self.viewport().update()

    # ---- live track ----
    def add_fix(self, lon, lat, follow=True):
        self.track.append((lon, lat))
        if len(self.track) > MAX_TRACK_POINTS:
            del self.track[: len(self.track) - MAX_TRACK_POINTS]
        self.latest = (lon, lat)
        if follow and self.latest:
            self.centerOn(*lonlat_to_global_px(lon, lat, self.z))
        self.viewport().update()

    def clear_track(self):
        self.track = []
        self.latest = None
        self.viewport().update()

    def fit_track(self):
        if len(self.track) < 2:
            return
        lons = [p[0] for p in self.track]
        lats = [p[1] for p in self.track]
        w, e, s, n = min(lons), max(lons), min(lats), max(lats)
        self.z = self._fit_zoom(w, s, e, n)
        self._apply_zoom()
        self.centerOn(*lonlat_to_global_px((w + e) / 2, (s + n) / 2, self.z))
        self.viewport().update()

    # ---- painting ----
    def drawBackground(self, painter, rect):
        z = self.z
        n = 2 ** z
        painter.fillRect(rect, QColor(232, 232, 230))

        x0 = max(int(rect.left()) // TILE, 0)
        x1 = min(int(rect.right()) // TILE, n - 1)
        y0 = max(int(rect.top()) // TILE, 0)
        y1 = min(int(rect.bottom()) // TILE, n - 1)

        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                pm = self.cache.get((z, tx, ty))
                if pm is not None:
                    painter.drawPixmap(tx * TILE, ty * TILE, pm)
                else:
                    painter.fillRect(tx * TILE, ty * TILE, TILE, TILE,
                                     QColor(232, 232, 230))
                    if self.worker and (z, tx, ty) not in self.requested:
                        self.requested.add((z, tx, ty))
                        self.worker.request(z, tx, ty)

        while len(self.cache) > MAX_CACHE:
            self.cache.pop(next(iter(self.cache)))

    def drawForeground(self, painter, rect):
        # everything here is in scene coordinates -> moves with pan/zoom
        # wrapped in try/except: an exception must NEVER escape a paint
        # override - repeated escapes segfault Qt
        try:
            if len(self.track) >= 2:
                pen_casing = QPen(QColor(17, 24, 39, 200), 7)
                pen_line = QPen(QColor(0, 166, 255, 245), 4)
                pen_casing.setCapStyle(Qt.RoundCap)
                pen_casing.setJoinStyle(Qt.RoundJoin)
                pen_line.setCapStyle(Qt.RoundCap)
                pen_line.setJoinStyle(Qt.RoundJoin)

                for pen in (pen_casing, pen_line):
                    painter.setPen(pen)
                    # decimate when the track is long so painting stays fast
                    step = max(1, len(self.track) // 4000)
                    pts = self.track[::step]
                    if pts[-1] != self.track[-1]:
                        pts.append(self.track[-1])
                    # PySide6 drawPolyline needs a QPolygonF, not list-of-tuples
                    poly = QPolygonF([QPointF(*lonlat_to_global_px(lon, lat, self.z))
                                      for lon, lat in pts])
                    painter.drawPolyline(poly)

            if self.latest is not None:
                x, y = lonlat_to_global_px(*self.latest, self.z)
                painter.setPen(QPen(QColor(255, 255, 255), 2))
                painter.setBrush(QColor(239, 68, 68))
                painter.drawEllipse(QRectF(x - 7, y - 7, 14, 14))
        except Exception:
            pass   # degrade gracefully; never let paint crash the app

    def mouseMoveEvent(self, event: QMouseEvent):
        sp = self.mapToScene(event.position().toPoint())
        lon, lat = global_px_to_lonlat(sp.x(), sp.y(), self.z)
        self.coords_changed.emit(lon, lat)
        super().mouseMoveEvent(event)


# ---------------- serial reader ----------------
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


# ---------------- telemetry block parser ----------------
class TelemetryParser:
    """Accumulates the 5-line block; emits a complete sample when
    the 'press' line closes the block."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.q = None
        self.a = None
        self.gps = None
        self.temp = None
        self.press = None

    def feed(self, line):
        """Returns a complete sample dict or None."""
        m = Q_RE.match(line)
        if m:
            self.q = tuple(float(v) for v in m.groups())
            return None

        m = A_RE.match(line)
        if m:
            vals = [float(v) for v in m.groups() if v is not None]
            self.a = tuple(vals)
            return None

        m = LATLON_RE.search(line)
        if m:
            self.gps = {
                "lat": float(m.group("lat")),      # signed, N+/S-
                "lon": float(m.group("lon")),      # signed, E+/W-
                "hmsl_m": float(m.group("hmsl")) / 1000.0,
                "fix": int(m.group("fix")),
                "sats": int(m.group("sats")),
            }
            return None

        m = TEMP_RE.search(line)
        if m:
            self.temp = float(m.group("value"))
            return None

        m = PRESS_RE.search(line)
        if m:
            self.press = float(m.group("value"))
            # block closes here -> commit if we at least have a GPS fix
            if self.gps is not None:
                sample = {
                    "time_pc": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    "qw": self.q[0] if self.q else "",
                    "qx": self.q[1] if self.q else "",
                    "qy": self.q[2] if self.q else "",
                    "qz": self.q[3] if self.q else "",
                    "ax": self.a[0] if self.a else "",
                    "ay": self.a[1] if self.a else "",
                    "az": self.a[2] if self.a else "",
                    "lat_deg": self.gps["lat"],
                    "lon_deg": self.gps["lon"],
                    "hmsl_m": self.gps["hmsl_m"],
                    "fix": self.gps["fix"],
                    "sats": self.gps["sats"],
                    "temp_c": self.temp if self.temp is not None else "",
                    "press_pa": self.press,
                }
                self.reset()
                return sample
            self.reset()   # incomplete block, discard
        return None


# ---------------- main window ----------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LSD — Offline MBTiles Live Logger")
        self.resize(1240, 820)

        self.reader = None
        self.parser = TelemetryParser()
        self.log_file = None
        self.csv_writer = None
        self.logging = False
        self.rows_logged = 0
        self.follow = True

        # ---- layout ----
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        # map buttons
        map_row = QHBoxLayout()
        self.btn_open = QPushButton("Open Map...")
        self.btn_open.clicked.connect(self.on_open_map)
        map_row.addWidget(self.btn_open)

        self.btn_zoom_in = QPushButton("Zoom +")
        self.btn_zoom_in.clicked.connect(lambda: self.map.set_zoom(self.map.z + 1))
        map_row.addWidget(self.btn_zoom_in)

        self.btn_zoom_out = QPushButton("Zoom -")
        self.btn_zoom_out.clicked.connect(lambda: self.map.set_zoom(self.map.z - 1))
        map_row.addWidget(self.btn_zoom_out)

        self.btn_home = QPushButton("Home")
        self.btn_home.clicked.connect(lambda: self.map.go_home())
        map_row.addWidget(self.btn_home)

        self.btn_fit = QPushButton("Fit Track")
        self.btn_fit.clicked.connect(lambda: self.map.fit_track())
        map_row.addWidget(self.btn_fit)

        self.btn_clear = QPushButton("Clear Track")
        self.btn_clear.clicked.connect(lambda: self.map.clear_track())
        map_row.addWidget(self.btn_clear)

        self.follow_check = QComboBox()
        self.follow_check.addItems(["Follow: ON", "Follow: OFF"])
        self.follow_check.currentIndexChanged.connect(
            lambda i: setattr(self, "follow", i == 0))
        map_row.addWidget(self.follow_check)

        map_row.addStretch(1)
        layout.addLayout(map_row)

        # serial row
        ser_row = QHBoxLayout()
        ser_row.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        ser_row.addWidget(self.port_combo, 1)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        ser_row.addWidget(self.btn_refresh)

        ser_row.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["9600", "38400", "57600", "115200",
                                  "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        ser_row.addWidget(self.baud_combo)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_serial)
        ser_row.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self.disconnect_serial)
        self.btn_disconnect.setEnabled(False)
        ser_row.addWidget(self.btn_disconnect)

        self.btn_start_log = QPushButton("Start Logging")
        self.btn_start_log.clicked.connect(self.start_logging)
        self.btn_start_log.setEnabled(False)
        ser_row.addWidget(self.btn_start_log)

        self.btn_stop_log = QPushButton("Stop Logging")
        self.btn_stop_log.clicked.connect(self.stop_logging)
        self.btn_stop_log.setEnabled(False)
        ser_row.addWidget(self.btn_stop_log)
        layout.addLayout(ser_row)

        # map
        self.map = MapView()
        self.map.coords_changed.connect(self.on_coords)
        layout.addWidget(self.map, 1)

        # latest fix label
        self.latest_label = QLabel("Latest fix: --")
        layout.addWidget(self.latest_label)

        # status + console
        self.status = QLabel("Open a .mbtiles map, connect the device, start logging.")
        layout.addWidget(self.status)

        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.document().setMaximumBlockCount(3000)
        self.output.setMaximumHeight(180)
        layout.addWidget(self.output)

        self.db_path = None
        self.refresh_ports()

    # ---- map ----
    def on_open_map(self):
        start_dir = os.path.expanduser("~/Desktop/QGIS_CROP")
        path, _ = QFileDialog.getOpenFileName(
            self, "Open MBTiles map", start_dir, "MBTiles (*.mbtiles)")
        if not path:
            return
        try:
            self.map.set_source(path)
        except Exception as e:
            QMessageBox.critical(self, "Open failed", str(e))
            return
        self.db_path = path
        self.status.setText("Map: %s (z%d)" % (os.path.basename(path), self.map.z))

    def on_coords(self, lon, lat):
        base = self.latest_label.text()
        if " | cursor:" in base:
            base = base.split(" | cursor:")[0]
        self.latest_label.setText("%s | cursor: %.5f, %.5f | z%d"
                                  % (base, lat, lon, self.map.z))

    # ---- serial ----
    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

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
            baud = int(self.baud_combo.currentText().strip())
        except ValueError:
            QMessageBox.warning(self, "Invalid baud rate", "Enter a numeric baud rate.")
            return

        self.parser.reset()
        self.reader = SerialReader(port, baud)
        self.reader.data_received.connect(self.handle_line)
        self.reader.error.connect(self.on_serial_error)
        self.reader.start()

        self.append(f"Connected to {port} @ {baud}.")
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.btn_start_log.setEnabled(True)
        self.btn_refresh.setEnabled(False)

    def disconnect_serial(self):
        if self.logging:
            self.stop_logging()
        if self.reader:
            self.reader.stop()
            self.reader = None
            self.append("Disconnected.")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.btn_start_log.setEnabled(False)
        self.btn_refresh.setEnabled(True)

    # ---- logging ----
    def start_logging(self):
        if not self.reader:
            QMessageBox.warning(self, "Not connected", "Connect the device first.")
            return

        default_name = datetime.now().strftime("telemetry_%Y%m%d_%H%M%S.csv")
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save telemetry CSV",
            str(Path.home() / default_name), "CSV Files (*.csv)")
        if not filename:
            return
        if not filename.lower().endswith(".csv"):
            filename += ".csv"

        try:
            self.log_file = open(filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(self.log_file, fieldnames=CSV_HEADER)
            self.csv_writer.writeheader()
            self.log_file.flush()
        except OSError as exc:
            self.log_file = None
            self.csv_writer = None
            QMessageBox.critical(self, "Cannot open file", str(exc))
            return

        self.logging = True
        self.rows_logged = 0
        self.btn_start_log.setEnabled(False)
        self.btn_stop_log.setEnabled(True)
        self.append(f"Logging started: {filename}")
        self.status.setText(f"Logging to {Path(filename).name} — 0 rows")

    def stop_logging(self):
        if not self.logging:
            return
        self.logging = False
        if self.log_file:
            self.log_file.flush()
            self.log_file.close()
            name = self.log_file.name
        else:
            name = "?"
        self.log_file = None
        self.csv_writer = None
        self.btn_start_log.setEnabled(self.reader is not None)
        self.btn_stop_log.setEnabled(False)
        self.append(f"Logging stopped. Saved {self.rows_logged} rows to {name}")
        self.status.setText(f"Logging stopped — {self.rows_logged} rows saved.")

    # ---- data ----
    def handle_line(self, line):
        self.append(line)
        sample = self.parser.feed(line)
        if sample is None:
            return

        # live map
        self.map.add_fix(sample["lon_deg"], sample["lat_deg"],
                         follow=self.follow)

        # latest label
        def _v(x):
            return float("nan") if x == "" else x

        self.latest_label.setText(
            "Latest fix: %s | %.7f, %.7f | hMSL %.1f m | fix=%d sats=%d | "
            "Q(%.3f %.3f %.3f %.3f) A(%.3f %.3f %.3f) | %.2f C, %.0f Pa"
            % (sample["time_pc"], sample["lat_deg"], sample["lon_deg"],
               sample["hmsl_m"], sample["fix"], sample["sats"],
               _v(sample["qw"]), _v(sample["qx"]), _v(sample["qy"]), _v(sample["qz"]),
               _v(sample["ax"]), _v(sample["ay"]), _v(sample["az"]),
               _v(sample["temp_c"]), sample["press_pa"]))

        # csv
        if self.logging and self.csv_writer:
            self.csv_writer.writerow(sample)
            self.log_file.flush()
            self.rows_logged += 1
            if self.db_path:
                self.status.setText(
                    "%s | z%d | Logging to %s — %d rows"
                    % (os.path.basename(self.db_path), self.map.z,
                       Path(self.log_file.name).name, self.rows_logged))
            else:
                self.status.setText("Logging — %d rows" % self.rows_logged)

    def on_serial_error(self, message):
        self.append(f"[ERROR] {message}")
        self.disconnect_serial()

    def append(self, message):
        self.output.append(message)
        sb = self.output.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event):
        if self.logging:
            self.stop_logging()
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.map.worker:
            self.map.worker.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
