# MBTILES_MAP_VIEWER.py BY Lorenzo Daidone - MIT
#
# Offline MBTiles map viewer (PySide6)
#
# - Opens a raster .mbtiles file (SQLite, z/x/y PNG/JPEG tiles) - fully offline
# - Pan with left-drag, zoom with mouse wheel (zooms toward cursor)
# - Tiles load in a worker thread so the GUI never blocks on the DB
# - Red crosshair marks the center of the view, status bar shows lat/lon + zoom
#
# Notes learned the hard way:
#   * MBTiles rows are TMS (y flipped): tile_row = 2^z - 1 - y
#   * open the DB read-only (file:...?mode=ro) - never write to the map in the field
#   * SQLite point lookups are ~1 ms, one worker thread + a small RAM cache is enough

import sys
import math
import sqlite3
import os

from PySide6.QtWidgets import (
    QApplication, QWidget, QGraphicsView, QGraphicsScene,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt, QThread, Signal, QMutex, QWaitCondition, QRectF
from PySide6.QtGui import QPixmap, QPainter, QPen, QColor, QMouseEvent, QWheelEvent

TILE = 256                 # tile size in pixels (web mercator)
MAX_CACHE = 600            # max pixmaps kept in RAM before evicting oldest


# ---------------- Slippy map math (lon/lat <-> global pixels) ----------------
def lonlat_to_global_px(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * TILE * n
    lat = max(min(lat, 85.05112878), -85.05112878)   # web mercator clamp
    latrad = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(latrad)) / math.pi) / 2.0 * TILE * n
    return x, y


def global_px_to_lonlat(x, y, z):
    n = 2 ** z
    lon = x / (TILE * n) * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / (TILE * n)))))
    return lon, lat


# ---------------- MBTiles worker thread ----------------
class TileWorker(QThread):
    tile_ready = Signal(int, int, int, QPixmap)   # z, x, y, pixmap

    def __init__(self, db_path):
        super().__init__()
        self.db_path = db_path
        self.con = None
        self.pending = []          # (z, x, y) queue
        self.mutex = QMutex()
        self.cond = QWaitCondition()
        self.running = True

    def request(self, z, x, y):
        self.mutex.lock()
        self.pending.append((z, x, y))
        self.cond.wakeOne()
        self.mutex.unlock()

    def run(self):
        # read-only URI: the map file is never modified
        self.con = sqlite3.connect("file:%s?mode=ro" % self.db_path, uri=True)
        while self.running:
            self.mutex.lock()
            while self.running and not self.pending:
                self.cond.wait(self.mutex)
            batch = self.pending[:64]
            self.pending = self.pending[64:]
            self.mutex.unlock()

            for (z, x, y) in batch:
                # MBTiles stores TMS rows: FLIP Y before querying
                row = self.con.execute(
                    "SELECT tile_data FROM tiles "
                    "WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                    (z, x, 2 ** z - 1 - y)).fetchone()
                if row:
                    pm = QPixmap()
                    pm.loadFromData(row[0])      # auto-detects PNG/JPEG/WebP
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


# ---------------- Map widget ----------------
class MapView(QGraphicsView):
    coords_changed = Signal(float, float)   # lon, lat under cursor
    zoom_changed = Signal(int)

    def __init__(self):
        super().__init__()
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self.worker = None
        self.cache = {}            # (z,x,y) -> QPixmap
        self.requested = set()     # in-flight tiles, do not queue twice

        self.z = 3
        self.min_zoom = 0
        self.max_zoom = 14
        self.bounds = None         # (w, s, e, n) from metadata

        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setMouseTracking(True)
        self.setRenderHint(QPainter.SmoothPixmapTransform, False)
        self.setBackgroundBrush(QColor(232, 232, 230))   # "no map here" color
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
            lon, lat = -69.9, 18.5     # Santo Domingo fallback
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
            self.zoom_changed.emit(self.z)

    def _apply_zoom(self):
        world = TILE << self.z
        self._scene.setSceneRect(QRectF(0, 0, world, world))

    def wheelEvent(self, event: QWheelEvent):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        old_z = self.z
        anchor = self.mapToScene(event.position().toPoint())   # keep under cursor
        self.set_zoom(old_z + (1 if delta > 0 else -1))
        if self.z != old_z:
            factor = float(2 ** (self.z - old_z))
            self.centerOn(anchor.x() * factor, anchor.y() * factor)
            self.viewport().update()
            print("zoom -> z%d" % self.z)

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
                    # placeholder while the worker fetches it
                    painter.fillRect(tx * TILE, ty * TILE, TILE, TILE,
                                     QColor(232, 232, 230))
                    if self.worker and (z, tx, ty) not in self.requested:
                        self.requested.add((z, tx, ty))
                        self.worker.request(z, tx, ty)

        # cache eviction (oldest first)
        while len(self.cache) > MAX_CACHE:
            self.cache.pop(next(iter(self.cache)))

    def drawForeground(self, painter, rect):
        # red crosshair fixed at the center of the viewport
        painter.save()
        painter.resetTransform()
        c = self.viewport().rect().center()
        painter.setPen(QPen(QColor(210, 40, 40), 1.5))
        painter.drawEllipse(c, 12, 12)
        painter.drawLine(c.x() - 22, c.y(), c.x() + 22, c.y())
        painter.drawLine(c.x(), c.y() - 22, c.x(), c.y() + 22)
        painter.restore()

    # ---- mouse coords ----
    def mouseMoveEvent(self, event: QMouseEvent):
        sp = self.mapToScene(event.position().toPoint())
        lon, lat = global_px_to_lonlat(sp.x(), sp.y(), self.z)
        self.coords_changed.emit(lon, lat)
        super().mouseMoveEvent(event)


# ---------------- Main window ----------------
class MapWindow(QWidget):
    def __init__(self):
        super().__init__()

        # ---- Window setup ----
        self.setWindowTitle("LSD - Offline MBTiles Map Viewer")
        self.resize(1100, 800)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        # ---- Button row ----
        btn_row = QHBoxLayout()

        self.btn_open = QPushButton("Open Map...")
        self.btn_open.clicked.connect(self.on_open)
        btn_row.addWidget(self.btn_open)

        self.btn_zoom_in = QPushButton("Zoom +")
        self.btn_zoom_in.clicked.connect(lambda: self.map.set_zoom(self.map.z + 1))
        btn_row.addWidget(self.btn_zoom_in)

        self.btn_zoom_out = QPushButton("Zoom -")
        self.btn_zoom_out.clicked.connect(lambda: self.map.set_zoom(self.map.z - 1))
        btn_row.addWidget(self.btn_zoom_out)

        self.btn_home = QPushButton("Home")
        self.btn_home.clicked.connect(self.on_home)
        btn_row.addWidget(self.btn_home)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)

        # ---- Map ----
        self.map = MapView()
        self.map.coords_changed.connect(self.on_coords)
        self.map.zoom_changed.connect(self.on_zoom)
        layout.addWidget(self.map)

        # ---- Status label ----
        self.status = QLabel("Open a .mbtiles file to begin")
        layout.addWidget(self.status)

        self.db_path = None

    # ---- Button callbacks ----
    def on_open(self):
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
        self.on_zoom(self.map.z)
        print("opened:", path)

    def on_home(self):
        self.map.go_home()

    def on_coords(self, lon, lat):
        self.status.setText("%s | z%d | cursor: %.5f, %.5f"
                            % (os.path.basename(self.db_path) if self.db_path else "-",
                               self.map.z, lat, lon))

    def on_zoom(self, z):
        self.on_coords(0.0, 0.0)

    def closeEvent(self, event):
        if self.map.worker:
            self.map.worker.stop()
        event.accept()


# ---------------- Main ----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MapWindow()
    win.show()
    sys.exit(app.exec())
