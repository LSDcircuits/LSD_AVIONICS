import csv
import html
import json
import math
import re
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QUrl, Qt, Signal
from PySide6.QtWebEngineCore import QWebEngineSettings
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


# Requires: pip install PySide6
# The map background uses OpenStreetMap tiles, so internet access is required
# for the street map. The route itself is generated locally from the CSV.

LATITUDE_NAMES = {"lat", "latitude", "latitudedeg", "latitude_deg"}
LONGITUDE_NAMES = {"lon", "lng", "long", "longitude", "longitudedeg", "longitude_deg"}
TIME_NAMES = {"time", "timestamp", "gpstime", "gps_time", "devicetime", "device_time"}
HEIGHT_NAMES = {"height", "heightm", "height_m", "alt", "altitude", "altitudem", "altitude_m"}
NUMBER_RE = re.compile(r"[+-]?\d+(?:\.\d+)?")


@dataclass
class GpsPoint:
    latitude: float
    longitude: float
    time: str = ""
    height: float | None = None


@dataclass
class CsvResult:
    points: list[GpsPoint]
    skipped_rows: int
    latitude_column: str
    longitude_column: str


def normalize_header(name):
    return re.sub(r"[^a-z0-9]", "", str(name).strip().lower())


def find_column(fieldnames, accepted_names):
    normalized = {normalize_header(name): name for name in fieldnames or []}
    for accepted in accepted_names:
        normalized_name = normalize_header(accepted)
        if normalized_name in normalized:
            return normalized[normalized_name]
    return None


def parse_number(value):
    if value is None:
        return None

    match = NUMBER_RE.search(str(value).strip())
    if not match:
        return None

    try:
        return float(match.group(0))
    except ValueError:
        return None


def parse_coordinate(value, coordinate_type):
    number = parse_number(value)
    if number is None:
        return None

    text = str(value).strip().upper()

    if coordinate_type == "latitude":
        if "N" in text or "S" in text:
            number = abs(number)
            if "S" in text:
                number = -number
        if not -90 <= number <= 90:
            return None
    else:
        if "E" in text or "W" in text:
            number = abs(number)
            if "W" in text:
                number = -number
        if not -180 <= number <= 180:
            return None

    return number


def read_gps_csv(filename):
    with open(filename, "r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []

        latitude_column = find_column(fieldnames, LATITUDE_NAMES)
        longitude_column = find_column(fieldnames, LONGITUDE_NAMES)
        time_column = find_column(fieldnames, TIME_NAMES)
        height_column = find_column(fieldnames, HEIGHT_NAMES)

        if not latitude_column or not longitude_column:
            raise ValueError(
                "Could not find latitude and longitude columns. Expected names such as "
                "latitude_deg and longitude_deg."
            )

        points = []
        skipped_rows = 0

        for row in reader:
            latitude = parse_coordinate(row.get(latitude_column), "latitude")
            longitude = parse_coordinate(row.get(longitude_column), "longitude")

            if latitude is None or longitude is None:
                skipped_rows += 1
                continue

            height = parse_number(row.get(height_column)) if height_column else None
            points.append(
                GpsPoint(
                    latitude=latitude,
                    longitude=longitude,
                    time=(row.get(time_column) or "").strip() if time_column else "",
                    height=height,
                )
            )

    return CsvResult(points, skipped_rows, latitude_column, longitude_column)


def haversine_meters(lat1, lon1, lat2, lon2):
    earth_radius = 6_371_000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    )
    return 2 * earth_radius * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def route_distance_meters(points):
    return sum(
        haversine_meters(
            first.latitude,
            first.longitude,
            second.latitude,
            second.longitude,
        )
        for first, second in zip(points, points[1:])
    )


def format_distance(distance_meters):
    if distance_meters >= 1000:
        return f"{distance_meters / 1000:.2f} km"
    return f"{distance_meters:.0f} m"


def format_height(value):
    if value is None:
        return "--"
    return f"{value:.1f} m"


def make_map_html(points):
    point_data = [
        {
            "lat": point.latitude,
            "lng": point.longitude,
            "time": point.time,
            "height": point.height,
        }
        for point in points
    ]
    points_json = json.dumps(point_data, separators=(",", ":")).replace("</", "<\\/")

    return f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link
        rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    >
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        html, body, #map {{
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
            background: #17202a;
            font-family: Arial, sans-serif;
        }}
        .map-note {{
            position: absolute;
            right: 12px;
            top: 12px;
            z-index: 1000;
            padding: 8px 10px;
            border-radius: 6px;
            color: #ffffff;
            background: rgba(20, 28, 38, 0.82);
            font-size: 12px;
            pointer-events: none;
        }}
        .leaflet-popup-content {{
            font-size: 13px;
            line-height: 1.45;
        }}
    </style>
</head>
<body>
    <div id="map"></div>
    <div class="map-note">OpenStreetMap basemap requires internet</div>
    <script>
        const points = {points_json};
        const map = L.map("map", {{
            zoomControl: true,
            preferCanvas: true,
            worldCopyJump: true
        }});

        L.tileLayer("https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png", {{
            maxZoom: 19,
            attribution: "&copy; OpenStreetMap contributors"
        }}).addTo(map);

        L.control.scale({{ imperial: false }}).addTo(map);

        const latLngs = points.map(point => [point.lat, point.lng]);
        let routeBounds = null;

        function pointPopup(point, title) {{
            const heightText = point.height === null ? "--" : Number(point.height).toFixed(1) + " m";
            return `<b>${{title}}</b><br>` +
                   `Time: ${{point.time || "--"}}<br>` +
                   `Lat: ${{point.lat.toFixed(7)}}<br>` +
                   `Lon: ${{point.lng.toFixed(7)}}<br>` +
                   `Height: ${{heightText}}`;
        }}

        if (latLngs.length > 0) {{
            routeBounds = L.latLngBounds(latLngs);

            if (latLngs.length === 1) {{
                L.circleMarker(latLngs[0], {{
                    radius: 7,
                    color: "#ffffff",
                    weight: 2,
                    fillColor: "#22c55e",
                    fillOpacity: 1
                }}).addTo(map).bindPopup(pointPopup(points[0], "Point"));
            }} else {{
                L.polyline(latLngs, {{
                    color: "#00a6ff",
                    weight: 4,
                    opacity: 0.9,
                    lineJoin: "round"
                }}).addTo(map);

                L.circleMarker(latLngs[0], {{
                    radius: 7,
                    color: "#ffffff",
                    weight: 2,
                    fillColor: "#22c55e",
                    fillOpacity: 1
                }}).addTo(map).bindPopup(pointPopup(points[0], "Start"));

                L.circleMarker(latLngs[latLngs.length - 1], {{
                    radius: 7,
                    color: "#ffffff",
                    weight: 2,
                    fillColor: "#ef4444",
                    fillOpacity: 1
                }}).addTo(map).bindPopup(pointPopup(points[points.length - 1], "End"));
            }}
        }}

        function fitRoute() {{
            if (!routeBounds) {{
                return;
            }}

            if (latLngs.length === 1) {{
                map.setView(latLngs[0], 17);
            }} else {{
                map.fitBounds(routeBounds, {{ padding: [30, 30] }});
            }}
        }}

        fitRoute();
    </script>
</body>
</html>"""


class MapView(QWebEngineView):
    file_dropped = Signal(str)

    def __init__(self):
        super().__init__()
        self.setAcceptDrops(True)
        self.settings().setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        self.settings().setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,
            True,
        )

    def dragEnterEvent(self, event):
        if self.has_csv_url(event.mimeData()):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event):
        if self.has_csv_url(event.mimeData()):
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            filename = url.toLocalFile()
            if filename.lower().endswith(".csv"):
                self.file_dropped.emit(filename)
                event.acceptProposedAction()
                return
        super().dropEvent(event)

    @staticmethod
    def has_csv_url(mime_data):
        return any(
            url.isLocalFile() and url.toLocalFile().lower().endswith(".csv")
            for url in mime_data.urls()
        )


class GpsMapViewer(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PICO M9N GPS Map Viewer")
        self.resize(1100, 760)
        self.setAcceptDrops(True)

        self.current_file = None
        self.points = []
        self.skipped_rows = 0
        self.map_files = []

        self.init_ui()
        self.show_empty_map()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        controls = QHBoxLayout()

        self.open_button = QPushButton("Open CSV")
        self.open_button.clicked.connect(self.choose_file)
        controls.addWidget(self.open_button)

        self.fit_button = QPushButton("Fit Route")
        self.fit_button.clicked.connect(self.fit_route)
        self.fit_button.setEnabled(False)
        controls.addWidget(self.fit_button)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_route)
        self.clear_button.setEnabled(False)
        controls.addWidget(self.clear_button)

        self.file_label = QLabel("Drag and drop a GPS CSV file onto the map")
        self.file_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        controls.addWidget(self.file_label, 1)

        layout.addLayout(controls)

        self.stats_label = QLabel("Expected columns: latitude_deg, longitude_deg; time and height_m are optional.")
        layout.addWidget(self.stats_label)

        self.map_view = MapView()
        self.map_view.file_dropped.connect(self.load_csv)
        layout.addWidget(self.map_view, 1)

    def choose_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open GPS CSV",
            str(Path.home()),
            "CSV Files (*.csv);;All Files (*)",
        )
        if filename:
            self.load_csv(filename)

    def load_csv(self, filename):
        try:
            result = read_gps_csv(filename)
        except (OSError, csv.Error, ValueError) as exc:
            QMessageBox.critical(self, "Cannot load CSV", str(exc))
            return

        if not result.points:
            QMessageBox.warning(
                self,
                "No valid coordinates",
                "The CSV was opened, but no valid latitude/longitude rows were found.",
            )
            return

        self.current_file = filename
        self.points = result.points
        self.skipped_rows = result.skipped_rows

        self.file_label.setText(Path(filename).name)
        self.update_stats()
        self.render_map()

        self.fit_button.setEnabled(True)
        self.clear_button.setEnabled(True)

    def update_stats(self):
        distance = route_distance_meters(self.points)
        start = self.points[0]
        end = self.points[-1]

        heights = [point.height for point in self.points if point.height is not None]
        height_text = ""
        if heights:
            height_text = f" • Height {min(heights):.1f} to {max(heights):.1f} m"

        skipped_text = ""
        if self.skipped_rows:
            skipped_text = f" • {self.skipped_rows} skipped"

        self.stats_label.setText(
            f"{len(self.points):,} points • Distance {format_distance(distance)}"
            f" • Start {start.time or '--'} • End {end.time or '--'}"
            f"{height_text}{skipped_text}"
        )

    def render_map(self):
        map_html = make_map_html(self.points)
        temporary = tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            suffix=".html",
            prefix="pico_gps_map_",
            delete=False,
        )

        with temporary:
            temporary.write(map_html)

        self.map_files.append(temporary.name)
        self.map_view.load(QUrl.fromLocalFile(temporary.name))

    def fit_route(self):
        self.map_view.page().runJavaScript("fitRoute();")

    def clear_route(self):
        self.current_file = None
        self.points = []
        self.skipped_rows = 0
        self.file_label.setText("Drag and drop a GPS CSV file onto the map")
        self.stats_label.setText(
            "Expected columns: latitude_deg, longitude_deg; time and height_m are optional."
        )
        self.fit_button.setEnabled(False)
        self.clear_button.setEnabled(False)
        self.show_empty_map()

    def show_empty_map(self):
        message = html.escape("Drop a CSV file here or press Open CSV")
        empty_html = f"""<!DOCTYPE html>
<html>
<body style="margin:0;background:#17202a;color:#d7dde8;display:grid;place-items:center;height:100vh;font-family:Arial,sans-serif;">
    <div style="text-align:center">
        <div style="font-size:28px;font-weight:600;margin-bottom:10px">GPS Map Viewer</div>
        <div style="font-size:15px;color:#9aa8ba">{message}</div>
    </div>
</body>
</html>"""
        self.map_view.setHtml(empty_html)

    def dragEnterEvent(self, event):
        if MapView.has_csv_url(event.mimeData()):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            filename = url.toLocalFile()
            if filename.lower().endswith(".csv"):
                self.load_csv(filename)
                event.acceptProposedAction()
                return
        super().dropEvent(event)

    def closeEvent(self, event):
        for filename in self.map_files:
            try:
                Path(filename).unlink(missing_ok=True)
            except OSError:
                pass
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GpsMapViewer()
    window.show()
    sys.exit(app.exec())
