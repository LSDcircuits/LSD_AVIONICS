import csv
import re
import sys
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# Accepts lines such as:
# 16:09:26  Lat 19.7002715N  Lon 71.4379715W  Spd 0.3 km/h ...
# Height is optional in the incoming text and supports common labels:
# Height 123.4m, Hgt=123.4m, Alt: 123.4m, AltMSL 123.4m
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


def parse_gps_line(line):
    """Return time, latitude, longitude, and height from one serial line."""
    lat_match = LAT_RE.search(line)
    lon_match = LON_RE.search(line)

    # Ignore status/debug lines that do not contain a coordinate.
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

    height = height_match.group("value") if height_match else ""

    return {
        "time": time_match.group("time") if time_match else datetime.now().strftime("%H:%M:%S"),
        "latitude": f"{latitude:.7f}",
        "longitude": f"{longitude:.7f}",
        "height": height,
    }


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


class GpsLogger(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PICO M9N GPS Logger")
        self.resize(760, 540)

        self.reader = None
        self.log_file = None
        self.csv_writer = None
        self.logging = False
        self.rows_logged = 0
        self.missing_height_warned = False

        self.init_ui()
        self.refresh_ports()
        self.update_status("Select a port and press Connect.")

    def init_ui(self):
        layout = QVBoxLayout(self)

        # Serial settings
        settings_row = QHBoxLayout()
        settings_row.addWidget(QLabel("Serial Port:"))

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        settings_row.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        settings_row.addWidget(self.refresh_btn)

        settings_row.addSpacing(16)
        settings_row.addWidget(QLabel("Baud:"))

        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["9600", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        settings_row.addWidget(self.baud_combo)

        settings_row.addStretch()
        layout.addLayout(settings_row)

        # Connection controls
        connection_row = QHBoxLayout()

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_serial)
        connection_row.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)
        connection_row.addWidget(self.disconnect_btn)

        connection_row.addSpacing(24)

        self.start_log_btn = QPushButton("Start Logging")
        self.start_log_btn.clicked.connect(self.start_logging)
        self.start_log_btn.setEnabled(False)
        connection_row.addWidget(self.start_log_btn)

        self.stop_log_btn = QPushButton("Stop Logging")
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.stop_log_btn.setEnabled(False)
        connection_row.addWidget(self.stop_log_btn)

        connection_row.addStretch()
        layout.addLayout(connection_row)

        self.status_label = QLabel()
        layout.addWidget(self.status_label)

        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.document().setMaximumBlockCount(5000)
        layout.addWidget(self.output)

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
        self.update_status("Connected. Press Start Logging to choose a CSV file.")

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
        self.disconnect_btn.setEnabled(True)

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

        if not self.logging or not self.csv_writer:
            return

        parsed = parse_gps_line(line)
        if not parsed:
            return

        self.csv_writer.writerow(
            [parsed["time"], parsed["latitude"], parsed["longitude"], parsed["height"]]
        )
        self.log_file.flush()
        self.rows_logged += 1

        if not parsed["height"] and not self.missing_height_warned:
            self.missing_height_warned = True
            self.append_message(
                "[NOTE] Coordinates are being logged, but no height field was found in the serial text."
            )

        self.update_status(
            f"Logging to {Path(self.log_file.name).name} — {self.rows_logged} rows"
        )

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
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GpsLogger()
    window.show()
    sys.exit(app.exec())
