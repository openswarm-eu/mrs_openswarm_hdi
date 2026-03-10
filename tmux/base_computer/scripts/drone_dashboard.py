#!/usr/bin/env python3

import sys
import time
import threading
import subprocess
import re
import socket
import shutil
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

import rospy
from std_msgs.msg import Float32, String, UInt8MultiArray
from sensor_msgs.msg import NavSatFix

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFrame,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
)


DRONE_IDS = [f"uav{i}" for i in range(7, 16)]
TOPIC_SUFFIX = {
    "altitude": "/drone/altitude",
    "flight_mode": "/drone/flight_mode",
    "gps": "/drone/gps",
}
STATUS_TOPIC_SUFFIX = "/mrs_uav_status/uav_status"
FMS_STATUS_TOPIC_SUFFIX = "/fms/drone_status"
# Prefer strongly-typed status message when available.
try:
    from mrs_msgs.msg import UavStatus as StatusMsgType
except Exception:
    StatusMsgType = rospy.AnyMsg
# Keep this mapping aligned with the operational ping scripts.
DRONE_PING_IP = {
    "uav7": "192.168.194.188",
    "uav8": "192.168.194.254",
    "uav9": "192.168.194.228",
    "uav10": "192.168.194.190",
    "uav11": "192.168.194.160",
    "uav12": "192.168.194.137",
    "uav13": "192.168.194.144",
    "uav14": "192.168.194.146",
    "uav15": "",
}
PING_RE = re.compile(r"time=([0-9.]+)\s*ms")
GNSS_FIX_TYPE_TEXT = {
    0: "0 No GPS",
    1: "1 Connected, no pos",
    2: "2 2D",
    3: "3 3D",
    4: "4 DGPS/SBAS 3D",
    5: "5 RTK float",
    6: "6 RTK fixed",
    7: "7 Static fixed",
    8: "8 PPP 3D",
}


def parse_active_drones_from_env():
    raw = os.environ.get("UAV_NAMES", "")
    if not raw.strip():
        return set(DRONE_IDS)

    # Prefer explicit uavXX matches, fallback to token split.
    matches = set(re.findall(r"uav\d+", raw.lower()))
    if matches:
        return set(drone_id for drone_id in DRONE_IDS if drone_id in matches)

    tokens = [token.strip().lower() for token in re.split(r"[,\s;]+", raw) if token.strip()]
    return set(drone_id for drone_id in DRONE_IDS if drone_id in tokens)


class DroneState:
    def __init__(self):
        self.battery_voltage_v = None
        self.altitude_m = 0.0
        self.flight_mode = "UNKNOWN"
        self.lidar_3d_ok = None
        self.swarm_info_ok = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.status_msg_time = 0.0
        self.last_msg_time = 0.0
        self.hw_api_gnss_fix_type = None
        self.ping_status = "N/A"
        self.ping_latency_ms = None
        self.ping_last_check = 0.0

    def heartbeat(self):
        self.last_msg_time = time.time()

    def status_heartbeat(self):
        self.status_msg_time = time.time()


class FleetData:
    def __init__(self, drone_ids):
        self.lock = threading.Lock()
        self.drone_ids = list(drone_ids)
        self.states = {drone_id: DroneState() for drone_id in self.drone_ids}

    def snapshot(self):
        with self.lock:
            return {
                drone_id: {
                    "battery_voltage_v": state.battery_voltage_v,
                    "altitude_m": state.altitude_m,
                    "flight_mode": state.flight_mode,
                    "lidar_3d_ok": state.lidar_3d_ok,
                    "swarm_info_ok": state.swarm_info_ok,
                    "latitude": state.latitude,
                    "longitude": state.longitude,
                    "status_msg_time": state.status_msg_time,
                    "last_msg_time": state.last_msg_time,
                    "hw_api_gnss_fix_type": state.hw_api_gnss_fix_type,
                    "ping_status": state.ping_status,
                    "ping_latency_ms": state.ping_latency_ms,
                    "ping_last_check": state.ping_last_check,
                }
                for drone_id, state in self.states.items()
            }


class PingMonitor:
    def __init__(self, data, ip_map, interval_s=2.0):
        self.data = data
        self.ip_map = dict(ip_map)
        self.interval_s = interval_s
        self.ping_exe = shutil.which("ping")
        self.executor = ThreadPoolExecutor(max_workers=len(DRONE_IDS))
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _tcp_probe(self, ip, port=22):
        t0 = time.time()
        try:
            with socket.create_connection((ip, port), timeout=1.0):
                latency_ms = (time.time() - t0) * 1000.0
                return "TCP", latency_ms
        except ConnectionRefusedError:
            # Host reachable but service closed.
            latency_ms = (time.time() - t0) * 1000.0
            return "TCP", latency_ms
        except socket.timeout:
            return "TCP TO", None
        except OSError:
            return "TCP ERR", None

    def _ping_once(self, ip):
        if not ip:
            return "NO IP", None
        if not self.ping_exe:
            return self._tcp_probe(ip)
        # Try common Linux ping syntaxes to handle distro/toolbox differences.
        ping_cmds = (
            [self.ping_exe, "-c", "1", "-W", "1", ip],
            [self.ping_exe, "-c", "1", "-w", "1", ip],
            [self.ping_exe, "-c", "1", ip],
        )
        last_result = None
        try:
            for cmd in ping_cmds:
                result = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2.0,
                    check=False,
                )
                last_result = result
                # Unsupported option output usually contains "invalid option".
                invalid_opt = "invalid option" in (result.stderr or "").lower()
                if not invalid_opt:
                    break
        except (subprocess.SubprocessError, OSError):
            return self._tcp_probe(ip)

        if last_result is None:
            return self._tcp_probe(ip)

        if last_result.returncode != 0:
            stderr = (last_result.stderr or "").lower()
            if "operation not permitted" in stderr:
                return self._tcp_probe(ip)
            if "name or service not known" in stderr or "temporary failure in name resolution" in stderr:
                return "DNS", None
            if "network is unreachable" in stderr:
                return "UNREACH", None
            return "TIMEOUT", None

        match = PING_RE.search(last_result.stdout)
        if match:
            return "OK", float(match.group(1))
        return "OK", None

    def _run(self):
        while not rospy.is_shutdown():
            futures = {
                self.executor.submit(self._ping_once, self.ip_map.get(drone_id, "")): drone_id
                for drone_id in DRONE_IDS
            }
            now = time.time()
            for future in as_completed(futures):
                drone_id = futures[future]
                try:
                    status, latency_ms = future.result()
                except Exception:
                    status, latency_ms = "ERROR", None
                with self.data.lock:
                    state = self.data.states[drone_id]
                    state.ping_status = status
                    state.ping_latency_ms = latency_ms
                    state.ping_last_check = now
            time.sleep(self.interval_s)


class DroneDashboard(QMainWindow):
    def __init__(self, data):
        super().__init__()
        self.data = data
        self.selected_drone = DRONE_IDS[0]
        self.active_drones = parse_active_drones_from_env()
        self.setWindowTitle("Fleet Telemetry Dashboard")
        self.setMinimumSize(950, 620)

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout()
        button_layout = QHBoxLayout()
        grid = QGridLayout()
        central.setLayout(main_layout)

        self.status_label = QLabel(f"{self.selected_drone} Status: DISCONNECTED")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: white; background-color: red; padding: 8px;"
        )

        self.drone_buttons = {}
        for drone_id in DRONE_IDS:
            button = QPushButton(drone_id.upper())
            button.setCheckable(True)
            button.clicked.connect(lambda checked, d=drone_id: self.select_drone(d))
            self.drone_buttons[drone_id] = button
            button_layout.addWidget(button)
        self.drone_buttons[self.selected_drone].setChecked(True)
        self._update_button_styles()

        self.battery_label = QLabel("Battery: N/A")
        self.altitude_label = QLabel("Altitude: 0.00 m")
        self.flight_mode_label = QLabel("Flight Mode: UNKNOWN")
        self.gps_label = QLabel("GPS: 0.000000, 0.000000")
        self.link_label = QLabel("Link: no data")

        for widget in [self.battery_label, self.altitude_label, self.flight_mode_label, self.gps_label, self.link_label]:
            widget.setFrameStyle(QFrame.Panel | QFrame.Sunken)
            widget.setStyleSheet("font-size: 16px; padding: 6px;")

        grid.addWidget(self.battery_label, 0, 0, 1, 2)
        grid.addWidget(self.altitude_label, 1, 0)
        grid.addWidget(self.flight_mode_label, 1, 1)
        grid.addWidget(self.gps_label, 2, 0, 1, 2)
        grid.addWidget(self.link_label, 3, 0, 1, 2)

        self.table = QTableWidget(len(DRONE_IDS), 8)
        self.table.setHorizontalHeaderLabels(
            ["Drone", "Ping", "Status", "Battery (V)", "Swarm Info", "Lidar 3D", "GPS", "GNSS Fix"]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        for row, drone_id in enumerate(DRONE_IDS):
            self.table.setItem(row, 0, QTableWidgetItem(drone_id))

        main_layout.addWidget(self.status_label)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(grid)
        main_layout.addWidget(self.table)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(200)  # refresh every 200 ms

    def select_drone(self, drone_id):
        self.selected_drone = drone_id
        for current_id, button in self.drone_buttons.items():
            button.setChecked(current_id == drone_id)
        self._update_button_styles()
        self.refresh_ui()

    def _update_button_styles(self):
        for drone_id, button in self.drone_buttons.items():
            is_active = drone_id in self.active_drones
            color = "black" if is_active else "red"
            if button.isChecked():
                button.setStyleSheet(f"font-weight: bold; color: {color}; border: 2px solid #4b6cb7;")
            else:
                button.setStyleSheet(f"font-weight: bold; color: {color};")

    def refresh_ui(self):
        snapshot = self.data.snapshot()
        now = time.time()

        selected = snapshot[self.selected_drone]
        status_age = now - selected["status_msg_time"] if selected["status_msg_time"] > 0 else 999.0
        connected = status_age < 2.0

        selected_battery_v = selected["battery_voltage_v"]
        selected_gnss_fix = selected["hw_api_gnss_fix_type"]
        selected_gnss_text = GNSS_FIX_TYPE_TEXT.get(selected_gnss_fix, "N/A")
        if selected_battery_v is None:
            self.battery_label.setText("Battery: N/A")
        else:
            self.battery_label.setText(f"Battery: {selected_battery_v:.2f} V")
        self.altitude_label.setText(f"Altitude: {selected['altitude_m']:.2f} m")
        self.flight_mode_label.setText(f"Flight Mode: {selected['flight_mode']}")
        self.gps_label.setText(f"GPS: {selected['latitude']:.6f}, {selected['longitude']:.6f}")
        self.link_label.setText(f"GNSS Fix: {selected_gnss_text} | Status age: {status_age:.2f} s")

        if connected:
            self.status_label.setText(f"{self.selected_drone} Status: CONNECTED")
            self.status_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: white; background-color: green; padding: 8px;"
            )
        else:
            self.status_label.setText(f"{self.selected_drone} Status: DISCONNECTED")
            self.status_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: white; background-color: red; padding: 8px;"
            )

        for row, drone_id in enumerate(DRONE_IDS):
            name_item = QTableWidgetItem(drone_id)
            if drone_id not in self.active_drones:
                name_item.setForeground(QColor("red"))
            self.table.setItem(row, 0, name_item)

            state = snapshot[drone_id]
            row_status_age = now - state["status_msg_time"] if state["status_msg_time"] > 0 else 999.0
            row_connected = row_status_age < 2.0
            status_text = "CONNECTED" if row_connected else "DISCONNECTED"
            ping_status = state["ping_status"]
            if state["ping_latency_ms"] is not None:
                ping_status = f"{ping_status} {state['ping_latency_ms']:.1f} ms"
            battery_text = "N/A"
            if state["battery_voltage_v"] is not None:
                battery_text = f"{state['battery_voltage_v']:.2f}"
            gnss_fix_text = GNSS_FIX_TYPE_TEXT.get(state["hw_api_gnss_fix_type"], "N/A")
            lidar_3d_text = "N/A"
            if state["lidar_3d_ok"] is True:
                lidar_3d_text = "VERIFIED"
            elif state["lidar_3d_ok"] is False:
                lidar_3d_text = "NOT VERIFIED"
            swarm_info_text = "N/A"
            if state["swarm_info_ok"] is True:
                swarm_info_text = "OK"
            elif state["swarm_info_ok"] is False:
                swarm_info_text = "NOT OK"

            self.table.setItem(row, 1, QTableWidgetItem(ping_status))
            self.table.setItem(row, 2, QTableWidgetItem(status_text))
            self.table.setItem(row, 3, QTableWidgetItem(battery_text))
            self.table.setItem(row, 4, QTableWidgetItem(swarm_info_text))
            self.table.setItem(row, 5, QTableWidgetItem(lidar_3d_text))
            self.table.setItem(
                row,
                6,
                QTableWidgetItem(f"{state['latitude']:.6f}, {state['longitude']:.6f}"),
            )
            self.table.setItem(row, 7, QTableWidgetItem(gnss_fix_text))


class RosInterface:
    def __init__(self, data):
        self.data = data
        self.subscribers = []
        for drone_id in DRONE_IDS:
            ns_prefix = f"/{drone_id}"
            self.subscribers.append(
                rospy.Subscriber(
                    ns_prefix + TOPIC_SUFFIX["altitude"],
                    Float32,
                    self.altitude_cb,
                    callback_args=drone_id,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    ns_prefix + TOPIC_SUFFIX["flight_mode"],
                    String,
                    self.flight_mode_cb,
                    callback_args=drone_id,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    ns_prefix + TOPIC_SUFFIX["gps"],
                    NavSatFix,
                    self.gps_cb,
                    callback_args=drone_id,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    ns_prefix + STATUS_TOPIC_SUFFIX,
                    StatusMsgType,
                    self.status_cb,
                    callback_args=drone_id,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    ns_prefix + FMS_STATUS_TOPIC_SUFFIX,
                    UInt8MultiArray,
                    self.fms_status_cb,
                    callback_args=drone_id,
                )
            )

    def altitude_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            state.altitude_m = msg.data
            state.heartbeat()

    def flight_mode_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            state.flight_mode = msg.data
            state.heartbeat()

    def gps_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            state.latitude = msg.latitude
            state.longitude = msg.longitude
            state.heartbeat()

    def status_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            state.status_heartbeat()
            for field in ("battery_volt", "battery_voltage", "voltage"):
                value = getattr(msg, field, None)
                if isinstance(value, (float, int)):
                    state.battery_voltage_v = float(value)
                    break
            for field in ("hw_api_gnss_fix_type", "gnss_fix_type"):
                value = getattr(msg, field, None)
                if isinstance(value, int):
                    state.hw_api_gnss_fix_type = value
                    break

    def fms_status_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            # Index 1 = LiDAR 3D verification flag in fms/drone_status.
            if hasattr(msg, "data") and len(msg.data) > 1:
                state.lidar_3d_ok = bool(msg.data[1])
            # Index 2 = Swarm Info verification flag in fms/drone_status.
            if hasattr(msg, "data") and len(msg.data) > 2:
                state.swarm_info_ok = bool(msg.data[2])


def main():
    rospy.init_node("drone_dashboard", anonymous=True)

    data = FleetData(DRONE_IDS)
    ros_interface = RosInterface(data)
    ping_monitor = PingMonitor(data, DRONE_PING_IP)

    app = QApplication(sys.argv)
    window = DroneDashboard(data)
    window.show()

    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
