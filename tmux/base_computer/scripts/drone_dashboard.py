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
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger

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


DRONE_IDS = [f"uav{i}" for i in range(6, 16)]
TOPIC_SUFFIX = {
    "flight_mode": "/drone/flight_mode",
}
DIAGNOSTICS_TOPIC_SUFFIX = "/uav_manager/diagnostics"
STATUS_TOPIC_SUFFIX = "/mrs_uav_status/uav_status"
FMS_STATUS_TOPIC_SUFFIX = "/fms/drone_status"
# Prefer strongly-typed status message when available.
try:
    from mrs_msgs.msg import UavStatus as StatusMsgType
except Exception:
    StatusMsgType = rospy.AnyMsg
try:
    from mrs_msgs.msg import UavManagerDiagnostics as DiagnosticsMsgType
except Exception:
    DiagnosticsMsgType = rospy.AnyMsg
try:
    from mrs_msgs.srv import Vec1 as Vec1SrvType
    from mrs_msgs.srv import Vec1Request as Vec1RequestType
except Exception:
    Vec1SrvType = None
    Vec1RequestType = None
# Keep this mapping aligned with the operational ping scripts.
DRONE_PING_IP = {
    "uav6": "192.168.194.170",
    "uav7": "192.168.194.188",
    "uav8": "192.168.194.254",
    "uav9": "192.168.194.228",
    "uav10": "192.168.194.190",
    "uav11": "192.168.194.160",
    "uav12": "192.168.194.137",
    "uav13": "192.168.194.144",
    "uav14": "192.168.194.203",
    "uav15": "192.168.194.146",
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


def parse_ordered_drones_from_env():
    raw = os.environ.get("UAV_NAMES", "")
    if not raw.strip():
        return list(DRONE_IDS)

    ordered = []
    seen = set()
    for token in re.findall(r"uav\d+", raw.lower()):
        if token in DRONE_IDS and token not in seen:
            ordered.append(token)
            seen.add(token)
    if ordered:
        return ordered

    for token in re.split(r"[,\s;]+", raw):
        token = token.strip().lower()
        if token in DRONE_IDS and token not in seen:
            ordered.append(token)
            seen.add(token)
    return ordered if ordered else list(DRONE_IDS)


def parse_active_drones_from_env():
    return set(parse_ordered_drones_from_env())


class DroneState:
    def __init__(self):
        self.battery_voltage_v = None
        self.flight_time_s = None
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
        self.drone_status_gnss = None
        self.drone_status_lidar = None
        self.drone_status_swarm = None
        self.drone_status_pre_flight = None
        self.drone_status_armed = None
        self.drone_status_offboard = None

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
                    "flight_time_s": state.flight_time_s,
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
                    "drone_status_gnss": state.drone_status_gnss,
                    "drone_status_lidar": state.drone_status_lidar,
                    "drone_status_swarm": state.drone_status_swarm,
                    "drone_status_pre_flight": state.drone_status_pre_flight,
                    "drone_status_armed": state.drone_status_armed,
                    "drone_status_offboard": state.drone_status_offboard,
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
        self.scan_lock = threading.Lock()
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

    def _scan_once(self):
        with self.scan_lock:
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

    def refresh_now(self):
        self._scan_once()

    def _run(self):
        while not rospy.is_shutdown():
            self._scan_once()
            time.sleep(self.interval_s)


class TakeoffController:
    def __init__(self, drone_ids):
        self.drone_ids = list(drone_ids)
        self.lock = threading.Lock()
        self.takeoff_services = {}
        self.emergency_services = {}
        self.swarm_start_service = None
        self.swarm_leader = None
        self.discovery_threads = []

        for drone_id in self.drone_ids:
            takeoff_thread = threading.Thread(
                target=self._discover_service,
                args=(drone_id, "/uav_manager/takeoff", self.takeoff_services, "take-off"),
                daemon=True,
            )
            emergency_thread = threading.Thread(
                target=self._discover_service,
                args=(drone_id, "/sweeping_generator/emergency", self.emergency_services, "emergency"),
                daemon=True,
            )
            takeoff_thread.start()
            emergency_thread.start()
            self.discovery_threads.append(takeoff_thread)
            self.discovery_threads.append(emergency_thread)

    def _discover_service(self, drone_id, suffix, service_map, label):
        service_name = f"/{drone_id}{suffix}"
        rospy.loginfo(f"[DASHBOARD]: Waiting for {label} service: {service_name}")
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
                proxy = rospy.ServiceProxy(service_name, Trigger, persistent=False)
                with self.lock:
                    service_map[drone_id] = proxy
                rospy.loginfo(f"[DASHBOARD]: {label.capitalize()} service ready: {service_name}")
                return
            except rospy.ROSException:
                continue
            except Exception as exc:
                rospy.logwarn(f"[DASHBOARD]: Service discovery error for {service_name}: {exc}")
                time.sleep(0.5)

    def _refresh_trigger_service(self, drone_id, suffix, service_map):
        service_name = f"/{drone_id}{suffix}"
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
            proxy = rospy.ServiceProxy(service_name, Trigger, persistent=False)
            with self.lock:
                service_map[drone_id] = proxy
            return True
        except rospy.ROSException:
            with self.lock:
                service_map.pop(drone_id, None)
            return False
        except Exception as exc:
            rospy.logwarn(f"[DASHBOARD]: Service refresh error for {service_name}: {exc}")
            with self.lock:
                service_map.pop(drone_id, None)
            return False

    def set_swarm_leader(self, leader_drone):
        with self.lock:
            self.swarm_leader = leader_drone
            self.swarm_start_service = None

        if not leader_drone:
            return
        if Vec1SrvType is None:
            rospy.logwarn("[DASHBOARD]: mrs_msgs/Vec1 is unavailable, swarm start service disabled.")
            return

        thread = threading.Thread(target=self._discover_swarm_start_service, args=(leader_drone,), daemon=True)
        thread.start()
        self.discovery_threads.append(thread)

    def _discover_swarm_start_service(self, leader_drone):
        service_name = f"/{leader_drone}/sweeping_generator/start"
        rospy.loginfo(f"[DASHBOARD]: Waiting for swarm-start service: {service_name}")
        while not rospy.is_shutdown():
            with self.lock:
                if self.swarm_leader != leader_drone:
                    return
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
                proxy = rospy.ServiceProxy(service_name, Vec1SrvType, persistent=False)
                with self.lock:
                    if self.swarm_leader == leader_drone:
                        self.swarm_start_service = proxy
                rospy.loginfo(f"[DASHBOARD]: Swarm-start service ready: {service_name}")
                return
            except rospy.ROSException:
                continue
            except Exception as exc:
                rospy.logwarn(f"[DASHBOARD]: Service discovery error for {service_name}: {exc}")
                time.sleep(0.5)

    def refresh_ready_services(self):
        for drone_id in self.drone_ids:
            self._refresh_trigger_service(drone_id, "/uav_manager/takeoff", self.takeoff_services)
            self._refresh_trigger_service(drone_id, "/sweeping_generator/emergency", self.emergency_services)

        with self.lock:
            leader = self.swarm_leader

        if not leader or Vec1SrvType is None:
            with self.lock:
                self.swarm_start_service = None
            return

        service_name = f"/{leader}/sweeping_generator/start"
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
            proxy = rospy.ServiceProxy(service_name, Vec1SrvType, persistent=False)
            with self.lock:
                if self.swarm_leader == leader:
                    self.swarm_start_service = proxy
        except rospy.ROSException:
            with self.lock:
                if self.swarm_leader == leader:
                    self.swarm_start_service = None
        except Exception as exc:
            rospy.logwarn(f"[DASHBOARD]: Service refresh error for {service_name}: {exc}")
            with self.lock:
                if self.swarm_leader == leader:
                    self.swarm_start_service = None

    def ready_count(self):
        with self.lock:
            return len(self.takeoff_services)

    def emergency_ready_count(self):
        with self.lock:
            return len(self.emergency_services)

    def has_service(self, drone_id):
        with self.lock:
            return drone_id in self.takeoff_services

    def has_emergency_service(self, drone_id):
        with self.lock:
            return drone_id in self.emergency_services

    def get_swarm_leader(self):
        with self.lock:
            return self.swarm_leader

    def has_swarm_service(self):
        with self.lock:
            return self.swarm_start_service is not None

    def call_takeoff(self, drone_id):
        with self.lock:
            proxy = self.takeoff_services.get(drone_id)
        if proxy is None:
            return False, "service not ready"

        try:
            response = proxy()
            success = bool(getattr(response, "success", False))
            message = str(getattr(response, "message", "") or "")
            return success, message
        except Exception as exc:
            return False, str(exc)

    def call_emergency(self, drone_id):
        with self.lock:
            proxy = self.emergency_services.get(drone_id)
        if proxy is None:
            return False, "service not ready"

        try:
            response = proxy()
            success = bool(getattr(response, "success", False))
            message = str(getattr(response, "message", "") or "")
            return success, message
        except Exception as exc:
            return False, str(exc)

    def call_swarm_start(self, value):
        with self.lock:
            proxy = self.swarm_start_service
            leader = self.swarm_leader
        if proxy is None or leader is None:
            return False, "service not ready"
        if Vec1RequestType is None:
            return False, "Vec1 request type unavailable"

        try:
            request = Vec1RequestType()
            assigned = False
            for field_name in ("goal", "value", "data", "x", "vector"):
                if hasattr(request, field_name):
                    setattr(request, field_name, float(value))
                    assigned = True
                    break
            if not assigned and hasattr(request, "__slots__") and len(request.__slots__) > 0:
                setattr(request, request.__slots__[0], float(value))
                assigned = True
            if not assigned:
                return False, "unable to set Vec1 request value"

            response = proxy(request)
            success = bool(getattr(response, "success", True))
            message = str(getattr(response, "message", "") or "")
            if not message:
                message = f"leader={leader}, value={value:.2f}"
            return success, message
        except Exception as exc:
            return False, str(exc)


class DroneDashboard(QMainWindow):
    def __init__(self, data, takeoff_controller, ping_monitor):
        super().__init__()
        self.data = data
        self.takeoff_controller = takeoff_controller
        self.ping_monitor = ping_monitor
        self.selected_drone = DRONE_IDS[0]
        self.ordered_active_drones = parse_ordered_drones_from_env()
        self.active_drones = set(self.ordered_active_drones)
        self.swarm_monitor_drones = list(self.ordered_active_drones)
        self.leader_drone = self.ordered_active_drones[0] if self.ordered_active_drones else None
        self.swarm_default_value = 0.2
        self.takeoff_controller.set_swarm_leader(self.leader_drone)
        self.action_busy = False
        self.action_results = []
        self.action_results_lock = threading.Lock()
        self.current_action_channel = None
        self.refresh_busy = False
        self.setWindowTitle("Fleet Telemetry Dashboard")
        self.setMinimumSize(1200, 720)

        self.setStyleSheet(
            """
            QMainWindow { background-color: #edf1f5; }
            QFrame#statusPanel, QFrame#controlPanel {
              background-color: #ffffff;
              border: 1px solid #d9e0ea;
              border-radius: 10px;
            }
            QFrame#selectorPanel, QFrame#telemetryPanel, QFrame#controlInnerPanel {
              background-color: #f7f9fc;
              border: 1px solid #e4e9f0;
              border-radius: 8px;
            }
            QLabel#sectionTitle {
              font-size: 14px;
              font-weight: 700;
              color: #243447;
              padding: 2px;
            }
            QTableWidget {
              background-color: #ffffff;
              gridline-color: #dde3ec;
            }
            """
        )

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(12)

        status_panel = QFrame()
        status_panel.setObjectName("statusPanel")
        status_layout = QVBoxLayout(status_panel)
        status_layout.setContentsMargins(12, 12, 12, 12)
        status_layout.setSpacing(8)

        control_panel = QFrame()
        control_panel.setObjectName("controlPanel")
        control_panel.setMinimumWidth(320)
        control_panel.setMaximumWidth(380)
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(12, 12, 12, 12)
        control_layout.setSpacing(8)

        main_layout.addWidget(status_panel, stretch=3)
        main_layout.addWidget(control_panel, stretch=1)

        central.setLayout(main_layout)

        status_title = QLabel("Status")
        status_title.setObjectName("sectionTitle")
        control_title = QLabel("Control")
        control_title.setObjectName("sectionTitle")

        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)
        ip_layout = QHBoxLayout()
        ip_layout.setSpacing(6)
        takeoff_main_layout = QVBoxLayout()
        takeoff_main_layout.setSpacing(6)
        grid = QGridLayout()
        takeoff_grid = QGridLayout()
        takeoff_grid.setSpacing(6)

        self.status_label = QLabel(f"{self.selected_drone} Status: DISCONNECTED")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: white; background-color: #c0392b; padding: 8px; border-radius: 6px;"
        )

        selector_panel = QFrame()
        selector_panel.setObjectName("selectorPanel")
        selector_layout = QVBoxLayout(selector_panel)
        selector_layout.setContentsMargins(8, 8, 8, 8)
        selector_layout.setSpacing(4)

        self.drone_buttons = {}
        self.ip_labels = {}
        for drone_id in DRONE_IDS:
            button = QPushButton(drone_id.upper())
            button.setCheckable(True)
            button.clicked.connect(lambda checked, d=drone_id: self.select_drone(d))
            self.drone_buttons[drone_id] = button
            button_layout.addWidget(button)

            ip_text = DRONE_PING_IP.get(drone_id, "") or "NO IP"
            ip_label = QLabel(ip_text)
            ip_label.setAlignment(Qt.AlignCenter)
            ip_label.setStyleSheet("font-size: 11px; color: #555555;")
            self.ip_labels[drone_id] = ip_label
            ip_layout.addWidget(ip_label)

        selector_layout.addLayout(button_layout)
        selector_layout.addLayout(ip_layout)

        self.drone_buttons[self.selected_drone].setChecked(True)
        self._update_button_styles()

        control_inner_panel = QFrame()
        control_inner_panel.setObjectName("controlInnerPanel")
        control_inner_layout = QVBoxLayout(control_inner_panel)
        control_inner_layout.setContentsMargins(8, 8, 8, 8)
        control_inner_layout.setSpacing(8)

        self.refresh_button = QPushButton("REFRESH")
        self.refresh_button.clicked.connect(self.manual_refresh)
        self.refresh_button.setStyleSheet(
            "font-size: 13px; font-weight: 700; padding: 8px; border: 1px solid #5b6777; background-color: #6c7a89; color: white; border-radius: 6px;"
        )

        self.takeoff_all_button = QPushButton("TAKE OFF ALL ACTIVE")
        self.takeoff_all_button.clicked.connect(self.takeoff_all_active)
        self.takeoff_all_button.setStyleSheet(
            "font-size: 13px; font-weight: 700; padding: 8px; border: 1px solid #3f6fd1; background-color: #4b7bec; color: white; border-radius: 6px;"
        )
        self.takeoff_status_label = QLabel("Takeoff services: discovering...")
        self.takeoff_status_label.setWordWrap(True)
        self.takeoff_status_label.setStyleSheet(
            "font-size: 12px; color: #2f3b4c; padding: 6px; border: 1px solid #dbe2ec; border-radius: 6px; background-color: #ffffff;"
        )
        takeoff_main_layout.addWidget(self.takeoff_all_button, 0)
        takeoff_main_layout.addWidget(self.takeoff_status_label, 0)

        self.takeoff_buttons = {}
        for idx, drone_id in enumerate(DRONE_IDS):
            button = QPushButton(f"TO {drone_id.upper()}")
            button.clicked.connect(lambda checked, d=drone_id: self.takeoff_single(d))
            self.takeoff_buttons[drone_id] = button
            row = idx // 2
            col = idx % 2
            takeoff_grid.addWidget(button, row, col)

        emergency_layout = QVBoxLayout()
        emergency_layout.setSpacing(6)
        self.emergency_all_button = QPushButton("EMERGENCY ALL ACTIVE")
        self.emergency_all_button.clicked.connect(self.emergency_all_active)
        self.emergency_all_button.setStyleSheet(
            "font-size: 13px; font-weight: 700; padding: 8px; border: 1px solid #b62929; background-color: #d64541; color: white; border-radius: 6px;"
        )
        self.emergency_status_label = QLabel("Emergency services: discovering...")
        self.emergency_status_label.setWordWrap(True)
        self.emergency_status_label.setStyleSheet(
            "font-size: 12px; color: #4a2e2e; padding: 6px; border: 1px solid #edd3d3; border-radius: 6px; background-color: #fff8f8;"
        )
        emergency_layout.addWidget(self.emergency_all_button, 0)
        emergency_layout.addWidget(self.emergency_status_label, 0)

        emergency_grid = QGridLayout()
        emergency_grid.setSpacing(6)
        self.emergency_buttons = {}
        for idx, drone_id in enumerate(DRONE_IDS):
            button = QPushButton(f"EM {drone_id.upper()}")
            button.clicked.connect(lambda checked, d=drone_id: self.emergency_single(d))
            self.emergency_buttons[drone_id] = button
            row = idx // 2
            col = idx % 2
            emergency_grid.addWidget(button, row, col)
        emergency_layout.addLayout(emergency_grid)

        swarm_layout = QVBoxLayout()
        swarm_layout.setSpacing(6)
        swarm_button_text = f"SWARM START LEADER ({self.swarm_default_value:.1f})"
        self.swarm_start_button = QPushButton(swarm_button_text)
        self.swarm_start_button.clicked.connect(self.swarm_start_leader)
        self.swarm_start_button.setStyleSheet(
            "font-size: 13px; font-weight: 700; padding: 8px; border: 1px solid #1d7f74; background-color: #26a69a; color: white; border-radius: 6px;"
        )
        self.swarm_status_label = QLabel("Swarm service: discovering...")
        self.swarm_status_label.setWordWrap(True)
        self.swarm_status_label.setStyleSheet(
            "font-size: 12px; color: #1e3f3c; padding: 6px; border: 1px solid #cde8e5; border-radius: 6px; background-color: #f4fffd;"
        )
        swarm_layout.addWidget(self.swarm_start_button, 0)
        swarm_layout.addWidget(self.swarm_status_label, 0)

        control_inner_layout.addWidget(self.refresh_button)
        control_inner_layout.addLayout(takeoff_main_layout)
        control_inner_layout.addLayout(takeoff_grid)
        control_inner_layout.addLayout(emergency_layout)
        control_inner_layout.addLayout(swarm_layout)

        self.battery_label = QLabel("Battery: N/A")
        self.flight_time_label = QLabel("Flight Time: N/A")
        self.flight_mode_label = QLabel("Flight Mode: UNKNOWN")
        self.gps_label = QLabel("GPS: 0.000000, 0.000000")
        self.link_label = QLabel("Link: no data")

        telemetry_panel = QFrame()
        telemetry_panel.setObjectName("telemetryPanel")
        telemetry_layout = QVBoxLayout(telemetry_panel)
        telemetry_layout.setContentsMargins(8, 8, 8, 8)
        telemetry_layout.setSpacing(6)

        for widget in [self.battery_label, self.flight_time_label, self.flight_mode_label, self.gps_label, self.link_label]:
            widget.setFrameStyle(QFrame.Panel | QFrame.Sunken)
            widget.setStyleSheet("font-size: 15px; padding: 6px;")

        grid.addWidget(self.battery_label, 0, 0, 1, 2)
        grid.addWidget(self.flight_time_label, 1, 0)
        grid.addWidget(self.flight_mode_label, 1, 1)
        grid.addWidget(self.gps_label, 2, 0, 1, 2)
        grid.addWidget(self.link_label, 3, 0, 1, 2)
        telemetry_layout.addLayout(grid)

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

        swarm_title = QLabel("Swarm Monitor")
        swarm_title.setObjectName("sectionTitle")
        self.swarm_table = QTableWidget(len(self.swarm_monitor_drones), 7)
        self.swarm_table.setHorizontalHeaderLabels(
            ["Drone", "GNSS", "LiDAR", "Swarm", "PreFlight", "Armed", "Offboard"]
        )
        self.swarm_table.verticalHeader().setVisible(False)
        self.swarm_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.swarm_table.setSelectionMode(QTableWidget.NoSelection)
        self.swarm_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.swarm_table.setMinimumHeight(220)

        for row, drone_id in enumerate(self.swarm_monitor_drones):
            name_item = QTableWidgetItem(drone_id)
            name_item.setTextAlignment(Qt.AlignCenter)
            self.swarm_table.setItem(row, 0, name_item)
            for col in range(1, 7):
                self._set_led_item(self.swarm_table, row, col, None)

        status_layout.addWidget(status_title)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(selector_panel)
        status_layout.addWidget(telemetry_panel)
        status_layout.addWidget(self.table)
        status_layout.addWidget(swarm_title)
        status_layout.addWidget(self.swarm_table)

        control_layout.addWidget(control_title)
        control_layout.addWidget(control_inner_panel)
        control_layout.addStretch(1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(200)  # refresh every 200 ms

    def select_drone(self, drone_id):
        self.selected_drone = drone_id
        for current_id, button in self.drone_buttons.items():
            button.setChecked(current_id == drone_id)
        self._update_button_styles()
        self.refresh_ui()

    def manual_refresh(self):
        if self.refresh_busy:
            return
        self.refresh_busy = True
        self.refresh_button.setEnabled(False)
        self.refresh_button.setText("REFRESHING...")
        worker = threading.Thread(target=self._run_manual_refresh, daemon=True)
        worker.start()

    def _run_manual_refresh(self):
        try:
            self.takeoff_controller.refresh_ready_services()
            self.ping_monitor.refresh_now()
            result_text = "REFRESH"
        except Exception as exc:
            rospy.logwarn(f"[DASHBOARD]: Manual refresh failed: {exc}")
            result_text = "REFRESH FAILED"
        self._queue_action_result("refresh", result_text)

    def _drone_link_ok(self, state, now):
        status_msg_time = state.get("status_msg_time", 0.0)
        status_age = now - status_msg_time if status_msg_time > 0 else 999.0
        if status_age < 2.0:
            return True
        ping_status = state.get("ping_status", "")
        return ping_status in ("OK", "TCP")

    def _update_button_styles(self, snapshot=None, now=None):
        for drone_id, button in self.drone_buttons.items():
            if snapshot is not None and now is not None:
                connected = self._drone_link_ok(snapshot.get(drone_id, {}), now)
                name_color = "black" if connected else "red"
                ip_color = "#555555" if connected else "red"
            else:
                is_active = drone_id in self.active_drones
                name_color = "black" if is_active else "red"
                ip_color = "#555555" if is_active else "red"
            if button.isChecked():
                button.setStyleSheet(
                    f"font-size: 12px; font-weight: bold; color: {name_color}; border: 2px solid #4b6cb7; border-radius: 5px; background-color: #eaf1ff;"
                )
            else:
                button.setStyleSheet(
                    f"font-size: 12px; font-weight: bold; color: {name_color}; border: 1px solid #ced7e3; border-radius: 5px; background-color: #ffffff;"
                )
            self.ip_labels[drone_id].setStyleSheet(f"font-size: 11px; color: {ip_color};")

    def _active_drone_ids(self):
        return [drone_id for drone_id in DRONE_IDS if drone_id in self.active_drones]

    def _set_led_item(self, table, row, col, value):
        led_item = QTableWidgetItem("●")
        led_item.setTextAlignment(Qt.AlignCenter)
        if value is True:
            led_item.setForeground(QColor("#1eaf5b"))
        elif value is False:
            led_item.setForeground(QColor("#d64541"))
        else:
            led_item.setForeground(QColor("#9aa5b1"))
        table.setItem(row, col, led_item)

    def _refresh_swarm_monitor(self, snapshot):
        for row, drone_id in enumerate(self.swarm_monitor_drones):
            state = snapshot.get(drone_id, {})
            self._set_led_item(self.swarm_table, row, 1, state.get("drone_status_gnss"))
            self._set_led_item(self.swarm_table, row, 2, state.get("drone_status_lidar"))
            self._set_led_item(self.swarm_table, row, 3, state.get("drone_status_swarm"))
            self._set_led_item(self.swarm_table, row, 4, state.get("drone_status_pre_flight"))
            self._set_led_item(self.swarm_table, row, 5, state.get("drone_status_armed"))
            self._set_led_item(self.swarm_table, row, 6, state.get("drone_status_offboard"))

    def _queue_action_result(self, channel, text):
        with self.action_results_lock:
            self.action_results.append((channel, text))

    def _process_action_results(self):
        pending = []
        with self.action_results_lock:
            if self.action_results:
                pending = self.action_results[:]
                self.action_results.clear()

        for channel, text in pending:
            if channel == "emergency":
                self.emergency_status_label.setText(text)
            elif channel == "swarm":
                self.swarm_status_label.setText(text)
            elif channel == "refresh":
                self.refresh_button.setText(text)
                self.refresh_button.setEnabled(True)
                self.refresh_busy = False
            else:
                self.takeoff_status_label.setText(text)
        if pending:
            non_refresh_pending = any(channel != "refresh" for channel, _ in pending)
            if non_refresh_pending:
                self.action_busy = False
                self.current_action_channel = None

    def _update_takeoff_controls(self):
        ready = self.takeoff_controller.ready_count()
        status_suffix = "busy" if self.action_busy else "idle"
        self.takeoff_status_label.setToolTip(f"Ready services: {ready}/{len(DRONE_IDS)} ({status_suffix})")

        ready_active = [
            drone_id for drone_id in self._active_drone_ids() if self.takeoff_controller.has_service(drone_id)
        ]
        self.takeoff_all_button.setEnabled(bool(ready_active) and not self.action_busy)

        for drone_id, button in self.takeoff_buttons.items():
            ready_drone = self.takeoff_controller.has_service(drone_id)
            is_active = drone_id in self.active_drones
            button.setEnabled(ready_drone and not self.action_busy)
            color = "black" if is_active else "red"
            border = "#4b6cb7" if ready_drone else "#999999"
            background = "#f4f8ff" if ready_drone else "#f6f6f6"
            button.setStyleSheet(
                f"font-size: 11px; font-weight: 600; color: {color}; border: 1px solid {border}; border-radius: 5px; background-color: {background}; padding: 6px;"
            )

    def _update_emergency_controls(self):
        ready = self.takeoff_controller.emergency_ready_count()
        status_suffix = "busy" if self.action_busy else "idle"
        self.emergency_status_label.setToolTip(f"Ready services: {ready}/{len(DRONE_IDS)} ({status_suffix})")

        ready_active = [
            drone_id for drone_id in self._active_drone_ids() if self.takeoff_controller.has_emergency_service(drone_id)
        ]
        self.emergency_all_button.setEnabled(bool(ready_active) and not self.action_busy)

        for drone_id, button in self.emergency_buttons.items():
            ready_drone = self.takeoff_controller.has_emergency_service(drone_id)
            is_active = drone_id in self.active_drones
            button.setEnabled(ready_drone and not self.action_busy)
            color = "black" if is_active else "red"
            border = "#b62929" if ready_drone else "#999999"
            background = "#fff1f1" if ready_drone else "#f6f6f6"
            button.setStyleSheet(
                f"font-size: 11px; font-weight: 600; color: {color}; border: 1px solid {border}; border-radius: 5px; background-color: {background}; padding: 6px;"
            )

    def _update_swarm_controls(self):
        leader = self.takeoff_controller.get_swarm_leader()
        ready = self.takeoff_controller.has_swarm_service()
        status_suffix = "busy" if self.action_busy else "idle"
        self.swarm_status_label.setToolTip(
            f"Leader: {leader if leader else 'N/A'} | Value: {self.swarm_default_value:.2f} | Ready: {ready} ({status_suffix})"
        )
        self.swarm_start_button.setEnabled(bool(leader) and ready and not self.action_busy)

    def emergency_single(self, drone_id):
        if self.action_busy:
            return
        self.action_busy = True
        self.current_action_channel = "emergency"
        self.emergency_status_label.setText(f"Emergency {drone_id}: running...")
        worker = threading.Thread(target=self._run_emergency_single, args=(drone_id,), daemon=True)
        worker.start()

    def _run_emergency_single(self, drone_id):
        success, message = self.takeoff_controller.call_emergency(drone_id)
        if success:
            text = f"Emergency {drone_id}: OK"
            if message:
                text = f"{text} ({message})"
        else:
            text = f"Emergency {drone_id}: FAILED"
            if message:
                text = f"{text} ({message})"
        self._queue_action_result("emergency", text)

    def swarm_start_leader(self):
        if self.action_busy:
            return

        leader = self.takeoff_controller.get_swarm_leader()
        if not leader:
            self.swarm_status_label.setText("Swarm start: leader unavailable")
            return

        self.action_busy = True
        self.current_action_channel = "swarm"
        self.swarm_status_label.setText(
            f"Swarm start {leader}: running (value={self.swarm_default_value:.2f})..."
        )
        worker = threading.Thread(target=self._run_swarm_start_leader, daemon=True)
        worker.start()

    def _run_swarm_start_leader(self):
        leader = self.takeoff_controller.get_swarm_leader()
        success, message = self.takeoff_controller.call_swarm_start(self.swarm_default_value)
        if success:
            text = f"Swarm start {leader}: OK"
            if message:
                text = f"{text} ({message})"
        else:
            text = f"Swarm start {leader}: FAILED"
            if message:
                text = f"{text} ({message})"
        self._queue_action_result("swarm", text)

    def takeoff_single(self, drone_id):
        if self.action_busy:
            return
        self.action_busy = True
        self.current_action_channel = "takeoff"
        self.takeoff_status_label.setText(f"Takeoff {drone_id}: running...")
        worker = threading.Thread(target=self._run_takeoff_single, args=(drone_id,), daemon=True)
        worker.start()

    def _run_takeoff_single(self, drone_id):
        success, message = self.takeoff_controller.call_takeoff(drone_id)
        if success:
            text = f"Takeoff {drone_id}: OK"
            if message:
                text = f"{text} ({message})"
        else:
            text = f"Takeoff {drone_id}: FAILED"
            if message:
                text = f"{text} ({message})"
        self._queue_action_result("takeoff", text)

    def takeoff_all_active(self):
        if self.action_busy:
            return

        targets = self._active_drone_ids()
        if not targets:
            self.takeoff_status_label.setText("Takeoff all: no active drones in UAV_NAMES")
            return

        self.action_busy = True
        self.current_action_channel = "takeoff"
        self.takeoff_status_label.setText(f"Takeoff all: running ({len(targets)} drones)...")
        worker = threading.Thread(target=self._run_takeoff_all, args=(targets,), daemon=True)
        worker.start()

    def _run_takeoff_all(self, targets):
        results = {}
        with ThreadPoolExecutor(max_workers=len(targets)) as executor:
            future_map = {executor.submit(self.takeoff_controller.call_takeoff, drone_id): drone_id for drone_id in targets}
            for future in as_completed(future_map):
                drone_id = future_map[future]
                try:
                    success, message = future.result()
                except Exception as exc:
                    success, message = False, str(exc)
                results[drone_id] = (success, message)

        ok_count = sum(1 for success, _ in results.values() if success)
        fail_items = [f"{drone_id}" for drone_id, (success, _) in results.items() if not success]
        text = f"Takeoff all: {ok_count}/{len(targets)} succeeded"
        if fail_items:
            text = f"{text} | failed: {', '.join(fail_items)}"
        self._queue_action_result("takeoff", text)

    def emergency_all_active(self):
        if self.action_busy:
            return

        targets = self._active_drone_ids()
        if not targets:
            self.emergency_status_label.setText("Emergency all: no active drones in UAV_NAMES")
            return

        self.action_busy = True
        self.current_action_channel = "emergency"
        self.emergency_status_label.setText(f"Emergency all: running ({len(targets)} drones)...")
        worker = threading.Thread(target=self._run_emergency_all, args=(targets,), daemon=True)
        worker.start()

    def _run_emergency_all(self, targets):
        results = {}
        with ThreadPoolExecutor(max_workers=len(targets)) as executor:
            future_map = {
                executor.submit(self.takeoff_controller.call_emergency, drone_id): drone_id for drone_id in targets
            }
            for future in as_completed(future_map):
                drone_id = future_map[future]
                try:
                    success, message = future.result()
                except Exception as exc:
                    success, message = False, str(exc)
                results[drone_id] = (success, message)

        ok_count = sum(1 for success, _ in results.values() if success)
        fail_items = [f"{drone_id}" for drone_id, (success, _) in results.items() if not success]
        text = f"Emergency all: {ok_count}/{len(targets)} succeeded"
        if fail_items:
            text = f"{text} | failed: {', '.join(fail_items)}"
        self._queue_action_result("emergency", text)

    def refresh_ui(self):
        self._process_action_results()
        self._update_takeoff_controls()
        self._update_emergency_controls()
        self._update_swarm_controls()
        snapshot = self.data.snapshot()
        self._refresh_swarm_monitor(snapshot)
        now = time.time()
        self._update_button_styles(snapshot=snapshot, now=now)

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
        selected_flight_time = selected["flight_time_s"]
        if selected_flight_time is None:
            self.flight_time_label.setText("Flight Time: N/A")
        else:
            self.flight_time_label.setText(f"Flight Time: {selected_flight_time:.1f} s")
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
            state = snapshot[drone_id]
            row_connected = self._drone_link_ok(state, now)
            row_color = QColor("black") if row_connected else QColor("red")

            name_item = QTableWidgetItem(drone_id)
            name_item.setForeground(row_color)
            self.table.setItem(row, 0, name_item)

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

            ping_item = QTableWidgetItem(ping_status)
            status_item = QTableWidgetItem(status_text)
            battery_item = QTableWidgetItem(battery_text)
            swarm_item = QTableWidgetItem(swarm_info_text)
            lidar_item = QTableWidgetItem(lidar_3d_text)
            gps_item = QTableWidgetItem(f"{state['latitude']:.6f}, {state['longitude']:.6f}")
            gnss_item = QTableWidgetItem(gnss_fix_text)

            for item in (ping_item, status_item, battery_item, swarm_item, lidar_item, gps_item, gnss_item):
                item.setForeground(row_color)

            self.table.setItem(row, 1, ping_item)
            self.table.setItem(row, 2, status_item)
            self.table.setItem(row, 3, battery_item)
            self.table.setItem(row, 4, swarm_item)
            self.table.setItem(row, 5, lidar_item)
            self.table.setItem(row, 6, gps_item)
            self.table.setItem(row, 7, gnss_item)


class RosInterface:
    def __init__(self, data):
        self.data = data
        self.subscribers = []
        self.active_drones = [drone_id for drone_id in DRONE_IDS if drone_id in parse_active_drones_from_env()]
        self.drone_status_gnss = {name: None for name in self.active_drones}
        self.drone_status_lidar = {name: None for name in self.active_drones}
        self.drone_status_swarm = {name: None for name in self.active_drones}
        self.drone_status_pre_flight = {name: None for name in self.active_drones}
        self.drone_status_armed = {name: None for name in self.active_drones}
        self.drone_status_offboard = {name: None for name in self.active_drones}

        for drone_id in DRONE_IDS:
            ns_prefix = f"/{drone_id}"
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
                    ns_prefix + DIAGNOSTICS_TOPIC_SUFFIX,
                    DiagnosticsMsgType,
                    self.diagnostics_cb,
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

        for name in self.active_drones:
            topic = f"/{name}{FMS_STATUS_TOPIC_SUFFIX}"
            sub = rospy.Subscriber(topic, UInt8MultiArray, self.make_callback(name))
            self.subscribers.append(sub)

    def flight_mode_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            state.flight_mode = msg.data
            state.heartbeat()

    def diagnostics_cb(self, msg, drone_id):
        with self.data.lock:
            state = self.data.states[drone_id]
            latitude = getattr(msg, "cur_latitude", None)
            if isinstance(latitude, (float, int)):
                state.latitude = float(latitude)

            longitude = getattr(msg, "cur_longitude", None)
            if isinstance(longitude, (float, int)):
                state.longitude = float(longitude)

            flight_time = getattr(msg, "flight_time", None)
            if isinstance(flight_time, (float, int)):
                state.flight_time_s = float(flight_time)

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

    def make_callback(self, name):
        def callback(msg):
            if not hasattr(msg, "data"):
                return

            gnss = bool(msg.data[0]) if len(msg.data) > 0 else False
            lidar = bool(msg.data[1]) if len(msg.data) > 1 else False
            swarm = bool(msg.data[2]) if len(msg.data) > 2 else False
            pre_flight = bool(msg.data[3]) if len(msg.data) > 3 else False
            armed = bool(msg.data[4]) if len(msg.data) > 4 else False
            offboard = bool(msg.data[5]) if len(msg.data) > 5 else False

            self.drone_status_gnss[name] = gnss
            self.drone_status_lidar[name] = lidar
            self.drone_status_swarm[name] = swarm
            self.drone_status_pre_flight[name] = pre_flight
            self.drone_status_armed[name] = armed
            self.drone_status_offboard[name] = offboard

            with self.data.lock:
                state = self.data.states[name]
                state.drone_status_gnss = gnss
                state.drone_status_lidar = lidar
                state.drone_status_swarm = swarm
                state.drone_status_pre_flight = pre_flight
                state.drone_status_armed = armed
                state.drone_status_offboard = offboard
                # Keep existing table fields in sync.
                state.lidar_3d_ok = lidar
                state.swarm_info_ok = swarm

        return callback

def main():
    rospy.init_node("drone_dashboard", anonymous=True)

    data = FleetData(DRONE_IDS)
    takeoff_controller = TakeoffController(DRONE_IDS)
    ros_interface = RosInterface(data)
    ping_monitor = PingMonitor(data, DRONE_PING_IP)

    app = QApplication(sys.argv)
    window = DroneDashboard(data, takeoff_controller, ping_monitor)
    window.show()

    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
