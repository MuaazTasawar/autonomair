import time
import unittest
import json
import os
from dronekit import connect, VehicleMode
from dronekit_sitl import SITL


def load_config():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config", "mission_params.json")
    with open(config_path) as f:
        return json.load(f)


class SITLTestBase(unittest.TestCase):
    """
    Base class that spins up a fresh SITL instance before each test
    and tears it down cleanly after.
    """

    @classmethod
    def setUpClass(cls):
        print("\n[TestBase] Starting SITL instance...")
        cls.sitl = SITL()
        cls.sitl.download("copter", "3.3", verbose=False)
        cls.sitl.launch({"speedup": "5"}, await_ready=True, restart=True)
        conn_str = cls.sitl.connection_string()
        cls.vehicle = connect(conn_str, wait_ready=True)
        cls.config  = load_config()
        print(f"[TestBase] Connected to SITL at {conn_str}")

    @classmethod
    def tearDownClass(cls):
        cls.vehicle.close()
        cls.sitl.stop()
        print("[TestBase] SITL stopped.")


# ------------------------------------------------------------------ #
#  Safety Monitor Tests                                                #
# ------------------------------------------------------------------ #

class TestSafetyMonitor(SITLTestBase):

    def test_monitor_starts_and_stops(self):
        from core.safety_monitor import SafetyMonitor
        monitor = SafetyMonitor(self.vehicle, self.config)
        monitor.start()
        self.assertTrue(monitor._running)
        time.sleep(1)
        monitor.stop()
        self.assertFalse(monitor._running)

    def test_haversine_zero_distance(self):
        from core.safety_monitor import SafetyMonitor
        dist = SafetyMonitor._haversine(10.0, 20.0, 10.0, 20.0)
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_haversine_known_distance(self):
        from core.safety_monitor import SafetyMonitor
        # Approx 111.2 km between 0,0 and 1,0
        dist = SafetyMonitor._haversine(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(dist, 111320, delta=200)


# ------------------------------------------------------------------ #
#  Takeoff / Land Tests                                                #
# ------------------------------------------------------------------ #

class TestTakeoffLand(SITLTestBase):

    def test_vehicle_becomes_armable(self):
        timeout, start = 30, time.time()
        while not self.vehicle.is_armable:
            self.assertLess(time.time() - start, timeout,
                            "Vehicle did not become armable in time.")
            time.sleep(1)
        self.assertTrue(self.vehicle.is_armable)

    def test_arm_and_takeoff(self):
        from core.takeoff_land import TakeoffLand
        tl = TakeoffLand(self.vehicle, self.config)
        tl.arm_and_takeoff(target_altitude=5)
        alt = self.vehicle.location.global_relative_frame.alt
        self.assertGreaterEqual(alt, 4.5,
            f"Expected alt >= 4.5m after takeoff, got {alt:.2f}m")

    def test_land(self):
        from core.takeoff_land import TakeoffLand
        tl = TakeoffLand(self.vehicle, self.config)
        tl.land()
        time.sleep(3)
        alt = self.vehicle.location.global_relative_frame.alt
        self.assertLessEqual(alt, 1.0,
            f"Expected alt <= 1.0m after landing, got {alt:.2f}m")

    def test_return_to_launch(self):
        from core.takeoff_land import TakeoffLand
        tl = TakeoffLand(self.vehicle, self.config)
        tl.arm_and_takeoff(target_altitude=5)
        tl.return_to_launch()
        time.sleep(2)
        self.assertEqual(self.vehicle.mode.name, "RTL")


# ------------------------------------------------------------------ #
#  Waypoint Mission Tests                                              #
# ------------------------------------------------------------------ #

class TestWaypointMission(SITLTestBase):

    def test_mission_upload(self):
        from core.waypoint_mission import WaypointMission
        wm = WaypointMission(self.vehicle, self.config)

        waypoints = [
            {"lat": -35.3633245, "lon": 149.1652373, "alt": 10},
            {"lat": -35.3629641, "lon": 149.1644025, "alt": 10},
            {"lat": -35.3624967, "lon": 149.1650177, "alt": 10},
        ]

        wm.build_mission(waypoints)
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready()

        # +2 for home waypoint and RTL command
        self.assertEqual(self.vehicle.commands.count, len(waypoints) + 2)

    def test_goto_changes_mode(self):
        from core.waypoint_mission import WaypointMission
        from core.takeoff_land import TakeoffLand

        tl = TakeoffLand(self.vehicle, self.config)
        tl.arm_and_takeoff(target_altitude=5)

        wm = WaypointMission(self.vehicle, self.config)
        wm.goto(-35.3633245, 149.1652373, alt=5)

        time.sleep(2)
        self.assertEqual(self.vehicle.mode.name, "GUIDED")


# ------------------------------------------------------------------ #
#  Formation Controller Tests                                          #
# ------------------------------------------------------------------ #

class TestFormationController(unittest.TestCase):
    """
    Pure geometry tests — no SITL needed.
    """

    def setUp(self):
        self.config = load_config()
        from swarm.formation_controller import FormationController
        self.fc = FormationController(self.config)

    def test_line_formation_count(self):
        positions = self.fc.get_positions("line", 0.0, 0.0, 10, 0, 3)
        self.assertEqual(len(positions), 3)

    def test_triangle_formation_count(self):
        positions = self.fc.get_positions("triangle", 0.0, 0.0, 10, 0, 3)
        self.assertEqual(len(positions), 3)

    def test_v_shape_formation_count(self):
        positions = self.fc.get_positions("v_shape", 0.0, 0.0, 10, 0, 4)
        self.assertEqual(len(positions), 4)

    def test_circle_formation_count(self):
        positions = self.fc.get_positions("circle", 0.0, 0.0, 10, 0, 5)
        self.assertEqual(len(positions), 5)

    def test_invalid_formation_raises(self):
        with self.assertRaises(ValueError):
            self.fc.get_positions("hexagon", 0.0, 0.0, 10, 0, 3)

    def test_lead_drone_is_at_origin_line(self):
        positions = self.fc.get_positions("line", 10.0, 20.0, 10, 0, 3)
        self.assertAlmostEqual(positions[0]["lat"], 10.0, places=5)
        self.assertAlmostEqual(positions[0]["lon"], 20.0, places=5)

    def test_spacing_is_respected(self):
        spacing = self.config.get("formation_spacing", 10)
        positions = self.fc.get_positions("line", 0.0, 0.0, 10, 0, 2)
        lat_diff_m = abs(positions[1]["lat"] - positions[0]["lat"]) * 111320
        self.assertAlmostEqual(lat_diff_m, spacing, delta=1.0)


# ------------------------------------------------------------------ #
#  Telemetry Stream Tests                                              #
# ------------------------------------------------------------------ #

class TestTelemetryStream(SITLTestBase):

    def test_snapshot_has_required_keys(self):
        from flask_socketio import SocketIO
        from flask import Flask
        from dashboard.telemetry_stream import TelemetryStream

        app = Flask(__name__)
        socketio = SocketIO(app)
        stream = TelemetryStream(self.vehicle, socketio)
        snap = stream.snapshot()

        required = [
            "lat", "lon", "alt", "roll", "pitch", "yaw",
            "groundspeed", "airspeed", "battery_level",
            "battery_voltage", "mode", "armed", "heading",
            "ekf_ok", "gps_fix", "satellites", "timestamp"
        ]

        for key in required:
            self.assertIn(key, snap, f"Missing key: {key}")

    def test_stream_starts_and_stops(self):
        from flask_socketio import SocketIO
        from flask import Flask
        from dashboard.telemetry_stream import TelemetryStream

        app = Flask(__name__)
        socketio = SocketIO(app)
        stream = TelemetryStream(self.vehicle, socketio)
        stream.start()
        self.assertTrue(stream._running)
        time.sleep(1)
        stream.stop()
        self.assertFalse(stream._running)


if __name__ == "__main__":
    unittest.main(verbosity=2)