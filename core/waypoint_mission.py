import time
from dronekit import Command, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class WaypointMission:
    """
    Builds, uploads, and monitors autonomous waypoint missions
    using the MAVLink mission protocol.
    """

    def __init__(self, vehicle, config):
        self.vehicle = vehicle
        self.config = config

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def build_mission(self, waypoints):
        """
        waypoints: list of dicts with keys: lat, lon, alt
        Example:
            [
                {"lat": -35.3633245, "lon": 149.1652373, "alt": 10},
                {"lat": -35.3629641, "lon": 149.1644025, "alt": 15},
            ]
        """
        cmds = self.vehicle.commands
        cmds.clear()

        # Dummy waypoint 0 — home position (required by ArduPilot)
        home = self.vehicle.location.global_relative_frame
        cmds.add(self._make_command(0, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    home.lat, home.lon, 0))

        for wp in waypoints:
            cmds.add(self._make_command(
                0, 0,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                wp["lat"], wp["lon"], wp["alt"]
            ))

        # Final command — return to launch
        cmds.add(self._make_command(0, 0, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                    0, 0, 0))

        cmds.upload()
        print(f"[WaypointMission] Uploaded {len(waypoints)} waypoints.")

    def execute(self):
        """Switch to AUTO mode and monitor mission progress."""
        print("[WaypointMission] Starting mission in AUTO mode...")
        self.vehicle.mode = VehicleMode("AUTO")

        self._monitor_mission()

    def goto(self, lat, lon, alt=None):
        """Fly directly to a single GPS coordinate in GUIDED mode."""
        alt = alt or self.config.get("default_altitude", 10)
        target = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target, airspeed=self.config.get("default_airspeed", 5))
        print(f"[WaypointMission] Going to ({lat}, {lon}) at {alt}m")

    # ------------------------------------------------------------------ #
    #  Internal helpers                                                    #
    # ------------------------------------------------------------------ #

    def _make_command(self, current, autocontinue, command, lat, lon, alt):
        return Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            current, autocontinue,
            0, 0, 0, 0,
            lat, lon, alt
        )

    def _monitor_mission(self):
        total = self.vehicle.commands.count
        print(f"[WaypointMission] Monitoring {total} commands...")

        last_wp = None
        while True:
            current_wp = self.vehicle.commands.next
            mode = self.vehicle.mode.name

            if current_wp != last_wp:
                print(f"  Heading to waypoint {current_wp} / {total}")
                last_wp = current_wp

            if mode == "RTL" or current_wp >= total:
                print("[WaypointMission] Mission complete. RTL initiated.")
                break

            time.sleep(1)