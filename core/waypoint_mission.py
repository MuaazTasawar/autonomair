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
        from pymavlink import mavutil
        print("[WaypointMission] Starting mission in AUTO mode...")

        # AUTO mode = 3 for ArduCopter — send via MAVLink directly
        timeout, start = 15, time.time()
        while True:
            self.vehicle._master.mav.set_mode_send(
                self.vehicle._master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                3
            )
            self.vehicle.mode = VehicleMode("AUTO")
            time.sleep(2)
            if self.vehicle.mode.name == "AUTO":
                print("[WaypointMission] AUTO mode confirmed.")
                break
            if time.time() - start > timeout:
                print("[WaypointMission] Warning: could not confirm AUTO, proceeding anyway.")
                break
            print(f"  Mode is {self.vehicle.mode.name}, retrying AUTO...")

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
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready()
        total = self.vehicle.commands.count
        print(f"[WaypointMission] Monitoring {total} commands...")

        last_wp = None
        no_progress_count = 0
        last_progress_wp = -1

        while True:
            current_wp = self.vehicle.commands.next
            mode = self.vehicle.mode.name
            alt = self.vehicle.location.global_relative_frame.alt or 0

            if current_wp != last_wp:
                print(f"  Waypoint {current_wp} / {total} | mode={mode} | alt={alt:.1f}m")
                last_wp = current_wp

            # Detect mission complete
            if mode in ("RTL", "LAND") or current_wp >= total:
                print("[WaypointMission] Mission complete.")
                break

            # Detect no progress — if stuck on same waypoint too long
            if current_wp == last_progress_wp:
                no_progress_count += 1
                if no_progress_count > 60:
                    print("[WaypointMission] No progress detected — mission may be complete.")
                    break
            else:
                no_progress_count = 0
                last_progress_wp = current_wp

            time.sleep(1)