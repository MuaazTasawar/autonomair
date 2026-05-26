import time
import threading
import math


class SafetyMonitor:
    """
    Continuously monitors vehicle state and triggers failsafes
    if battery, geofence, or connection thresholds are breached.
    """

    def __init__(self, vehicle, config):
        self.vehicle = vehicle
        self.config = config
        self._running = False
        self._thread = None
        self.warnings = []

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        print("[SafetyMonitor] Started.")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        print("[SafetyMonitor] Stopped.")

    # ------------------------------------------------------------------ #
    #  Internal loop                                                       #
    # ------------------------------------------------------------------ #

    def _monitor_loop(self):
        while self._running:
            try:
                self._check_battery()
                self._check_geofence()
                self._check_connection()
            except Exception as e:
                print(f"[SafetyMonitor] Monitor error: {e}")
            time.sleep(1)

    # ------------------------------------------------------------------ #
    #  Individual checks                                                   #
    # ------------------------------------------------------------------ #

    def _check_battery(self):
        level = self.vehicle.battery.level
        threshold = self.config.get("battery_failsafe_percent", 20)
        if level is not None and level < threshold:
            msg = f"[SafetyMonitor] WARNING: Battery {level}% below threshold {threshold}%"
            print(msg)
            self.warnings.append(msg)
            self._trigger_rtl("Low battery")

    def _check_geofence(self):
        home = self.vehicle.home_location
        loc  = self.vehicle.location.global_relative_frame
        radius = self.config.get("geofence_radius", 200)

        if home is None or loc is None or loc.lat is None:
            return

        dist = self._haversine(home.lat, home.lon, loc.lat, loc.lon)
        if dist > radius:
            msg = (f"[SafetyMonitor] WARNING: Geofence breach — "
                   f"{dist:.1f}m from home (limit {radius}m)")
            print(msg)
            self.warnings.append(msg)
            self._trigger_rtl("Geofence breach")

    def _check_connection(self):
        """
        vehicle.last_heartbeat is seconds SINCE last heartbeat,
        not a Unix timestamp.
        """
        hb = self.vehicle.last_heartbeat
        if hb is None:
            return
        if hb > 5:
            msg = f"[SafetyMonitor] WARNING: No heartbeat for {hb:.1f}s"
            print(msg)
            self.warnings.append(msg)

    # ------------------------------------------------------------------ #
    #  Failsafe actions                                                    #
    # ------------------------------------------------------------------ #

    def _trigger_rtl(self, reason):
        from dronekit import VehicleMode
        print(f"[SafetyMonitor] Triggering RTL — reason: {reason}")
        self.vehicle.mode = VehicleMode("RTL")

    # ------------------------------------------------------------------ #
    #  Utilities                                                           #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi    = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))