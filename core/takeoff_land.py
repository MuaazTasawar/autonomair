import time
from dronekit import VehicleMode, LocationGlobalRelative


class TakeoffLand:
    """
    Handles arming, takeoff to a target altitude, hover, and landing.
    Depends on SafetyMonitor being started externally before flight.
    """

    def __init__(self, vehicle, config):
        self.vehicle = vehicle
        self.config = config

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def arm_and_takeoff(self, target_altitude=None):
        alt = target_altitude or self.config.get("takeoff_altitude", 10)

        print("[TakeoffLand] Running pre-flight checks...")
        self._preflight_checks()

        print("[TakeoffLand] Arming motors...")
        self._arm()

        print(f"[TakeoffLand] Taking off to {alt}m...")
        self.vehicle.simple_takeoff(alt)

        self._wait_for_altitude(alt)
        print(f"[TakeoffLand] Reached {alt}m — hovering.")

    def land(self):
        print("[TakeoffLand] Initiating landing...")
        self.vehicle.mode = VehicleMode("LAND")
        self._wait_for_landed()
        print("[TakeoffLand] Landed successfully.")

    def return_to_launch(self):
        print("[TakeoffLand] Returning to launch...")
        self.vehicle.mode = VehicleMode("RTL")

    def hover(self, duration):
        print(f"[TakeoffLand] Hovering for {duration}s...")
        time.sleep(duration)

    # ------------------------------------------------------------------ #
    #  Internal helpers                                                    #
    # ------------------------------------------------------------------ #

    def _preflight_checks(self):
        # Wait until vehicle is ready
        timeout = 30
        start = time.time()
        while not self.vehicle.is_armable:
            if time.time() - start > timeout:
                raise TimeoutError("[TakeoffLand] Vehicle not armable after 30s. Check SITL connection.")
            print("  Waiting for vehicle to initialise...")
            time.sleep(1)
        print("  Pre-flight checks passed.")

    def _arm(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        timeout = 15
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                raise TimeoutError("[TakeoffLand] Vehicle failed to arm.")
            print("  Waiting for arming...")
            time.sleep(1)
        print("  Motors armed.")

    def _wait_for_altitude(self, target_alt, tolerance=0.95):
        while True:
            current = self.vehicle.location.global_relative_frame.alt or 0
            print(f"  Altitude: {current:.1f}m / {target_alt}m")
            if current >= target_alt * tolerance:
                break
            time.sleep(0.5)

    def _wait_for_landed(self):
        while self.vehicle.location.global_relative_frame.alt > 0.1:
            print(f"  Altitude: {self.vehicle.location.global_relative_frame.alt:.1f}m")
            time.sleep(1)