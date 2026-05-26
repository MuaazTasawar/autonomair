import time
from dronekit import VehicleMode
from pymavlink import mavutil


class TakeoffLand:

    def __init__(self, vehicle, config):
        self.vehicle = vehicle
        self.config = config

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def arm_and_takeoff(self, target_altitude=None):
        alt = target_altitude or self.config.get("takeoff_altitude", 10)

        print("[TakeoffLand] Step 1: Disabling arming checks...")
        self._disable_prearm_checks()

        print("[TakeoffLand] Step 2: Waiting for armable state...")
        self._wait_for_armable()

        print("[TakeoffLand] Step 3: Switching to GUIDED...")
        self._set_guided()

        print("[TakeoffLand] Step 4: Arming motors...")
        self._arm_motors()

        print(f"[TakeoffLand] Step 5: Taking off to {alt}m...")
        self._takeoff(alt)

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
    #  Steps                                                               #
    # ------------------------------------------------------------------ #

    def _disable_prearm_checks(self):
        for attempt in range(10):
            try:
                self.vehicle.parameters["ARMING_CHECK"]  = 0
                self.vehicle.parameters["FS_THR_ENABLE"] = 0
                self.vehicle.parameters["FS_GCS_ENABLE"] = 0
                time.sleep(1)
                val = self.vehicle.parameters.get("ARMING_CHECK", 1)
                if val == 0:
                    print(f"  ARMING_CHECK=0 confirmed on attempt {attempt + 1}.")
                    return
                print(f"  Attempt {attempt + 1}: ARMING_CHECK={val}, retrying...")
            except Exception as e:
                print(f"  Param write error: {e}")
            time.sleep(1)
        print("  Warning: could not confirm ARMING_CHECK=0, proceeding anyway.")

    def _wait_for_armable(self):
        timeout, start = 60, time.time()
        while not self.vehicle.is_armable:
            elapsed = int(time.time() - start)
            gps = self.vehicle.gps_0.fix_type if self.vehicle.gps_0 else 0
            print(f"  [{elapsed}s] is_armable={self.vehicle.is_armable} "
                  f"gps={gps} mode={self.vehicle.mode.name}")
            if time.time() - start > timeout:
                print("  Warning: not armable after 60s, proceeding anyway.")
                return
            time.sleep(2)
        print("  Vehicle is armable.")

    def _set_guided(self):
        """
        Send GUIDED mode change directly via MAVLink SET_MODE.
        ArduCopter GUIDED = mode 4.
        DroneKit's vehicle.mode setter is unreliable on TCP connections.
        """
        GUIDED_MODE_NUM = 4

        timeout, start = 30, time.time()
        while True:
            # Method 1: direct MAVLink set_mode_send
            self.vehicle._master.mav.set_mode_send(
                self.vehicle._master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                GUIDED_MODE_NUM
            )

            # Method 2: also try DroneKit setter as backup
            self.vehicle.mode = VehicleMode("GUIDED")

            time.sleep(2)
            mode = self.vehicle.mode.name
            print(f"  Current mode: {mode}")

            if mode == "GUIDED":
                print("  GUIDED confirmed.")
                return

            if time.time() - start > timeout:
                raise TimeoutError(
                    f"[TakeoffLand] Cannot enter GUIDED after 30s. "
                    f"Stuck at: {mode}"
                )
            print(f"  Retrying GUIDED...")

    def _arm_motors(self):
        self.vehicle.armed = True
        timeout, start = 20, time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                raise TimeoutError("[TakeoffLand] Vehicle failed to arm.")
            print("  Waiting for arming...")
            time.sleep(1)
        print("  Motors armed.")
        time.sleep(1)

    def _takeoff(self, alt):
        self.vehicle.simple_takeoff(alt)
        for _ in range(5):
            time.sleep(1)
            current = self.vehicle.location.global_relative_frame.alt or 0
            if current > 0.3:
                break
            if self.vehicle.armed:
                print("  Re-sending takeoff command...")
                self.vehicle.simple_takeoff(alt)

    def _wait_for_altitude(self, target_alt, tolerance=0.95):
        no_climb_count = 0
        last_alt = 0
        while True:
            current = self.vehicle.location.global_relative_frame.alt or 0
            print(f"  Altitude: {current:.1f}m / {target_alt}m")
            if current >= target_alt * tolerance:
                break
            if not self.vehicle.armed:
                raise RuntimeError("[TakeoffLand] Vehicle disarmed during takeoff.")
            if abs(current - last_alt) < 0.05:
                no_climb_count += 1
                if no_climb_count > 20:
                    raise RuntimeError("[TakeoffLand] Not climbing — SITL issue.")
            else:
                no_climb_count = 0
            last_alt = current
            time.sleep(0.5)

    def _wait_for_landed(self):
        while self.vehicle.location.global_relative_frame.alt > 0.1:
            print(f"  Altitude: {self.vehicle.location.global_relative_frame.alt:.1f}m")
            time.sleep(1)