import time
import cv2
from dronekit import VehicleMode, LocationGlobalRelative


class PrecisionLand:
    """
    Uses TargetDetector output to guide the drone over a ground target
    using a PID-based correction loop, then executes a precision landing.

    Coordinate correction is done in NED (North-East-Down) frame using
    the drone's current altitude as a scale factor.
    """

    def __init__(self, vehicle, detector, config):
        self.vehicle = vehicle
        self.detector = detector
        self.config = config

        # PID gains — tune these for your simulation environment
        self.kp = 0.00005
        self.ki = 0.000001
        self.kd = 0.00001

        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0

        self._frame_width = 640
        self._frame_height = 480

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def run(self, camera_index=0, max_duration=300):
        """
        Main loop: read camera, detect target, correct position.
        Descends gradually once centered, then lands.

        Args:
            camera_index: OpenCV camera index (0 = default webcam)
            max_duration: safety timeout in seconds
        """
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            raise RuntimeError("[PrecisionLand] Cannot open camera.")

        self._frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("[PrecisionLand] Starting precision landing sequence...")
        self.vehicle.mode = VehicleMode("GUIDED")

        start_time = time.time()
        centered_count = 0

        try:
            while True:
                elapsed = time.time() - start_time
                if elapsed > max_duration:
                    print("[PrecisionLand] Timeout reached — switching to normal LAND.")
                    break

                ret, frame = cap.read()
                if not ret:
                    continue

                result = self.detector.detect(frame)
                alt = self.vehicle.location.global_relative_frame.alt or 1.0

                if not result["found"]:
                    print(f"  [{elapsed:.1f}s] Target not found — holding position.")
                    time.sleep(0.1)
                    continue

                cx, cy = result["cx"], result["cy"]
                error_x, error_y = self._compute_error(cx, cy)

                print(f"  [{elapsed:.1f}s] Target at ({cx},{cy}) "
                      f"error=({error_x:.0f},{error_y:.0f}px) alt={alt:.1f}m")

                # Check if we're centered enough to descend
                if abs(error_x) < 80 and abs(error_y) < 80:
                    centered_count += 1
                    if centered_count >= 5:
                        print("[PrecisionLand] Centered. Descending...")
                        self._descend_step(alt)
                        centered_count = 0
                else:
                    centered_count = 0
                    correction = self._pid_correction(error_x, error_y, alt)
                    self._apply_correction(correction)

                # Land when close enough to the ground
                if alt < 1.0:
                    print("[PrecisionLand] Close to ground — executing final LAND.")
                    break

                cv2.imshow("PrecisionLand", result["annotated_frame"])
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[PrecisionLand] Aborted by user.")
                    break

                time.sleep(0.05)

        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.vehicle.mode = VehicleMode("LAND")
            print("[PrecisionLand] LAND mode engaged.")

    # ------------------------------------------------------------------ #
    #  PID control                                                         #
    # ------------------------------------------------------------------ #

    def _compute_error(self, cx, cy):
        """Pixel error from frame center."""
        error_x = cx - self._frame_width // 2
        error_y = cy - self._frame_height // 2
        return error_x, error_y

    def _pid_correction(self, error_x, error_y, alt):
        """
        Compute NED correction in metres using PID.
        Altitude is used as a scale factor — higher = larger corrections.
        """
        scale = max(alt, 1.0)

        # Integral
        self._integral_x += error_x
        self._integral_y += error_y

        # Derivative
        deriv_x = error_x - self._prev_error_x
        deriv_y = error_y - self._prev_error_y

        self._prev_error_x = error_x
        self._prev_error_y = error_y

        correction_x = (self.kp * error_x +
                        self.ki * self._integral_x +
                        self.kd * deriv_x) * scale

        correction_y = (self.kp * error_y +
                        self.ki * self._integral_y +
                        self.kd * deriv_y) * scale

        return correction_x, correction_y

    def _apply_correction(self, correction):
        """Move the drone by sending a MAVLink SET_POSITION_TARGET command."""
        import math
        from pymavlink import mavutil

        north, east = correction
        current = self.vehicle.location.global_relative_frame

        new_lat = current.lat + (north / 111320)
        new_lon = current.lon + (east / (111320 * abs(
            math.cos(math.radians(current.lat))
        )))

        # Send position target via MAVLink directly
        self.vehicle._master.mav.set_position_target_global_int_send(
            0,
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position only
            int(new_lat * 1e7),
            int(new_lon * 1e7),
            int(current.alt),
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def _descend_step(self, current_alt, step=0.5):
        """Lower altitude by one step via MAVLink."""
        import math
        from pymavlink import mavutil

        new_alt = max(current_alt - step, 0.5)
        current = self.vehicle.location.global_relative_frame

        self.vehicle._master.mav.set_position_target_global_int_send(
            0,
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(current.lat * 1e7),
            int(current.lon * 1e7),
            int(new_alt),
            0, 0, 0,
            0, 0, 0,
            0, 0
        )