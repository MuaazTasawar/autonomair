import time
import threading


class TelemetryStream:
    """
    Reads vehicle telemetry at a fixed interval and emits it
    over a Socket.IO instance so the dashboard updates in real time.

    Emitted event name: 'telemetry'
    Payload: dict with all vehicle state fields
    """

    def __init__(self, vehicle, socketio, interval=0.5):
        self.vehicle = vehicle
        self.socketio = socketio
        self.interval = interval
        self._running = False
        self._thread = None

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        print("[TelemetryStream] Streaming started.")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        print("[TelemetryStream] Streaming stopped.")

    def snapshot(self):
        """Return a single telemetry snapshot as a dict."""
        return self._build_payload()

    # ------------------------------------------------------------------ #
    #  Internal loop                                                       #
    # ------------------------------------------------------------------ #

    def _stream_loop(self):
        while self._running:
            try:
                payload = self._build_payload()
                self.socketio.emit("telemetry", payload)
            except Exception as e:
                print(f"[TelemetryStream] Error: {e}")
            time.sleep(self.interval)

    def _build_payload(self):
        v = self.vehicle
        loc = v.location.global_relative_frame
        att = v.attitude
        vel = v.velocity

        return {
            # Position
            "lat":          round(loc.lat, 7)  if loc.lat  else 0,
            "lon":          round(loc.lon, 7)  if loc.lon  else 0,
            "alt":          round(loc.alt, 2)  if loc.alt  else 0,

            # Attitude (degrees)
            "roll":         round(att.roll  * 57.2958, 2) if att.roll  else 0,
            "pitch":        round(att.pitch * 57.2958, 2) if att.pitch else 0,
            "yaw":          round(att.yaw   * 57.2958, 2) if att.yaw   else 0,

            # Velocity (m/s)
            "vx":           round(vel[0], 2) if vel else 0,
            "vy":           round(vel[1], 2) if vel else 0,
            "vz":           round(vel[2], 2) if vel else 0,
            "groundspeed":  round(v.groundspeed, 2),
            "airspeed":     round(v.airspeed, 2),

            # Power
            "battery_level":   v.battery.level   if v.battery.level   else 0,
            "battery_voltage": round(v.battery.voltage, 2) if v.battery.voltage else 0,

            # State
            "mode":         v.mode.name,
            "armed":        v.armed,
            "heading":      v.heading or 0,
            "ekf_ok":       v.ekf_ok,
            "is_armable":   v.is_armable,
            "gps_fix":      v.gps_0.fix_type if v.gps_0 else 0,
            "satellites":   v.gps_0.satellites_visible if v.gps_0 else 0,

            # Timestamp
            "timestamp":    round(time.time(), 3),
        }