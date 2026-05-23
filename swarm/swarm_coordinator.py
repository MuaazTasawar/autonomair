import time
import threading
import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from .formation_controller import FormationController


class SwarmCoordinator:
    """
    Connects to multiple ArduPilot SITL instances and coordinates
    them as a swarm — takeoff, formation flight, and landing.

    Each SITL instance must be running on a separate UDP port.
    Default ports: 14550, 14560, 14570 (one per drone).
    """

    def __init__(self, config):
        self.config = config
        self.formation = FormationController(config)
        self.vehicles = []       # list of connected DroneKit vehicle objects
        self.threads = []        # monitoring threads
        self._running = False

    # ------------------------------------------------------------------ #
    #  Connection                                                          #
    # ------------------------------------------------------------------ #

    def connect_all(self):
        """Connect to all SITL instances defined in config."""
        instances = self.config.get("sitl_instances", [
            {"id": 0, "port": 14550},
            {"id": 1, "port": 14560},
            {"id": 2, "port": 14570},
        ])

        print(f"[SwarmCoordinator] Connecting to {len(instances)} drones...")

        for inst in instances:
            port = inst["port"]
            conn_str = f"udp:127.0.0.1:{port}"
            print(f"  Connecting drone {inst['id']} on {conn_str}...")
            try:
                vehicle = connect(conn_str, wait_ready=True, timeout=30)
                self.vehicles.append(vehicle)
                print(f"  Drone {inst['id']} connected.")
            except Exception as e:
                print(f"  Failed to connect drone {inst['id']}: {e}")

        print(f"[SwarmCoordinator] {len(self.vehicles)} drones connected.")

    def disconnect_all(self):
        """Cleanly close all vehicle connections."""
        for i, v in enumerate(self.vehicles):
            v.close()
            print(f"[SwarmCoordinator] Drone {i} disconnected.")
        self.vehicles.clear()

    # ------------------------------------------------------------------ #
    #  Swarm flight operations                                             #
    # ------------------------------------------------------------------ #

    def arm_all(self):
        """Arm all drones simultaneously using threads."""
        print("[SwarmCoordinator] Arming all drones...")
        self._run_parallel(self._arm_vehicle, self.vehicles)
        print("[SwarmCoordinator] All drones armed.")

    def takeoff_all(self, altitude=None):
        """Command all drones to take off to a target altitude."""
        alt = altitude or self.config.get("takeoff_altitude", 10)
        print(f"[SwarmCoordinator] All drones taking off to {alt}m...")
        self._run_parallel(lambda v: self._takeoff_vehicle(v, alt), self.vehicles)
        print("[SwarmCoordinator] All drones airborne.")

    def fly_formation(self, formation_name, duration=30):
        """
        Command all drones into a named formation and hold it.

        Args:
            formation_name : one of line / triangle / v_shape / circle
            duration       : seconds to hold formation
        """
        if not self.vehicles:
            print("[SwarmCoordinator] No vehicles connected.")
            return

        lead = self.vehicles[0]
        lead_loc = lead.location.global_relative_frame
        heading = lead.heading or 0

        positions = self.formation.get_positions(
            formation=formation_name,
            lead_lat=lead_loc.lat,
            lead_lon=lead_loc.lon,
            lead_alt=lead_loc.alt,
            heading_deg=heading,
            num_drones=len(self.vehicles)
        )

        print(f"[SwarmCoordinator] Flying {formation_name} formation for {duration}s...")

        for i, (vehicle, pos) in enumerate(zip(self.vehicles, positions)):
            target = LocationGlobalRelative(pos["lat"], pos["lon"], pos["alt"])
            vehicle.simple_goto(target)
            print(f"  Drone {i} → ({pos['lat']:.6f}, {pos['lon']:.6f}) alt={pos['alt']}m")

        # Hold formation, update positions every 2 seconds
        start = time.time()
        while time.time() - start < duration:
            self._update_formation(formation_name)
            time.sleep(2)

        print("[SwarmCoordinator] Formation hold complete.")

    def land_all(self):
        """Land all drones simultaneously."""
        print("[SwarmCoordinator] Landing all drones...")
        self._run_parallel(lambda v: v.__setattr__("mode", VehicleMode("LAND")), self.vehicles)
        print("[SwarmCoordinator] All drones landing.")

    def rtl_all(self):
        """Return all drones to launch simultaneously."""
        print("[SwarmCoordinator] RTL all drones...")
        self._run_parallel(lambda v: v.__setattr__("mode", VehicleMode("RTL")), self.vehicles)

    # ------------------------------------------------------------------ #
    #  Monitoring                                                          #
    # ------------------------------------------------------------------ #

    def print_status(self):
        """Print a status table for all connected drones."""
        print("\n--- Swarm Status ---")
        for i, v in enumerate(self.vehicles):
            loc = v.location.global_relative_frame
            print(
                f"  Drone {i} | Mode: {v.mode.name:10} | "
                f"Alt: {loc.alt:.1f}m | "
                f"Bat: {v.battery.level}% | "
                f"Armed: {v.armed}"
            )
        print("--------------------\n")

    # ------------------------------------------------------------------ #
    #  Internal helpers                                                    #
    # ------------------------------------------------------------------ #

    def _arm_vehicle(self, vehicle):
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        timeout, start = 15, time.time()
        while not vehicle.armed:
            if time.time() - start > timeout:
                raise TimeoutError("Vehicle failed to arm.")
            time.sleep(0.5)

    def _takeoff_vehicle(self, vehicle, alt):
        vehicle.simple_takeoff(alt)
        while True:
            current_alt = vehicle.location.global_relative_frame.alt or 0
            if current_alt >= alt * 0.95:
                break
            time.sleep(0.5)

    def _update_formation(self, formation_name):
        """Recalculate and reissue formation positions based on lead drone."""
        if not self.vehicles:
            return
        lead = self.vehicles[0]
        lead_loc = lead.location.global_relative_frame
        heading = lead.heading or 0

        positions = self.formation.get_positions(
            formation=formation_name,
            lead_lat=lead_loc.lat,
            lead_lon=lead_loc.lon,
            lead_alt=lead_loc.alt,
            heading_deg=heading,
            num_drones=len(self.vehicles)
        )

        for vehicle, pos in zip(self.vehicles[1:], positions[1:]):
            target = LocationGlobalRelative(pos["lat"], pos["lon"], pos["alt"])
            vehicle.simple_goto(target)

    def _run_parallel(self, fn, vehicles):
        """Run a function on all vehicles simultaneously using threads."""
        threads = []
        for v in vehicles:
            t = threading.Thread(target=fn, args=(v,), daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()