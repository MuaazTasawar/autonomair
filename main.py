#!/usr/bin/env python3
"""
AutonomAir — Main Entry Point
"""

import json
import os
import time

# ------------------------------------------------------------------ #
#  Config loader                                                       #
# ------------------------------------------------------------------ #

def load_config():
    path = os.path.join(os.path.dirname(__file__), "config", "mission_params.json")
    with open(path) as f:
        return json.load(f)


# ------------------------------------------------------------------ #
#  SITL + Vehicle connection                                           #
# ------------------------------------------------------------------ #

def start_sitl():
    from dronekit_sitl import SITL
    print("[Main] Starting SITL simulator...")
    sitl = SITL()
    sitl.download("copter", "3.3", verbose=False)
    sitl.launch([], await_ready=True, restart=True)
    print(f"[Main] SITL running at {sitl.connection_string()}")
    return sitl


def connect_vehicle(conn_str):
    from dronekit import connect
    print(f"[Main] Connecting to vehicle at {conn_str}...")
    vehicle = connect(conn_str, wait_ready=True, timeout=60)
    time.sleep(3)
    print(f"[Main] Connected. Mode: {vehicle.mode.name} | "
          f"Armable: {vehicle.is_armable} | "
          f"GPS fix: {vehicle.gps_0.fix_type if vehicle.gps_0 else 0}")
    return vehicle


# ------------------------------------------------------------------ #
#  Module runners                                                      #
# ------------------------------------------------------------------ #

def run_takeoff_land(vehicle, config):
    from core.safety_monitor import SafetyMonitor
    from core.takeoff_land import TakeoffLand

    tl = TakeoffLand(vehicle, config)
    monitor = None
    try:
        tl.arm_and_takeoff()

        monitor = SafetyMonitor(vehicle, config)
        monitor.start()

        tl.hover(duration=5)
        tl.land()
    except Exception as e:
        print(f"[Main] Error during flight: {e}")
    finally:
        if monitor:
            monitor.stop()


def run_waypoint_mission(vehicle, config):
    from core.safety_monitor import SafetyMonitor
    from core.takeoff_land import TakeoffLand
    from core.waypoint_mission import WaypointMission

    tl = TakeoffLand(vehicle, config)
    monitor = None
    try:
        # Must take off before uploading mission
        tl.arm_and_takeoff()

        monitor = SafetyMonitor(vehicle, config)
        monitor.start()

        wm = WaypointMission(vehicle, config)

        waypoints = [
            {"lat": -35.3633245, "lon": 149.1652373, "alt": 10},
            {"lat": -35.3629641, "lon": 149.1644025, "alt": 15},
            {"lat": -35.3624967, "lon": 149.1650177, "alt": 10},
            {"lat": -35.3628731, "lon": 149.1658758, "alt": 12},
        ]

        wm.build_mission(waypoints)
        wm.execute()

    except Exception as e:
        print(f"[Main] Error during waypoint mission: {e}")
    finally:
        if monitor:
            monitor.stop()


def run_precision_land(vehicle, config):
    from core.safety_monitor import SafetyMonitor
    from core.takeoff_land import TakeoffLand
    from vision.target_detector import TargetDetector
    from vision.precision_land import PrecisionLand

    tl = TakeoffLand(vehicle, config)
    monitor = None
    try:
        # Step 1 — takeoff first
        tl.arm_and_takeoff(target_altitude=15)

        # Step 2 — start safety monitor after airborne
        monitor = SafetyMonitor(vehicle, config)
        monitor.start()

        # Step 3 — start vision
        print("[Main] Drone airborne. Starting vision targeting...")
        print("[Main] Hold a GREEN object in front of your webcam.")
        time.sleep(2)

        detector = TargetDetector(config)
        pl = PrecisionLand(vehicle, detector, config)
        pl.run(camera_index=0, max_duration=300)

    except Exception as e:
        print(f"[Main] Error during precision landing: {e}")
        try:
            tl.land()
        except Exception:
            pass
    finally:
        if monitor:
            monitor.stop()


def run_swarm(config):
    from dronekit_sitl import SITL
    from swarm.swarm_coordinator import SwarmCoordinator

    print("\n[Main] Starting 3 SITL instances for swarm...")
    sitl_instances = []
    ports = [14550, 14560, 14570]

    for i, port in enumerate(ports):
        print(f"  Starting SITL instance {i} on port {port}...")
        sitl = SITL(instance=i)
        sitl.download("copter", "3.3", verbose=False)
        sitl.launch([], await_ready=True, restart=True)
        sitl_instances.append(sitl)
        print(f"  SITL {i} running at {sitl.connection_string()}")
        time.sleep(2)

    time.sleep(5)

    config_copy = config.copy()
    config_copy["sitl_instances"] = [
        {"id": i, "port": port} for i, port in enumerate(ports)
    ]

    swarm = SwarmCoordinator(config_copy)
    swarm.connect_all()

    try:
        if not swarm.vehicles:
            print("[Main] No drones connected — aborting swarm.")
            return

        swarm.arm_all()
        swarm.takeoff_all(altitude=10)
        time.sleep(3)

        for formation in ["line", "triangle", "v_shape", "circle"]:
            print(f"\n[Main] Flying {formation} formation for 15 seconds...")
            swarm.fly_formation(formation, duration=15)
            swarm.print_status()

        swarm.rtl_all()
        time.sleep(10)

    except Exception as e:
        print(f"[Main] Swarm error: {e}")
    finally:
        swarm.disconnect_all()
        for sitl in sitl_instances:
            try:
                sitl.stop()
            except Exception:
                pass
        print("[Main] All SITL instances stopped.")


def run_dashboard(vehicle, config):
    from dashboard.app import create_app

    app, socketio, stream = create_app(vehicle=vehicle, config=config)
    print("\n[Main] Dashboard running at http://localhost:5000")
    print("       Open in your browser to see live telemetry.\n")
    socketio.run(app, host="0.0.0.0", port=5000)


def run_full_mission(vehicle, config):
    from core.safety_monitor import SafetyMonitor
    from core.takeoff_land import TakeoffLand
    from core.waypoint_mission import WaypointMission

    print("\n[Main] === FULL MISSION SEQUENCE ===\n")

    tl = TakeoffLand(vehicle, config)
    monitor = None

    waypoints = [
        {"lat": -35.3633245, "lon": 149.1652373, "alt": 10},
        {"lat": -35.3629641, "lon": 149.1644025, "alt": 15},
        {"lat": -35.3624967, "lon": 149.1650177, "alt": 10},
    ]

    try:
        print("--- Phase 1: Takeoff ---")
        tl.arm_and_takeoff()

        monitor = SafetyMonitor(vehicle, config)
        monitor.start()

        tl.hover(duration=3)

        print("\n--- Phase 2: Waypoint Mission ---")
        wm = WaypointMission(vehicle, config)
        wm.build_mission(waypoints)
        wm.execute()

        print("\n--- Phase 3: RTL ---")
        tl.return_to_launch()
        time.sleep(15)

        print("\n--- Full mission complete ---")

    except KeyboardInterrupt:
        print("\n[Main] Interrupted — landing.")
        tl.land()
    except Exception as e:
        print(f"[Main] Error during full mission: {e}")
    finally:
        if monitor:
            monitor.stop()


# ------------------------------------------------------------------ #
#  CLI Menu                                                            #
# ------------------------------------------------------------------ #

MENU = """
╔══════════════════════════════════════╗
║         AutonomAir  Mission CLI      ║
╠══════════════════════════════════════╣
║  1. Takeoff, Hover & Land            ║
║  2. Waypoint Mission                 ║
║  3. Precision Landing (Vision)       ║
║  4. Swarm Formation Flight           ║
║  5. Live Telemetry Dashboard         ║
║  6. Full Mission Sequence            ║
║  7. Run Tests                        ║
║  0. Exit                             ║
╚══════════════════════════════════════╝
"""


def main():
    config = load_config()

    sitl = start_sitl()
    vehicle = connect_vehicle(sitl.connection_string())

    try:
        while True:
            print(MENU)
            choice = input("Select option: ").strip()

            if choice == "1":
                run_takeoff_land(vehicle, config)

            elif choice == "2":
                run_waypoint_mission(vehicle, config)

            elif choice == "3":
                run_precision_land(vehicle, config)

            elif choice == "4":
                vehicle.close()
                sitl.stop()
                run_swarm(config)
                break

            elif choice == "5":
                run_dashboard(vehicle, config)

            elif choice == "6":
                run_full_mission(vehicle, config)

            elif choice == "7":
                print("[Main] Running tests...")
                os.system("python -m pytest tests/sitl_tests.py -v")

            elif choice == "0":
                print("[Main] Exiting.")
                break

            else:
                print("[Main] Invalid option.")

    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")

    finally:
        try:
            vehicle.close()
            sitl.stop()
            print("[Main] Cleanup complete.")
        except Exception:
            pass


if __name__ == "__main__":
    main()