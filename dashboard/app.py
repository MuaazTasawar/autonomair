import json
import os
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
from .telemetry_stream import TelemetryStream


def create_app(vehicle=None, config=None):
    """
    Factory function that creates and returns the Flask app and SocketIO instance.

    Args:
        vehicle : DroneKit vehicle object (can be None for UI-only testing)
        config  : mission config dict

    Returns:
        (app, socketio, stream) tuple
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    app = Flask(__name__, template_folder=os.path.join(base_dir, "templates"))
    app.config["SECRET_KEY"] = "autonomair_secret"

    socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")
    stream = TelemetryStream(vehicle, socketio) if vehicle else None

    # ------------------------------------------------------------------ #
    #  Routes                                                              #
    # ------------------------------------------------------------------ #

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/api/telemetry")
    def telemetry_snapshot():
        if not stream:
            return jsonify({"error": "No vehicle connected"}), 503
        return jsonify(stream.snapshot())

    @app.route("/api/status")
    def status():
        return jsonify({
            "status": "online",
            "vehicle_connected": vehicle is not None,
            "streaming": stream._running if stream else False,
        })

    # ------------------------------------------------------------------ #
    #  Socket.IO events                                                    #
    # ------------------------------------------------------------------ #

    @socketio.on("connect")
    def on_connect():
        print("[Dashboard] Client connected.")
        if stream and not stream._running:
            stream.start()

    @socketio.on("disconnect")
    def on_disconnect():
        print("[Dashboard] Client disconnected.")

    @socketio.on("command")
    def on_command(data):
        """
        Accept simple commands from the dashboard UI.
        Payload: { "action": "rtl" | "land" | "arm" | "disarm" }
        """
        if not vehicle:
            return

        from dronekit import VehicleMode
        action = data.get("action", "")

        if action == "rtl":
            vehicle.mode = VehicleMode("RTL")
            print("[Dashboard] Command: RTL")
        elif action == "land":
            vehicle.mode = VehicleMode("LAND")
            print("[Dashboard] Command: LAND")
        elif action == "arm":
            vehicle.armed = True
            print("[Dashboard] Command: ARM")
        elif action == "disarm":
            vehicle.armed = False
            print("[Dashboard] Command: DISARM")
        else:
            print(f"[Dashboard] Unknown command: {action}")

    return app, socketio, stream