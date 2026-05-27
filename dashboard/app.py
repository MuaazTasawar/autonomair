import json
import os
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
from .telemetry_stream import TelemetryStream


def create_app(vehicle=None, config=None):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    app = Flask(__name__, template_folder=os.path.join(base_dir, "templates"))
    app.config["SECRET_KEY"] = "autonomair_secret"

    # Use gevent instead of eventlet — eventlet breaks on Python 3.13
    socketio = SocketIO(app, cors_allowed_origins="*", async_mode="gevent")
    stream = TelemetryStream(vehicle, socketio) if vehicle else None

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
        if not vehicle:
            return
        from dronekit import VehicleMode
        from pymavlink import mavutil
        action = data.get("action", "")
        mode_map = {"rtl": 6, "land": 9}
        if action in mode_map:
            vehicle._master.mav.set_mode_send(
                vehicle._master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_map[action]
            )
            print(f"[Dashboard] Command: {action.upper()}")
        elif action == "arm":
            vehicle.armed = True
            print("[Dashboard] Command: ARM")
        elif action == "disarm":
            vehicle.armed = False
            print("[Dashboard] Command: DISARM")

    return app, socketio, stream