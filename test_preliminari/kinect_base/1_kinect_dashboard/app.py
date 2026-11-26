"""Kinect v1 dashboard: Flask + Socket.IO server.

Run:  python app.py            then open http://localhost:5000
"""
import argparse
import threading
import time

from flask import Flask, Response, jsonify, render_template, request
from flask_socketio import SocketIO

import noise_filter
from kinect_audio import MicArray, list_audio_devices
from kinect_streams import LED_OPTIONS, TILT_MAX, TILT_MIN, KinectManager

NAMESPACE = "/data"

app = Flask(__name__)
app.config["SECRET_KEY"] = "kinect-dashboard"
socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")

state = {"kinect": None, "mic": None, "mic_last": None, "mic_status": {"available": False, "error": "disabled"}}


def emit(event, data):
    socketio.emit(event, data, namespace=NAMESPACE)


def full_status():
    kinect = state["kinect"]
    st = kinect.status() if kinect else {"connected": False, "error": "not started"}
    mic = state["mic"]
    st["mic"] = dict(state["mic_status"])
    st["mic"]["hz"] = round(mic.fps, 1) if mic else 0.0
    last = state["mic_last"] or {}
    st["mic"]["angle"] = last.get("angle")
    st["mic"]["confidence"] = last.get("confidence")
    st["usb"] = noise_filter.snapshot()
    return st


# ------------------------------------------------------------------ pages
@app.route("/")
def index():
    return render_template("index.html", tilt_min=TILT_MIN, tilt_max=TILT_MAX, leds=list(LED_OPTIONS))


def mjpeg_response(stream):
    return Response(stream.mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-cache, no-store, must-revalidate"})


@app.route("/video_rgb")
def video_rgb():
    return mjpeg_response(state["kinect"].rgb)


@app.route("/video_depth")
def video_depth():
    return mjpeg_response(state["kinect"].depth)


@app.route("/video_ir")
def video_ir():
    return mjpeg_response(state["kinect"].ir)


# ------------------------------------------------------------------ REST API
def api_error(exc, code=400):
    return jsonify({"ok": False, "error": str(exc)}), code


@app.route("/api/status")
def api_status():
    return jsonify(full_status())


@app.route("/api/tilt", methods=["GET", "POST"])
def api_tilt():
    kinect = state["kinect"]
    if request.method == "GET":
        return jsonify({"ok": True, "tilt": kinect.tilt_deg, "target": kinect.tilt_target,
                        "status": kinect.tilt_status})
    body = request.get_json(silent=True) or {}
    try:
        angle = kinect.set_tilt(body.get("angle", 0))
    except ValueError as exc:
        return api_error(exc)
    except RuntimeError as exc:
        return api_error(exc, 503)
    return jsonify({"ok": True, "target": angle})


@app.route("/api/led", methods=["POST"])
def api_led():
    body = request.get_json(silent=True) or {}
    try:
        led = state["kinect"].set_led(str(body.get("state", "")))
    except ValueError as exc:
        return api_error(exc)
    except RuntimeError as exc:
        return api_error(exc, 503)
    return jsonify({"ok": True, "state": led})


@app.route("/api/video_mode", methods=["POST"])
def api_video_mode():
    body = request.get_json(silent=True) or {}
    try:
        mode = state["kinect"].set_video_mode(str(body.get("mode", "")))
    except ValueError as exc:
        return api_error(exc)
    except RuntimeError as exc:
        return api_error(exc, 409)
    return jsonify({"ok": True, "mode": mode})


# ------------------------------------------------------------------ Socket.IO
@socketio.on("connect", namespace=NAMESPACE)
def on_connect():
    socketio.emit("status", full_status(), namespace=NAMESPACE, to=request.sid)


def status_loop():
    while True:
        emit("status", full_status())
        time.sleep(1.0)


def on_mic(data):
    state["mic_last"] = data
    emit("mic_direction", data)


def on_mic_status(st):
    state["mic_status"] = st


# ------------------------------------------------------------------ main
def main():
    parser = argparse.ArgumentParser(description="Kinect v1 web dashboard")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--index", type=int, default=0, help="Kinect device index")
    parser.add_argument("--accel-hz", type=float, default=100.0, help="accelerometer polling rate")
    parser.add_argument("--no-audio", action="store_true", help="disable microphone array")
    parser.add_argument("--audio-device", default=None,
                        help="PortAudio device index or name substring (default: auto-detect 'Kinect')")
    parser.add_argument("--noise-gate", type=float, default=-55.0, help="mic noise gate in dBFS")
    parser.add_argument("--video-mode", choices=("rgb", "ir"), default="rgb",
                        help="initial video stream (RGB and IR share the same USB endpoint)")
    parser.add_argument("--list-audio", action="store_true", help="list audio input devices and exit")
    parser.add_argument("--verbose", action="store_true",
                        help="show raw libfreenect/ALSA/JACK messages (disables the stderr filter)")
    args = parser.parse_args()

    if args.list_audio:
        list_audio_devices()
        return

    # must run before libfreenect / PortAudio are loaded
    noise_filter.install(verbose=args.verbose)

    kinect = KinectManager(index=args.index, accel_hz=args.accel_hz, video_mode=args.video_mode, on_accel=lambda d: emit("accel_data", d))
    state["kinect"] = kinect
    kinect.start()

    if not args.no_audio:
        mic = MicArray(on_mic, device=args.audio_device, noise_gate_db=args.noise_gate, on_status=on_mic_status)
        state["mic"] = mic
        mic.start()

    threading.Thread(target=status_loop, name="status", daemon=True).start()
    print(f"[app] dashboard on http://localhost:{args.port}  (Ctrl+C to quit)")
    try:
        try:
            socketio.run(app, host=args.host, port=args.port, allow_unsafe_werkzeug=True)
        except TypeError:  # older flask-socketio without allow_unsafe_werkzeug
            socketio.run(app, host=args.host, port=args.port)
    except KeyboardInterrupt:
        pass
    finally:
        print("[app] shutting down...")
        if state["mic"]:
            state["mic"].stop()
        kinect.stop()


if __name__ == "__main__":
    main()
