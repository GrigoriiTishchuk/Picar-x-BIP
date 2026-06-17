from picamera2 import Picamera2
from time import strftime, localtime
from flask import Flask, Response, jsonify
import threading
import cv2

# define date and time
DATE_TIME = strftime("%y%m%d_%H%M%S", localtime())

# path were images are stored
SAVE_PATH = f"/home/admin/Pictures/{DATE_TIME}.jpg"

# port
PORT = 9000

app = Flask(__name__)
camera = None
latest_frame = None
frame_lock = threading.Lock()

# ── Camera setup ──────────────────────────────────────────────────────────────

def start_camera():
    global camera
    camera = Picamera2()
    config = camera.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"}
    )
    camera.configure(config)
    camera.start()

def capture_loop():
    """Continuously grab frames into latest_frame."""
    global latest_frame
    while True:
        frame = camera.capture_array()
        with frame_lock:
            latest_frame = frame

# ── Routes ────────────────────────────────────────────────────────────────────

def generate_mjpeg():
    """Yield frames as a multipart MJPEG stream."""
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
        _, jpeg = cv2.imencode(".jpg", frame)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg.tobytes()
            + b"\r\n"
        )

@app.route("/mjpg")
def mjpeg_feed():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/screenshot")
def screenshot():
    with frame_lock:
        if latest_frame is None:
            return jsonify({"error": "No frame available"}), 503
        frame = latest_frame.copy()
    DATE_TIME = strftime("%y%m%d_%H%M%S", localtime())
    path = f"{SAVE_DIR}{DATE_TIME}.jpg"
    cv2.imwrite(path, frame)
    print(f"Screenshot saved to: {path}")
    return jsonify({"saved": path})

@app.route("/")
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>PiCar-X Camera</title>
        <style>
            body { background: #111; display: flex; flex-direction: column;
                   align-items: center; justify-content: center;
                   height: 100vh; margin: 0; color: #eee; font-family: sans-serif; }
            img  { border: 2px solid #444; border-radius: 4px; }
            p    { margin-top: 12px; font-size: 14px; color: #888; }
            #msg { margin-top: 8px; font-size: 13px; color: #6f6; min-height: 18px; }
        </style>
    </head>
    <body>
        <img src="/mjpg" width="1280" height="720">
        <p>Press <strong>S</strong> to take a screenshot &nbsp;|&nbsp; stream: /mjpg</p>
        <div id="msg"></div>
        <script>
            document.addEventListener("keydown", (e) => {
                if (e.key.toLowerCase() !== "s") return;
                fetch("/screenshot")
                    .then(r => r.json())
                    .then(d => {
                        document.getElementById("msg").textContent =
                            d.saved ? "Saved: " + d.saved : "Error: " + d.error;
                    });
            });
        </script>
    </body>
    </html>
    """

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    start_camera()

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    print(f"Open http://<your-pi-ip>:{PORT}/ in your browser.")
    print("Press S on the page to take a screenshot. Ctrl+C to quit.")
    app.run(host="0.0.0.0", port=PORT)