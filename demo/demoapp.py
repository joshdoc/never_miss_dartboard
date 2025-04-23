/**
 * Flask application to stream binary video frames from a Raspberry Pi camera and display centroid history, used during Expo demonstration.
 *
 * This web app captures grayscale frames via a GStreamer pipeline, processes them to detect the largest
 * bright object's centroid, and serves both the binary video feed and centroid coordinates over HTTP on a local network.
 *
 * The web interface shows the binary video and a scrolling list of the last 15 centroids.
 *
 * USAGE:
 *   python demoapp.py
 *
 * Authors: Jacob Blosser, Jacob Dell, Joshua Doctor, William Hurley, Jacob Root
 * Date: 04-13-2025
 */

from flask import Flask, Response, render_template_string
import cv2
import numpy as np
import threading

app = Flask(__name__)
app.jinja_env.cache = {}

centroid_history = []
history_lock = threading.Lock()

def generate_frames():
    global centroid_history
    cap = cv2.VideoCapture(
        "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! "
        "videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1",
        cv2.CAP_GSTREAMER
    )
    if not cap.isOpened():
        print("Failed to open camera stream.")
        return

    scale_factor = 0.7
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_resized = cv2.resize(
            frame, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST
        )
        blurred = cv2.GaussianBlur(frame_resized, (3, 3), 0)
        eroded = cv2.erode(blurred, kernel)
        dilated = cv2.dilate(eroded, kernel)
        top_hat = cv2.subtract(blurred, dilated)

        _, binary = cv2.threshold(top_hat, 50, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(
            binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        max_area = 0
        best_moments = {}
        centroid = (-1, -1)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_moments = cv2.moments(cnt)
        if best_moments.get("m00", 0) != 0:
            cx = int(best_moments["m10"] / best_moments["m00"] / scale_factor)
            cy = int(best_moments["m01"] / best_moments["m00"] / scale_factor)
            centroid = (cx, cy)
        else:
            centroid = (-1, -1)

        if centroid != (-1, -1):
            with history_lock:
                centroid_history.append(centroid)
                if len(centroid_history) > 15:
                    centroid_history = centroid_history[-15:]

        ret, buffer = cv2.imencode('.jpg', binary)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
        )

HTML_TEMPLATE = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Binary Video and Centroid Feed</title>
  <style>
    html, body {
      height: 100%;
      margin: 0;
      padding: 0;
      background-color: #FFFDE7;
      font-family: 'Arial', sans-serif;
      color: #00274C;
    }

    .main-container {
      display: flex;
      flex-direction: row;
      justify-content: center;
      align-items: center;
      height: 100vh;
      gap: 40px;
    }

    .video-container {
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .video-header {
      text-align: center;
      margin-bottom: 20px;
    }

    .video-header h1 {
      font-size: 3em;
      font-weight: bold;
      margin: 0;
      color: #00274C;
    }

    .video-header p {
      margin: 10px 0 0 0;
      font-size: 1em;
      color: #00274C;
    }

    .video-feed-box {
      border: 3px solid #FFCB05;
      border-radius: 8px;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.15);
    }

    .video-feed-box img {
      display: block;
      max-width: 100%;
      height: auto;
      border-radius: 8px;
    }

    .centroid-container {
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .centroid-header {
      font-size: 3em;
      font-weight: bold;
      margin-bottom: 10px;
      color: #00274C;
    }

    .centroid-box {
      width: 320px;
      background-color: #E6EEF5;
      padding: 12px;
      border: 3px solid #FFCB05;
      border-radius: 8px;
      font-size: 1em;
      min-height: 120px;
      max-height: 400px;
      overflow-y: auto;
      color: #00274C;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.15);
    }

    .centroid-entry {
      margin: 4px 0;
    }
  </style>
</head>
<body>
  <div class="main-container">
    <div class="video-container">
      <div class="video-header">
        <h1>Binary Video Feed</h1>
        <p>Disclaimer: This is a Python script for demo purposes.</p>
      </div>
      <div class="video-feed-box">
        <img src="/video_feed" alt="Binary Stream">
      </div>
    </div>
    <div class="centroid-container">
      <div class="centroid-header">Centroid Feed</div>
      <div class="centroid-box" id="centroidBox">Loading...</div>
    </div>
  </div>

  <script>
    function updateCentroid() {
      fetch('/centroid')
        .then(response => response.text())
        .then(text => {
          document.getElementById('centroidBox').innerHTML = text;
        })
        .catch(err => console.error(err));
    }
    setInterval(updateCentroid, 100);
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/centroid')
def centroid_feed():
    with history_lock:
        valid_entries = [f"<div class='centroid-entry'>{c}</div>" for c in centroid_history]
        result = "".join(valid_entries) if valid_entries else ""
    return result

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
