from flask import Flask, Response, render_template_string
import cv2
import numpy as np
import threading

app = Flask(__name__)

# Global variable to hold the latest centroid text
latest_centroid_text = "Centroid: N/A"

# A lock to ensure thread-safe updates to latest_centroid_text
centroid_lock = threading.Lock()

def generate_frames():
    global latest_centroid_text
    cap = cv2.VideoCapture("libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1", cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera stream.")
        return

    scale_factor = 0.4
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Preprocess the frame: resize, blur, morphological operations
        frame_resized = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        blurred = cv2.GaussianBlur(frame_resized, (3, 3), 0)
        eroded = cv2.erode(blurred, kernel)
        dilated = cv2.dilate(eroded, kernel)
        top_hat = cv2.subtract(blurred, dilated)

        # Apply threshold to get a binary image
        _, binary = cv2.threshold(top_hat, 50, 255, cv2.THRESH_BINARY)

        # --- CENTROID DETECTION ---
        contours, _ = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

        # Update the global centroid text in a thread-safe way
        with centroid_lock:
            latest_centroid_text = f"Centroid: {centroid}"

        # For display, you might want to resize the binary image to a fixed display resolution.
        binary_display = cv2.resize(binary, (1280, 720), interpolation=cv2.INTER_LINEAR)

        ret, buffer = cv2.imencode('.jpg', binary_display)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# HTML template with a two-column layout: video feed on the left and text feed on the right.
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
      background-color: #222;
      font-family: Arial, sans-serif;
      color: #eee;
    }
    .main-container {
      display: flex;
      height: 100%;
      justify-content: center;
      align-items: center;
    }
    .box {
      background-color: #333;
      padding: 20px;
      margin: 10px;
      border: 2px solid #eee;
      box-shadow: 0 0 10px rgba(255,255,255,0.5);
    }
    .video-box img {
      display: block;
      max-width: 100%;
      height: auto;
    }
    .centroid-box {
      width: 300px;
      word-wrap: break-word;
      text-align: left;
    }
    h1, h2 {
      text-align: center;
    }
  </style>
</head>
<body>
  <div class="main-container">
    <div class="box video-box">
      <h1>Binary Video Feed</h1>
      <img src="/video_feed" alt="Binary Stream">
    </div>
    <div class="box centroid-box">
      <h2>Centroid Feed</h2>
      <div id="centroidText" style="font-size:1.5em; margin-top:20px;">Loading...</div>
    </div>
  </div>
  <script>
    // Poll the /centroid endpoint every 500ms to update the centroid text.
    function updateCentroid() {
      fetch('/centroid')
        .then(response => response.text())
        .then(text => {
          document.getElementById('centroidText').innerText = text;
        })
        .catch(err => console.error(err));
    }
    setInterval(updateCentroid, 500);
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
    with centroid_lock:
        text = latest_centroid_text
    return text

if __name__ == '__main__':
    # Run Flask with threading enabled so multiple requests can be handled.
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

