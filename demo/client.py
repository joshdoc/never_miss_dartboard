from flask import Flask, Response, render_template_string
import cv2

app = Flask(__name__)

def generate_frames():
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

        # Resize and pre-process the frame
        frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)

        eroded = cv2.erode(frame, kernel)
        dilated = cv2.dilate(eroded, kernel)
        top_hat = cv2.subtract(frame, dilated)

        # Apply threshold for binary image
        _, binary = cv2.threshold(top_hat, 50, 255, cv2.THRESH_BINARY)

        ret, buffer = cv2.imencode('.jpg', binary)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# HTML template to center the video stream on the webpage
HTML_TEMPLATE = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Binary Video Stream</title>
  <style>
    body {
      background-color: #222;
      color: #eee;
      display: flex;
      align-items: center;
      justify-content: center;
      height: 100vh;
      margin: 0;
    }
    img {
      border: 2px solid #eee;
      box-shadow: 0 0 10px rgba(255,255,255,0.5);
    }
  </style>
</head>
<body>
  <img src="/video_feed" alt="Binary Stream">
</body>
</html>
"""

@app.route('/')
def index():
    # Render the main HTML page that centers the video stream
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    # Return the MJPEG stream
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Run the Flask server on all network interfaces at port 8080
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
