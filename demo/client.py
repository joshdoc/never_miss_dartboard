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

        frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        eroded = cv2.erode(frame, kernel)
        dilated = cv2.dilate(eroded, kernel)
        top_hat = cv2.subtract(frame, dilated)
    
        _, binary = cv2.threshold(top_hat, 20, 255, cv2.THRESH_BINARY)
        
        binary_resized = cv2.resize(binary, (1280, 720), interpolation=cv2.INTER_LINEAR)
        ret, buffer = cv2.imencode('.jpg', binary_resized)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

HTML_TEMPLATE = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Binary Video Stream</title>
  <style>
    html, body {
      height: 100%;
      margin: 0;
      padding: 0;
    }
    .container {
      background-color: #222;
      color: #eee;
      height: 100%;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      font-family: Arial, sans-serif;
      text-align: center;
    }
    h1 {
      margin-bottom: 20px;
      font-size: 2.5em;
    }
    img {
      border: 2px solid #eee;
      box-shadow: 0 0 10px rgba(255,255,255,0.5);
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Binary Video Feed</h1>
    <img src="/video_feed" alt="Binary Stream">
  </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
