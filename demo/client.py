from flask import Flask, Response
import cv2

app = Flask(__name__)

def generate_frames():
    cap = cv2.VideoCapture("libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open camera")
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

        _, binary = cv2.threshold(top_hat, 50, 255, cv2.THRESH_BINARY)

        ret, buffer = cv2.imencode('.jpg', binary)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
