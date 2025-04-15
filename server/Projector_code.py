import cv2
import numpy as np

#Setup
SCREEN_WIDTH, SCREEN_HEIGHT = 1920, 1080

# Load and prepare dartboard image
dartboard = cv2.imread("target_large_bullseye.png", cv2.IMREAD_UNCHANGED)
DARTBOARD_SIZE = 435
dartboard = cv2.resize(dartboard, (DARTBOARD_SIZE, DARTBOARD_SIZE))
dartboard = cv2.cvtColor(dartboard, cv2.COLOR_BGRA2RGBA)

#Distortion Correction
image_width, image_height = 640, 640
rect = np.array([[0, 0], [image_width, 0], 
                 [0, image_height], [image_width, image_height]], dtype=np.float32)

theta = 15  # Projector tilt angle in degrees
y_shift = 15  # Vertical displacement due to tilt
trapezoid = np.array([[20, y_shift], [630, 0], 
                      [10, image_height - y_shift], [640, image_height]], dtype=np.float32)

H_distortion = cv2.getPerspectiveTransform(rect, trapezoid)
warped_dartboard = cv2.warpPerspective(dartboard, H_distortion, (image_width, image_height))
warped_height, warped_width = warped_dartboard.shape[:2]

#Position Prediction Setup
H_position = np.array([[ 1.07581897e+01,  1.18900969e+00,  9.48182841e+02],
                       [ 2.77725678e-01, -7.43562709e+00,  5.40503774e+02],
                       [ 4.55825948e-04,  9.93956939e-04,  1.00000000e+00]])

def real_to_projector(real_pos):
    point = np.array([[[real_pos[0], real_pos[1]]]], dtype=np.float32)
    projected = cv2.perspectiveTransform(point, H_position)[0][0]
    return int(projected[0]), int(projected[1])

# Initial real-world position (in cm) and its projector coordinates
current_real = np.array([5.5*2.54, (56-50)*2.54])
current_prediction = real_to_projector(current_real)

def handle_kf_prediction(real_x_cm, real_y_cm):
    global current_prediction
    current_prediction = real_to_projector((real_x_cm, real_y_cm))


def draw_dartboard(center_pos):
    
    frame = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 3), dtype=np.uint8)
    
    
    x = center_pos[0] - warped_width // 2
    y = center_pos[1] - warped_height // 2

    
    if 0 <= x < SCREEN_WIDTH - warped_width and 0 <= y < SCREEN_HEIGHT - warped_height:
        # Extract region of interest (ROI) from the frame
        roi = frame[y:y+warped_height, x:x+warped_width]
        # Normalize the alpha channel to the range [0,1]
        alpha = warped_dartboard[:, :, 3] / 255.0
        # Blend each channel
        for c in range(0, 3):
            roi[:, :, c] = (warped_dartboard[:, :, c] * alpha + roi[:, :, c] * (1 - alpha)).astype(np.uint8)
    
    
    cv2.putText(frame, f"Projector: {center_pos}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return frame

#Main Loop
cv2.namedWindow("Projection", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Projection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while True:
    
    handle_kf_prediction(current_real[0], current_real[1])
    
    
    frame = draw_dartboard(current_prediction)
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    cv2.imshow("Projection", frame_bgr)
    
    
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
