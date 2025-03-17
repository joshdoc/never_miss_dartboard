import cv2
import time
import argparse

def main(headless: bool):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Request high FPS
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffering to decrease latency
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    prev_time = time.time()
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - prev_time
        
        if elapsed_time >= 1.0:
            print(f"FPS: {frame_count / elapsed_time:.2f}")
            prev_time = current_time
            frame_count = 0
        
        if not headless:
            cv2.imshow("USB Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USB Camera FPS Optimizer")
    parser.add_argument("--headless", action="store_true", help="Run without display window")
    args = parser.parse_args()
    
    main(args.headless)
