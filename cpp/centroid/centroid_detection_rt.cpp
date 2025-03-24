import cv2
import os
import numpy as np
import time

def process_frames_tophat_downsampled(input_folder, output_folder, 
                                      kernel_size=(15, 15),  # Adjust as needed
                                      threshold_value=30,
                                      scale_factor=0.5):  # Downsampling factor
    """
    Uses a morphological top-hat filter to isolate small bright features 
    from a darker or uniform background, applied to downsampled images.
    
    Parameters:
        input_folder (str): Folder containing .png frames.
        output_folder (str): Folder where results are saved.
        kernel_size (tuple): Size of structuring element for top-hat.
        threshold_value (int): Threshold to isolate bright top-hat regions.
        scale_factor (float): Factor by which to downscale images.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    frame_files = sorted(f for f in os.listdir(input_folder) if f.endswith(".png"))
    centroids = []
    
    # Create a structuring element (ellipse often works well for organic shapes)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
    
    total_time = 0
    for frame_file in frame_files:
        frame_path = os.path.join(input_folder, frame_file)
        frame = cv2.imread(frame_path)
        if frame is None:
            continue
        
        start_time = time.time()
        
        t1 = time.time()
        # Downsample the image
        frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
        t2 = time.time()
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t3 = time.time()
        
        # Apply a slight Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        t4 = time.time()
        
        # Morphological Top-Hat to isolate small bright regions
        top_hat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
        t5 = time.time()
        
        # Threshold the top-hat result
        _, binary = cv2.threshold(top_hat, threshold_value, 255, cv2.THRESH_BINARY)
        t6 = time.time()
        
        # Optional morphological close to fill small holes
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        t7 = time.time()
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        t8 = time.time()
        
        if contours:
            # Pick the largest contour (assuming it's the dart)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Compute centroid using moments
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))
                
                # Draw a red circle on the original frame at the centroid
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        t9 = time.time()
        
        # Save the binary (top-hat + threshold) image
        binary_path = os.path.join(output_folder, frame_file)
        cv2.imwrite(binary_path, binary)
        
        # Save the marked original frame
        marked_path = os.path.join(output_folder, "marked_" + frame_file)
        cv2.imwrite(marked_path, frame)
        
        end_time = time.time()
        frame_time = end_time - start_time
        total_time += frame_time
        
        print(f"Processed {frame_file} in {frame_time:.4f} seconds")
        print(f"  Downsampling: {(t2 - t1):.4f} sec")
        print(f"  Grayscale conversion: {(t3 - t2):.4f} sec")
        print(f"  Gaussian Blur: {(t4 - t3):.4f} sec")
        print(f"  Top-Hat transform: {(t5 - t4):.4f} sec")
        print(f"  Thresholding: {(t6 - t5):.4f} sec")
        print(f"  Morphological closing: {(t7 - t6):.4f} sec")
        print(f"  Contour detection: {(t8 - t7):.4f} sec")
        print(f"  Centroid calculation & marking: {(t9 - t8):.4f} sec")
    
    avg_time = total_time / len(frame_files) if frame_files else 0
    print(f"Average processing time per frame: {avg_time:.4f} seconds")
    print("Centroids computed:", centroids)
    return centroids

# Example usage:
if __name__ == "__main__":
    input_folder = "frames_output"      # Folder with your .png frames
    output_folder = "binary_frames"     # Folder to save results
    process_frames_tophat_downsampled(input_folder, output_folder, 
                                      kernel_size=(12, 12), 
                                      threshold_value=8,
                                      scale_factor=0.4)

