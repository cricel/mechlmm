import cv2
import time
import os
from datetime import datetime

def save_video_from_camera():
    # Remove old video files
    video_files = [f for f in os.listdir() if f.startswith('output_video_') and f.endswith('.mp4')]
    for file in video_files:
        os.remove(file)
        print(f"Deleted old video file: {file}")

    # Open the default camera (usually the webcam)
    cap = cv2.VideoCapture(1)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Frame size (width and height)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Define the codec for MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 'mp4v' is commonly used for MP4 files
    
    # Start capturing video
    start_time = time.time()
    
    # Initialize video_writer to None
    video_writer = None

    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Could not read frame.")
            break

        # Get the current time
        current_time = time.time()
        
        # Save a new video file every 10 seconds
        if current_time - start_time >= 10 or video_writer is None:
            if video_writer is not None:
                # Release the previous video writer
                video_writer.release()
            
            # Get the current timestamp and format it for the filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_filename = f'output_video_{timestamp}.mp4'  # Use timestamp as filename
            
            # Create a new VideoWriter object
            video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (frame_width, frame_height))
            start_time = current_time
            print(f"Started recording: {video_filename}")
        
        # Write the frame to the video file
        if video_writer is not None:
            video_writer.write(frame)
        
        # Display the frame (optional)
        cv2.imshow('Camera Feed', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release everything if the job is finished
    cap.release()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()

save_video_from_camera()
