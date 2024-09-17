import cv2
import os
from datetime import datetime, timedelta

def query_video_frame(requested_time):
    # List all video files in the current directory (assuming they are stored here)
    video_files = sorted([f for f in os.listdir() if f.startswith('output_video_') and f.endswith('.mp4')])
    
    # Parse timestamps from the filenames and sort them
    timestamps = []
    for filename in video_files:
        try:
            # Extract the timestamp part of the filename
            timestamp_str = filename.replace('output_video_', '').replace('.mp4', '')
            timestamp = datetime.strptime(timestamp_str, '%Y%m%d_%H%M%S')
            timestamps.append((timestamp, filename))
        except ValueError:
            print(f"Error parsing timestamp from filename: {filename}")
            continue
    
    # Sort video files by timestamp
    timestamps.sort(key=lambda x: x[0])
    
    # Find the correct video and the frame within that video
    for i in range(len(timestamps)):
        video_start_time = timestamps[i][0]
        
        # Calculate the video end time
        if i + 1 < len(timestamps):
            video_end_time = timestamps[i + 1][0]
        else:
            video_end_time = video_start_time + timedelta(seconds=10)
        
        # Check if the requested time falls within this video
        video_relative_time = requested_time - (video_start_time - timestamps[0][0]).total_seconds()
        if 0 <= video_relative_time < 10:
            video_filename = timestamps[i][1]
            frame_time_in_video = video_relative_time
            break
    else:
        print(f"No video file found for the requested time: {requested_time} seconds")
        return None
    
    # Open the video file
    cap = cv2.VideoCapture(video_filename)
    
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_filename}")
        return None
    
    # Get the frames per second (fps) of the video
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Calculate the frame number to capture
    frame_number = int(fps * frame_time_in_video)
    
    # Set the video frame position
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    
    # Read the frame
    ret, frame = cap.read()
    
    if not ret:
        print(f"Error: Could not read frame at {requested_time} seconds from video {video_filename}")
        cap.release()
        return None
    
    # Release the video capture object
    cap.release()
    
    return frame

def frame_to_jpg(frame, filename):
    # Save the frame as a JPG file
    cv2.imwrite(filename, frame)
    print(f"Frame saved as {filename}")

# Example usage
frame = query_video_frame(16)
if frame is not None:
    frame_to_jpg(frame, 'queried_frame_22.jpg')
    # # Display the frame (optional)
    # cv2.imshow('Queried Frame', frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
