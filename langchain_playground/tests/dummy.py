import cv2
import os
import time
import numpy as np
from moviepy.editor import VideoFileClip, concatenate_videoclips

# Setup video capture
cap = cv2.VideoCapture(1)

# Define codec and create VideoWriter objects
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use mp4v for MP4 format
fps = 30
frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# Create an empty list to store video clips
clips = []
start_time = time.time()
clip_duration = 10  # 30 seconds clip duration

def save_temp_video(frames, clip_index):
    temp_filename = f"temp_{clip_index}.mp4"
    out = cv2.VideoWriter(temp_filename, fourcc, fps, frame_size)
    for frame in frames:
        out.write(frame)
    out.release()
    return temp_filename

def merge_videos():
    global clips
    if len(clips) > 0:
        video_clips = [VideoFileClip(clip) for clip in clips]
        final_clip = concatenate_videoclips(video_clips)
        final_clip.write_videofile("merge.mp4", codec="libx264")
        final_clip.close()
        for clip in clips:
            os.remove(clip)  # Clean up temporary files
        clips = []  # Clear the list after merging

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    current_time = time.time()
    
    # Every 30 seconds, save the current clip
    if current_time - start_time >= clip_duration:
        clip_index = int(current_time - start_time) // clip_duration
        frames = []
        frames.append(frame)
        temp_video = save_temp_video(frames, clip_index)
        clips.append(temp_video)
        
        # Merge video clips into merge.mp4
        merge_videos()
        
        start_time = current_time

    # Display the frame (optional)
    cv2.imshow('Camera Feed', frame)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Simulate a read action that requires checking time and prompt
    # For example: If a user requests to access video at a certain time
    # requested_time = current_time  # Replace with actual requested time logic
    # if requested_time >= current_time:
    #     print("The requested time frame is still recording. Please wait.")
    # elif requested_time < current_time:
    #     print("The requested time frame is in the merged video.")
    #     # Perform your action on the merged video (read/processing etc.)

# Release everything when job is done
cap.release()
cv2.destroyAllWindows()
