import cv2
import time
from datetime import datetime

# Function to get the current timestamp for the file name
def get_timestamp():
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# Initialize the video capture (0 is the default camera)
cap = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the frame width and height (optional)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 20  # You can adjust the FPS

# Start time for 20-second intervals
start_time = time.time()

# Initialize video writer
out = None

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Couldn't capture video frame.")
        break

    # Get the current time
    current_time = time.time()

    # If 20 seconds have passed or no writer is initialized, create a new file
    if out is None or (current_time - start_time) >= 10:
        if out is not None:
            out.release()  # Release the previous file before creating a new one

        # Get the timestamp and create a new file with timestamped name
        timestamp = get_timestamp()
        file_name = f"{timestamp}.mp4"

        # Define the codec and create a VideoWriter object for mp4
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for mp4
        out = cv2.VideoWriter(file_name, fourcc, fps, (frame_width, frame_height))

        print(f"Recording started: {file_name}")
        start_time = current_time  # Reset start time for the next 20 seconds

    # Write the frame to the file
    out.write(frame)

    # Display the frame in a window
    cv2.imshow('Camera View', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when the job is finished
cap.release()
if out is not None:
    out.release()
cv2.destroyAllWindows()
