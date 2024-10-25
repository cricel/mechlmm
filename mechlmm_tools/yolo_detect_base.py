import cv2
from ultralytics import YOLO
import math

# Load the model
yolo = YOLO('yolov8s.pt')

# Load the video capture
videoCap = cv2.VideoCapture(0)

# Function to get class colors
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [base_colors[color_index][i] + increments[color_index][i] * 
    (cls_num // len(base_colors)) % 256 for i in range(3)]
    return tuple(color)

def is_point_inside_rectangle(px, py, x1, y1, x2, y2):
    return x1 <= px <= x2 and y1 <= py <= y2

def get_center_of_bounding_box(x1, y1, x2, y2):
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    return int(cx), int(cy)

def distance_to_nearest_box_point(px, py, x1, y1, x2, y2):
    # If the point is inside, the distance is 0
    if is_point_inside_rectangle(px, py, x1, y1, x2, y2):
        return 0

    # Calculate the horizontal distance to the nearest x boundary
    if px < x1:
        nearest_x = x1
    elif px > x2:
        nearest_x = x2
    else:
        nearest_x = px  # Point is between x1 and x2

    # Calculate the vertical distance to the nearest y boundary
    if py < y1:
        nearest_y = y1
    elif py > y2:
        nearest_y = y2
    else:
        nearest_y = py  # Point is between y1 and y2

    # Return the Euclidean distance to the nearest point on the box
    distance = math.sqrt((px - nearest_x) ** 2 + (py - nearest_y) ** 2)
    return distance

while True:
    ret, frame = videoCap.read()
    if not ret:
        continue
    results = yolo.track(frame, stream=True)


    for result in results:
        # get the classes names
        classes_names = result.names

        # iterate over each box
        for box in result.boxes:
            # check if confidence is greater than 40 percent
            if box.conf[0] > 0.4:
                # get coordinates
                [x1, y1, x2, y2] = box.xyxy[0]
                # convert to int
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                # get the class
                cls = int(box.cls[0])
                # get the class name
                class_name = classes_names[cls]
                # get the respective colour
                colour = getColours(cls)
                # draw the rectangle
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(frame, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                
                if(classes_names[int(box.cls[0])] == "cup"):
                    object_center = get_center_of_bounding_box(x1, y1, x2, y2)
                    cv2.line(frame, (0,0), object_center, (0, 255, 0), 2)
                    dis = distance_to_nearest_box_point(0, 0, x1, y1, x2, y2)
                    cv2.putText(frame, str(dis), (x1 + 3, y1+30), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the video capture and destroy all windows
videoCap.release()
cv2.destroyAllWindows()
