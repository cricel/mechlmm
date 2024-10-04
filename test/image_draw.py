import cv2

def draw_bounding_box(image_path, start_point, end_point, color=(0, 255, 0), thickness=2):
    # Load the image
    image = cv2.imread(image_path)
    
    # Draw a rectangle (bounding box) on the image
    cv2.rectangle(image, start_point, end_point, color, thickness)
    
    # Show the image with bounding box
    cv2.imshow("Image with Bounding Box", image)
    
    # Wait for any key press to close the image window
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Optionally save the image with the bounding box
    # cv2.imwrite('output_with_bounding_box.jpg', image)

# Example usage
image_path = 'test.jpg'  # Replace with your image path
start_point = (int(100), int(580))         # Top-left corner of the bounding box (x, y)
end_point = (int(998), int(999))       # Bottom-right corner of the bounding box (x, y)

draw_bounding_box(image_path, start_point, end_point)
