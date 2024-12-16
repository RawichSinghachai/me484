import cv2
import numpy as np

clicked_pos = None


def hsv_color(h, s, v):
    h = round(h / 2)
    s = round((s * 255) / 100)
    v = round((v * 255) / 100)
    return h, s, v


def get_color(event, x, y, flags, param):
    global clicked_pos
    if event == cv2.EVENT_LBUTTONDOWN:  # Check if left mouse button is clicked
        clicked_pos = (x, y)  # Update the position of the last click


# Open the webcam
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

# Create a box to display the selected color
color_box = np.zeros((100, 200, 3), dtype=np.uint8)

if not cap.isOpened():
    print("Error: Could not open the webcam.")
    exit()

# Create a window to display the image
cv2.namedWindow('Color Detection')
cv2.setMouseCallback("Color Detection", get_color)

# Set the desired window size
window_width = 640  # Desired width
window_height = 480  # Desired height
cv2.resizeWindow('Color Detection', window_width, window_height)

# Callback function for the trackbars (does nothing, required by OpenCV)
def nothing(x):
    pass

# Create trackbars for lower and upper HSV values
cv2.createTrackbar('blur', 'Color Detection', 0, 20, nothing)
cv2.createTrackbar('Lower H', 'Color Detection', 0, 180, nothing)
cv2.createTrackbar('Lower S', 'Color Detection', 50, 255, nothing)
cv2.createTrackbar('Lower V', 'Color Detection', 50, 255, nothing)
cv2.createTrackbar('Upper H', 'Color Detection', 100, 180, nothing)
cv2.createTrackbar('Upper S', 'Color Detection', 100, 255, nothing)
cv2.createTrackbar('Upper V', 'Color Detection', 100, 255, nothing)

# Main loop
while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read a frame from the webcam.")
        break

    # If there's a clicked position, display it
    if clicked_pos:
        x, y = clicked_pos
        pixel_text = f"Pos: ({x}, {y})"
        cv2.putText(frame, pixel_text, (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # Get the current positions of all trackbars
    lower_h = cv2.getTrackbarPos('Lower H', 'Color Detection')
    lower_s = cv2.getTrackbarPos('Lower S', 'Color Detection')
    lower_v = cv2.getTrackbarPos('Lower V', 'Color Detection')
    upper_h = cv2.getTrackbarPos('Upper H', 'Color Detection')
    upper_s = cv2.getTrackbarPos('Upper S', 'Color Detection')
    upper_v = cv2.getTrackbarPos('Upper V', 'Color Detection')
    # Convert to odd number for GaussianBlur
    blur_value = cv2.getTrackbarPos('blur', 'Color Detection') * 2 + 1  

    # Define lower and upper bounds for HSV
    lower_bound = np.array((lower_h, lower_s, lower_v))
    upper_bound = np.array((upper_h, upper_s, upper_v))

    # Apply Gaussian blur to reduce noise
    blurred_frame = cv2.GaussianBlur(frame, (blur_value, blur_value), 0)

    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the color using the trackbar values
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a copy of the original image to draw contours
    result = frame.copy()

    count_objects = 0  # Initialize object count

    for contour in contours:
        # Filter out small contours based on area
        if cv2.contourArea(contour) > 500:
            count_objects += 1  # Increment object count

            # Approximate the shape of the contour
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Draw the contour
            cv2.drawContours(result, [approx], -1, (0, 255, 0), 2)

            # Calculate the center of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.putText(result, str(f"{cx, cy}"), (cx, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Display the count of detected objects
    cv2.putText(result, f"Objects Detected : {count_objects}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Show the original frame with contours and shapes
    cv2.imshow('Color Detection', result)

    # Handle the key press event for "i"
    key = cv2.waitKey(1) & 0xFF
    if key == ord('i') and clicked_pos:  # Press "i" to get the HSV value
        x, y = clicked_pos
        hsv_color_value = hsv[y, x]
        h, s, v = hsv_color_value
        print(f"HSV Color: H={h}, S={s}, V={v}")

        # Display the selected color in the color box
        color_box[:] = cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)
        # Show the selected color in a new window with the HSV values
        color_with_text = color_box.copy()
        text = f"H={h}, S={s}, V={v}"
        cv2.putText(color_with_text, text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Selected Color", color_with_text)

    # Exit the loop when 'q' is pressed
    if key == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
