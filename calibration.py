import cv2
import numpy as np

def transform_coordinate(pixel_x, pixel_y, offset_x=0, offset_y=0):
    """
    Transforms pixel coordinates from an image into real-world coordinates using a homography matrix.

    Parameters:
        pixel_x (float): X-coordinate in the image (pixel coordinate).
        pixel_y (float): Y-coordinate in the image (pixel coordinate).
        offset_x (float): Offset to apply to the transformed X-coordinate. Default is 0.
        offset_y (float): Offset to apply to the transformed Y-coordinate. Default is 5.

    Returns:
        tuple: Transformed (X, Y) coordinates in the real-world coordinate system, including offsets.
    """

    # Source points in the image (pixel coordinates)
    source_points = np.array([
       # [230, 106], [473, 95], [481, 342], [234, 343],
       # [270, 143], [437, 179], [441, 301], [276, 303],
       # [313, 183], [394, 179], [397, 261], [316, 263],
       [252,90], [500,78], [505,330], [256,327],
       [293, 126], [458,121], [465, 290], [298,292],
    ], dtype=np.float32)

    # Destination points in the real-world coordinate system
    destination_points = np.array([
        #[0, 0], [30, 0], [30, 30], [0, 30],
        #[5, 5], [25, 5], [25, 25], [5, 25],
        #[10, 10], [20, 10], [20, 20], [10, 20],
        [-15,0], [-15,30], [15,30], [15,0],
        [-10,5], [-10,25], [10,25], [10,5],
    ], dtype=np.float32)

    # Calculate the Homography Matrix
    homography_matrix, status = cv2.findHomography(source_points, destination_points)

    # Log the Homography Matrix
    print("Homography Matrix (H):")
    print(homography_matrix)

    # Prepare the point to transform (pixel coordinates)
    input_point = np.array([[pixel_x, pixel_y]], dtype=np.float32).reshape(-1, 1, 2)

    # Transform the pixel coordinates into real-world coordinates
    transformed_point = cv2.perspectiveTransform(input_point, homography_matrix)

    # Extract the transformed coordinates
    transformed_x, transformed_y = transformed_point[0, 0]

    # Apply offsets
    world_x = transformed_x + offset_x
    world_y = transformed_y + offset_y

    # Log the transformation results
    print("Point in Image (Pixel Coordinates):", input_point)
    print("Point in World (Real-World Coordinates):", transformed_point)

    return world_x, world_y

# Example usage
# x, y = transform_coordinate(460, 122)
# print(f"Transformed coordinates: X={x}, Y={y}")