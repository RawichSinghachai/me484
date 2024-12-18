import cv2
import numpy as np

def transform_coordinate(pixel_x, pixel_y, points,offset_x=0, offset_y=0):
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
    source_points = np.array(points,dtype=np.int32)

    # Destination points in the real-world coordinate system
    destination_points = np.array([
        [-15,0], [-15,30], [15,30], [15,0],
        # [-10,5], [-10,25], [10,25], [10,5],
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