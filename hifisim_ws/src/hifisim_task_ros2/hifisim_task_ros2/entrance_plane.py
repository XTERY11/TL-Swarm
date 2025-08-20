import numpy as np

class Plane:
    def __init__(self, point, normal):
        """
        Initialize a Plane with a point on the plane and a normal vector.
        
        Parameters:
            point (list or array-like): A 3-element sequence representing a point [x, y, z].
            normal (list or array-like): A 3-element sequence representing the normal vector [nx, ny, nz].
        """
        # Convert the inputs to NumPy arrays of type float.
        self.point = np.array(point, dtype=float)
        self.normal = np.array(normal, dtype=float)
        
        normal_formatted = [f"{value:.1f}" for value in self.normal]
        # print("Class Plane:self.point is:", self.point, "self.normal is:", normal_formatted)
        # Normalize the normal vector.
        norm = np.linalg.norm(self.normal)
        if norm == 0:
            raise ValueError("The normal vector must not be the zero vector.")
        self.normal = self.normal / norm

    def perp_distance_to_pt(self, pt):
        """
        Compute the perpendicular distance from a given point to the plane.
        
        Parameters:
            pt (list or array-like): A 3-element sequence representing a point [x, y, z].
            
        Returns:
            float: The perpendicular distance from the point to the plane.
                   Positive if the point is in the positive direction of the normal vector,
                   negative otherwise.
        """
        pt = np.array(pt, dtype=float)
        # Use the formula: distance = n · (pt - point_on_plane)
        distance = np.dot(pt - self.point, self.normal)
        return distance

    def center_distance_to_pt(self, pt):
        """
        Compute the Euclidean distance between the plane's reference point and a given point.
        
        Parameters:
            pt (list or array-like): A 3-element sequence representing a point [x, y, z].
            
        Returns:
            float: The Euclidean distance between the plane's center (self.point) and pt.
        """
        pt = np.array(pt, dtype=float)
        return np.linalg.norm(pt - self.point)

    def plane_equation(self):
        """
        Return the coefficients (A, B, C, D) of the plane equation in the form:
            A*x + B*y + C*z + D = 0.
        
        Returns:
            tuple: (A, B, C, D) where (A, B, C) are the components of the normal vector and D is computed accordingly.
        """
        A, B, C = self.normal
        D = -np.dot(self.normal, self.point)
        return A, B, C, D

    @staticmethod
    def rpy_to_rotation_matrix(roll, pitch, yaw):
        """
        Convert roll, pitch, yaw (in degrees) to a rotation matrix.
        Assumes the order R = R_z(yaw) * R_y(pitch) * R_x(roll)
        
        Parameters:
            roll (float): Rotation around the x-axis in degrees.
            pitch (float): Rotation around the y-axis in degrees.
            yaw (float): Rotation around the z-axis in degrees.
        
        Returns:
            np.ndarray: The 3x3 rotation matrix.
        """
        # Convert degrees to radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)
        
        # Rotation matrix for roll (rotation about x-axis)
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll),  np.cos(roll)]])
        
        # Rotation matrix for pitch (rotation about y-axis)
        R_y = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        # Rotation matrix for yaw (rotation about z-axis)
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw),  np.cos(yaw), 0],
                        [0, 0, 1]])
        
        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))
        
        return R

    @staticmethod
    def rpy_to_normal(roll, pitch, yaw):
        """
        Convert roll, pitch, yaw angles (in degrees) to a normal vector.
        Assumes that the object's local normal is [1, 0, 0].
        """
        R = Plane.rpy_to_rotation_matrix(roll, pitch, yaw)
        local_normal = np.array([1, 0, 0])
        # Multiply the rotation matrix by the local normal vector:
        normal_vector = R @ local_normal
        return normal_vector


# Example usage:
if __name__ == '__main__':
    # Get a normal vector by rotating RPY from positive X direction 
    roll = 0.0    # roll angle in degrees
    pitch = 0.0  # pitch angle in degrees
    yaw = 0.0    # yaw angle in degrees
    normal = Plane.rpy_to_normal(roll, pitch, yaw)
    # Using list comprehension
    normal_formatted = [f"{value:.1f}" for value in normal]
    print(f"Main:The normal vector rotated {roll, pitch, yaw} from positive X direction is: {normal_formatted}")

    # Rotate the normal vector of above by the RPY angles to get another vector.  
    roll = 0.0    # roll angle in degrees
    pitch = 0.0  # pitch angle in degrees
    yaw = 90.0    # yaw angle in degrees
    print("Main:RPY angles in degrees:", roll, pitch, yaw)
    R = Plane.rpy_to_rotation_matrix(roll, pitch, yaw)
    left_vector = R @ normal
    left_vector_formatted = [f"{value:.1f}" for value in left_vector]
    print(f"Main:The left pointing vector is: {left_vector_formatted}")

###########
    # Define a plane with a point on it and a normal vector.
    # For example, the plane through (1, 2, 3) with a normal [0, 0, 1] (which corresponds to z = 3)
    plane = Plane(point=[1, 2, 3], normal=normal)
    print("Main:Point on the plane:", plane.point)
    normal_formatted = [f"{value:.1f}" for value in plane.normal]
    print("Main:Normal vector to the plane:", normal_formatted)
          
    # Get the plane equation coefficients (A, B, C, D)
    coeffs = plane.plane_equation()
    coeffs_formatted = [f"{value:.1f}" for value in coeffs]
    print("Main:Plane equation coefficients (A, B, C, D):", coeffs_formatted)
    
    # Calculate the distance from a sample point to the plane.
    sample_point = [10, 3, 15]
    distance = plane.perp_distance_to_pt(sample_point)
    print(f"Main:Distance from {sample_point} to the plane: {distance:.2f}")
