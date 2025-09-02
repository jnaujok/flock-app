import math
from flock.vector3 import Vector3

class Position(Vector3):
    def __init__(self, x: float, y: float, z: float ) -> None:
        super().__init__(x,y,z)

    def get_distance(self, pos: "Position" ) -> float:
        """
        Calculate the Euclidean distance between two points in 3D space.
        
        Args:
            pos (Position): Second point with x, y, z coordinates
            
        Returns:
            float: The Euclidean distance between the two points
        """
        return math.sqrt(
            (pos.x - self.x) ** 2 +
            (pos.y - self.y) ** 2 +
            (pos.z - self.z) ** 2
        )
    

    def __add__(self, other: Vector3 ) -> "Position":
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)