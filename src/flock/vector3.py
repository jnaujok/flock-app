import math

class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def get_x(self) -> float:
        return self.x
    def get_y(self) -> float:
        return self.y
    def get_z(self) -> float:
        return self.z

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar):
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def magnitude(self):
        return math.sqrt(self.magnitude_squared())

    def magnitude_squared(self):
        return (self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        l = self.magnitude()
        if l > 0:
            return self / l
        return Vector3()

    def to_tuple(self):
        return (self.x, self.y, self.z)
    
    def distance_to(self, other):
        return math.sqrt( (self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2 )
    
    def rotate_random( self, angle_x: float, angle_y: float, angle_z: float ) -> "Vector3":
        # Rotation around X-axis
        cos_x, sin_x = math.cos(angle_x), math.sin(angle_x)
        new_y = self.y * cos_x - self.z * sin_x
        new_z = self.y * sin_x + self.z * cos_x
        self.y, self.z = new_y, new_z
        
        # Rotation around Y-axis
        cos_y, sin_y = math.cos(angle_y), math.sin(angle_y)
        new_x = self.x * cos_y + self.z * sin_y
        new_z = -self.x * sin_y + self.z * cos_y
        self.x, self.z = new_x, new_z
        
        # Rotation around Z-axis
        cos_z, sin_z = math.cos(angle_z), math.sin(angle_z)
        new_x = self.x * cos_z - self.y * sin_z
        new_y = self.x * sin_z + self.y * cos_z
        self.x, self.y = new_x, new_y
        
        return self
    