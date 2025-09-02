from direct.showbase.Loader import Loader
from direct.actor.Actor import Actor
from typing import Any, TYPE_CHECKING

from flock.position import Position
from flock.vector3 import Vector3


from random import random, uniform
import math

# Parameters
BOID_SIZE = 8

MIN_SPEED = 0.5
MAX_SPEED = 25

MAX_FORCE = 25

BOID_FRICTION = 0.75

WANDER_FACTOR = 20

SEPARATION = 15
SEPARATION_RADIUS = 20

ALIGNMENT = 5
ALIGNMENT_RADIUS = 20

COHESION = 2.5
COHESION_RADIUS = 30

PURSUIT_RADIUS = 15

if TYPE_CHECKING:
    from flock.flock import Flock

class Bird:
    def __init__(self, flock: "Flock", render: Any, name: str, actor: Actor ) -> None:
        self.name = name
        self.flock = flock
        self.actor: Actor = actor
        self.placeholder = render.attachNewNode(name)
        self.position: Position = Position( random() * 100.0, random() * 100.0, random() * 100.0 )
        self.placeholder.setPos(self.position.x, self.position.y, self.position.z)
        self.placeholder.setScale(0.001)
        self.actor.instanceTo(self.placeholder)

        self.speed = MIN_SPEED  # Max speed
        self.velocity = Vector3(uniform(-2, 2), uniform(-2, 2), uniform(-2,2)).normalize() * self.speed  # Random initial velocity
        self.hpr=Vector3(0,0,0)  # Initial orientation

        self.pursuit_radius = PURSUIT_RADIUS  # Radius for pursuing a target

        self.max_force = MAX_FORCE  # force cap, limits the size of the different forces
        self.friction = BOID_FRICTION  # Friction coefficient for the simplistic physics

        # Parameters for wandering behaviour
        self.target = Position(0,0,0)
        self.future_loc = Vector3(0, 0, 0)

        # Random initial angles for wandering
        self.theta = uniform(-math.pi, math.pi)
        self.omega = uniform(-math.pi, math.pi)
        self.omicron = uniform(-math.pi, math.pi)

    def get_x(self) -> float:
        return self.position.x
    
    def get_y(self) -> float:
        return self.position.y
    
    def get_z(self) -> float:
        return self.position.z
    
    def set_position(self, position: Position) -> None:
        self.position = position
        self.placeholder.setPos(self.position.x, self.position.y, self.position.z)

    def set_hpr(self, h: float, p: float, r: float) -> None:
        self.placeholder.setHpr(h, p, r)

    def get_hpr(self) -> tuple[float, float, float]:
        return self.placeholder.getHpr()
    
    def get_position(self) -> Position:
        return self.position
    
    def __str__(self) -> str:
        return f"Bird {self.name} at position ({self.position.x}, {self.position.y}, {self.position.z})"
    
    def __repr__(self) -> str:
        return self.__str__()
    
    def get_name(self) -> str:
        return self.name    
    
    def update(self, dt: float) -> None:
        """
        Updates the acceleration of the boid by adding together the different forces that acts on it
        """
        new_velocity = Vector3(0,0,0)

        # Add together all the forces
        new_velocity += self.wander()  # Wandering force
        new_velocity += self.separation() * SEPARATION  # separation force scaled with a control parameter
        new_velocity += self.alignment() * ALIGNMENT  # alignment force scaled with a control parameter
        new_velocity += self.cohesion() * COHESION  # cohesion force scaled with a control parameter

        # multiply the new velocity with the time delta to get the change in velocity
        new_velocity *= dt
        # Apply friction to the new velocity
        new_velocity *= self.friction
        self.velocity += new_velocity
        self.position += self.velocity
        self.placeholder.setPos(self.position.x, self.position.y, self.position.z)
        roll, pitch, yaw = self.get_euler_angles()
        self.placeholder.setHpr(math.degrees(yaw), math.degrees(pitch)+180, math.degrees(roll))

    def separation(self):
        """
        Calculate the separation force vector
        Separation: steer to avoid crowding local flockmates
        :return force vector
        """
        force_vector = Vector3(0, 0, 0)
        boids_in_view = self.boids_in_radius(SEPARATION_RADIUS)
    
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # TODO: Implement this
        for other_boid in boids_in_view:
            distance = self.position.distance_to(other_boid.position)

            # Make sure the distance is not 0 to avoid division by 0
            if distance == 0:
                continue

            # Calculate the force vector, the closer the boid is the larger the force
            x_diff = self.position.x - other_boid.position.x
            y_diff = self.position.y - other_boid.position.y
            z_diff = self.position.z - other_boid.position.z
            force_vector += Vector3(x_diff, y_diff,z_diff) * (SEPARATION_RADIUS / distance)
        
        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    def alignment(self):
        """
        Calculate the alignment force vector
        Alignment: steer towards the average heading of local flockmates
        :return force vector
        """
        force_vector = Vector3(0, 0, 0)
        boids_in_view = self.boids_in_radius(ALIGNMENT_RADIUS)
        
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # Find the direction of the flock by adding together the velocity vectors of the boids in view
        for other_boid in boids_in_view:
            force_vector += other_boid.velocity
            
        if force_vector.magnitude() == 0:
            return force_vector
        
        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    def cohesion(self):
        """
        Calculate the cohesion force vector
        Cohesion: steer to move toward the average position of local flockmates
        """
        force_vector = Vector3(0, 0, 0)
        boids_in_view = self.boids_in_radius(COHESION_RADIUS)
        
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # Calculate the average position of the boids in view
        for other_boid in boids_in_view:
            # Make the boids move towards the average position of the boids in view
            dx = other_boid.position.x - self.position.x
            dy = other_boid.position.y - self.position.y
            dz = other_boid.position.z - self.position.z
            force_vector += Vector3(dx, dy, dz)

        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    
    def boids_in_radius(self, radius: float) -> list["Bird"]:
        """
        Find all boids in a given radius
        """
        boids: list = []
        for other_boid in self.flock.birds:
            if other_boid == self:
                continue

            if self.position.get_distance(other_boid.position) < radius:
                boids.append(other_boid)
        return boids

    def cap_force(self, force_vector: Vector3, boids_in_view: list["Bird"]) -> Vector3:
        """
        Takes a list of boids in view and returns a force vector that is capped by the max force
        """
        force_vector /= len(boids_in_view)
        # Make sure the force vector is not 0
        if force_vector.magnitude() <= 0:
            return force_vector
        
        force_vector = force_vector.normalize() * self.speed - self.velocity

        if force_vector.magnitude() > self.max_force:
            force_vector.normalize() * self.max_force
        return force_vector

    def move_towards_target(self, target: Position ) -> Vector3:
        """
        Calculate force vector for moving the boid to the target
        """
        # vector to the target
        desired = target - self.position

        distance = desired.magnitude()
        desired = desired.normalize()

        if distance < self.pursuit_radius:
            # if the distance is less than the radius, remap it to a value between 0 and speed
            m = self.remap(distance, 0, self.pursuit_radius, 0, self.speed)

            # scale the desired vector up to continue movement in that direction
            desired *= m
        else:
            # scale the desired vector to the max speed
            desired *= self.speed

        force_vector = desired - self.velocity
        self.cap_magnitude(force_vector, self.max_force)
        return force_vector

    def cap_magnitude(self, vector: Vector3, max_magnitude: float) -> None:
        """
        Cap the magnitude of a vector to a maximum value
        """
        mag = vector.magnitude()
        if mag > max_magnitude:
            vector = vector.normalize() * max_magnitude
        return vector   
    
    def wander(self):
        """
        Calcualte a random target to move towards to get natural random flight
        """
        if self.velocity.magnitude_squared() != 0:
            # Calculate where you will be in the future
            self.future_loc = self.velocity.normalize() * 80

            # Calculate a random angle addition
            self.theta += uniform(-math.pi, math.pi) / 30
            self.omega += uniform(-math.pi, math.pi) / 30
            self.omicron += uniform(-math.pi, math.pi) / 30

            # Create a random vector on a sphere with radius WANDER_RADIUS
            wander_vector = Vector3(WANDER_FACTOR, WANDER_FACTOR, WANDER_FACTOR)
            wander_vector.rotate_random( self.theta, self.omega, self.omicron)

            # set the target to your position + your future position + a distance in the direction of the random angle
            self.target = self.position + self.future_loc + wander_vector

        return self.move_towards_target(self.target)

    def remap(self, value, from1, to1, from2, to2):
        """
        Remap a value from one range to another
        """
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2  

    def get_euler_angles(self):
        # Normalize the vector to ensure unit length
        magnitude = self.velocity.magnitude()
        if magnitude == 0:
            return 0.0, 0.0, 0.0  # Return zeros if vector is zero-length
        nx, ny, nz = self.velocity.x / magnitude, self.velocity.y / magnitude, self.velocity.z / magnitude
        
        # Calculate pitch (rotation around X-axis, angle from XY-plane)
        pitch = math.asin(-nz)  # Z points down in standard convention
        
        # Calculate yaw (rotation around Y-axis, angle in XY-plane)
        yaw = math.atan2(nx, ny)
        
        # Calculate roll (rotation around Z-axis)
        # Assume default up vector (0, 1, 0) for reference
        # Project the up vector onto the plane perpendicular to the direction
        up = Vector3(0, 1, 0)
        # Right vector: cross product of direction and up
        right = Vector3(
            ny * up.z - nz * up.y,
            nz * up.x - nx * up.z,
            nx * up.y - ny * up.x
        )
        right_magnitude = math.sqrt(right.x**2 + right.y**2 + right.z**2)
        if right_magnitude == 0:
            roll = 0.0  # Undefined roll if direction aligns with up vector
        else:
            # Recompute up vector from cross product of direction and right
            up = Vector3(
                ny * right.z - nz * right.y,
                nz * right.x - nx * right.z,
                nx * right.y - ny * right.x
            )
            # Roll is the angle of the right vector in the plane
            roll = math.atan2(right.z, right.x)
        
        return roll, pitch, yaw