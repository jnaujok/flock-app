from direct.showbase.Loader import Loader
from direct.actor.Actor import Actor
from direct.task.Task import Task, TaskManager
from panda3d.core import PythonTask
from typing import Any

from flock.bird import Bird
from flock.position import Position

def update(flock, task ) -> PythonTask:
    for bird in flock.birds:
        # Calculate new position and orientation here
        bird.update(task.dt)
    return task.cont



class Flock:
    def __init__(self, name: str, render: Any, bird_model: list | Any | Loader._Callback, count: int, task_mgr: TaskManager ) -> None:
        self.name = name
        self.bird_model = bird_model
        self.bird_actor = Actor(self.bird_model)
        self.birds: list[Bird] = []
        self.task_mgr = task_mgr

        # Create birds
        for i in range(count):
            new_bird = Bird( self, render, f"{name}_Bird_{i}", self.bird_actor )
            self.birds.append(new_bird)
        
        # Add the update task to the task manager       
        self.task_mgr.add( update, f'update_flock_{self.name}', sort = 20, extraArgs=[self], appendTask=True )

        
    def center_of_mass(self) -> Position:
        """
        Calculate the center of mass for a list of points in 3D space, assuming equal mass for each point.
        
        Args:
            points (list[Position]): List of Position objects with x, y, z coordinates
            
        Returns:
            Position: The center of mass as a Position object
            
        Raises:
            ValueError: If the input list is empty
        """
        if not self.birds:
            raise ValueError("List of 'birds' cannot be empty")
            
        n = len(self.birds)
        sum_x: float = 0.0
        sum_y: float = 0.0
        sum_z: float = 0.0
        for bird in self.birds:
            sum_x += bird.get_x()
            sum_y += bird.get_y()
            sum_z += bird.get_z()

        return Position(sum_x / n, sum_y / n, sum_z / n)
    
