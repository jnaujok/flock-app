from direct.showbase.ShowBase import ShowBase
from direct.showbase.Loader import Loader
from panda3d.core import TextureStage, TexGenAttrib
from direct.task import Task

from flock.flock import Flock

import sys
from math import sin, cos, pi

cam_speed = 0.2
NUM_BOIDS = 100


class FlockApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        skybox = self.loader.loadModel("../models/skybox_1024")
        skybox.setScale(10)
        skybox.reparentTo(self.camera)
        skybox.set_two_sided(True)
        skybox.set_bin("background", 0)
        skybox.set_depth_write(False)
        skybox.set_compass()
        # Bookkeeping for the rotation around the model
        self.angle = 0.0
        self.pitch = 0.0
        self.adjust_angle = 0
        self.adjust_pitch = 0
        self.last_time = 0.0
        # Initial camera setup
        self.camera.set_pos(sin(self.angle)*20,-cos(self.angle)*20,0)
        self.camera.look_at(0,0,0)
        # Key events and camera movement task
        self.accept("arrow_left", self.adjust_turning, [-1.0, 0.0])
        self.accept("arrow_left-up", self.adjust_turning, [1.0, 0.0])
        self.accept("arrow_right", self.adjust_turning, [1.0, 0.0])
        self.accept("arrow_right-up", self.adjust_turning, [-1.0, 0.0])
        self.accept("arrow_up", self.adjust_turning, [0.0, 1.0])
        self.accept("arrow_up-up", self.adjust_turning, [0.0, -1.0])
        self.accept("arrow_down", self.adjust_turning, [0.0, -1.0])
        self.accept("arrow_down-up", self.adjust_turning, [0.0, 1.0])
        self.accept("escape", sys.exit)
        self.taskMgr.add(self.update_camera, 'adjust camera', sort = 10)

        spaceship = self.loader.loadModel("../models/spaceship.egg.pz")

        self.flock = Flock("TestFlock", self.render, spaceship, NUM_BOIDS, self.task_mgr )

    def adjust_turning(self, heading, pitch):
        self.adjust_angle += heading
        self.adjust_pitch += pitch

    def update_camera(self, task):
        if task.time != 0.0:
            dt = task.time - self.last_time
            self.last_time = task.time
            self.angle += pi * dt * self.adjust_angle * cam_speed
            self.pitch += pi * dt * self.adjust_pitch * cam_speed
            # Why /2.001 and not an even 2.0? Because then we'd have to set_Hpr
            # explicitly, as look_at can't deduce the heading when the camera is
            # exactly above/below the spheres center.
            if self.pitch > pi/2.001:
                self.pitch = pi/2.001
            if self.pitch < -pi/2.001:
                self.pitch = -pi/2.001
            self.camera.set_pos(sin(self.angle)*cos(abs(self.pitch))*20,
                                -cos(self.angle)*cos(abs(self.pitch))*20,
                                sin(self.pitch)*20)
            self.camera.look_at(0,0,0)
        return Task.cont


if __name__ == "__main__":
    app = FlockApp()
    app.run()
