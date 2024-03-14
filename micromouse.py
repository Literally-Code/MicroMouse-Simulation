import pygame
import math
import numpy as np

MAP_DIMENSIONS = (600, 1200)
SENSOR_ANGLES = [
    -math.pi / 2, 
    0, 
    math.pi / 2
]

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

def meters_to_pixels(meters):
    return meters * 3779.52

def pixels_to_meters(pixels):
    return pixels / 3779.52

def update(dt, robot, obstacles, gfx):
    gfx.map.blit(gfx.map_img, (0, 0))
    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    gfx.draw_sensor_data(obstacles)

class Robot:
    def __init__(self, startpos, width):
        self.w = width

        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        self.vl = 0
        self.vr = 0

        self.max_speed = 0.02*meters_to_pixels(1)
        self.min_speed = -0.02*meters_to_pixels(1)

        self.count_down = 5

    # @param motor: "LEFT", "RIGHT", or "BOTH"
    def set_motor_strength(self, motor, strength):
        if motor == "LEFT":
            self.vl = strength * meters_to_pixels(1)
        if motor == "RIGHT":
            self.vr = strength * meters_to_pixels(1)
        if motor == "BOTH":
            self.vl = strength * meters_to_pixels(1)
            self.vr = strength * meters_to_pixels(1)

    def kinematics(self, dt):
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0

        self.vr = max(min(self.max_speed, self.vr), self.min_speed)
        self.vl = max(min(self.max_speed, self.vl), self.min_speed)

class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)

        self.height, self.width = dimensions

        pygame.display.set_caption("Micromouse Simulation")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, (255, 0, 0), point, 3, 0)

class Sensor:
    def __init__(self, sensor_range, active_map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = active_map

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x + math.cos(heading) * 23, y - math.sin(heading) * 23
        for sensor_name, angle in zip(["LEFT", "FRONT", "RIGHT"], SENSOR_ANGLES):
            x2 = x1 + self.sensor_range * math.cos(heading + angle)
            y2 = y1 - self.sensor_range * math.sin(heading + angle)
            for i in range(0, 500):
                u = i / 500
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    # This makes the laser visible, but only works if the function is called after the map is drawn
                    # self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles

## Main Loop

start = (125, 125) # Starting (x, y) position of robot
sensor_range = 300 # Distance 
gfx = Graphics(MAP_DIMENSIONS, 'assets\Robot.png', 'assets\Map.png')
robot = Robot(start, 0.015 * meters_to_pixels(1))
laser_sensor = Sensor(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()
running = True

while running:
    for event in pygame.event.get():
        running = not (event.type == pygame.QUIT)
        keys = pygame.key.get_pressed()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                robot.set_motor_strength("LEFT", 0.1)
            if event.key == pygame.K_d:
                robot.set_motor_strength("RIGHT", 0.1)
            if event.key == pygame.K_j:
                robot.set_motor_strength("LEFT", -0.1)
            if event.key == pygame.K_l:
                robot.set_motor_strength("RIGHT", -0.1)
 
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a:
                robot.set_motor_strength("LEFT", 0)
            if event.key == pygame.K_d:
                robot.set_motor_strength("RIGHT", 0)
            if event.key == pygame.K_j:
                robot.set_motor_strength("LEFT", 0)
            if event.key == pygame.K_l:
                robot.set_motor_strength("RIGHT", 0)

    obstacles = laser_sensor.sense_obstacles(robot.x, robot.y, robot.heading)

    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()    
    update(dt, robot, obstacles, gfx)
    pygame.display.flip()