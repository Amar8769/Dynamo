import math
import numpy as np

# constant for now allow change later
gravity = -9.81

particles = []
physics_objects = []
environment = []


class particle:  # add delta dime in the future
    def __init__(self, mass, x, y, vx, vy, radius):
        self.mass = mass
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius

        # used for position correction
        self.inverse_mass = 1 / mass

    def move(self):
        self.x += self.vx
        self.y += self.vy


class environment_segment:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

        # infinite mass since environment is immovable
        self.mass = float('inf')
        self.inverse_mass = 0


class circle:
    def __init__(self, mass, x, y, vx, vy, radius, dt):
        self.mass = mass
        self.x = x
        self.y = y
        # todo fix the gravity logic
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.dt = dt  # delta time (seconds per frame)

        # used for position correction
        self.inverse_mass = 1 / mass

    def move(self):
        self.vy += self.dt * gravity
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
            


def collision(particle1, particle2):
    p1 = particle1
    p2 = particle2

    distance_between_particles = math.sqrt(
        math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
    )
    total_radius = p1.radius + p2.radius

    if distance_between_particles <= total_radius:
        # mass
        m1 = p1.mass
        m2 = p2.mass

        # v = (vx,vy) original vector
        v1 = np.array([p1.vx, p1.vy])
        v2 = np.array([p2.vx, p2.vy])

        # normal vector
        nx = (p2.x - p1.x) / math.sqrt(
            math.pow((p2.x - p1.x), 2) + math.pow((p2.y - p1.y), 2)
        )
        ny = (p2.y - p1.y) / math.sqrt(
            math.pow((p2.x - p1.x), 2) + math.pow((p2.y - p1.y), 2)
        )

        # tangent vector
        tx = -ny
        ty = nx

        n = np.array([nx, ny])
        t = np.array([tx, ty])

        # Correcting positions in case of overlap
        if distance_between_particles < total_radius:
            penetration = total_radius - distance_between_particles
            total_inverse_mass = p1.inverse_mass + p2.inverse_mass

            p1_im = p1.inverse_mass / total_inverse_mass
            p2_im = p2.inverse_mass / total_inverse_mass

            p1_dist = p1_im * penetration
            p2_dist = p2_im * penetration

            p1.x = p1.x + (p1_dist * nx)
            p1.y = p1.y + (p1_dist * ny)

            p2.x = p2.x + (p2_dist * -nx)
            p2.y = p2.y + (p2_dist * -ny)

        # Continuing Collision
        v1n = np.array(np.dot(v1, n))
        v1t = np.array(np.dot(v1, t))

        v2n = np.array(np.dot(v2, n))
        v2t = np.array(np.dot(v2, t))

        # 1d collision
        v1n_collided = (((m1 - m2) * v1n) + (2 * m2 * v2n)) / (m1 + m2)
        v2n_collided = (((m1 - m2) * v2n) + (2 * m1 * v1n)) / (m1 + m2)

        v1_new = (v1n_collided * n) + (v1t * t)
        v2_new = (v2n_collided * n) + (v2t * t)

        particle1.vx = v1_new[0]
        particle1.vy = v1_new[1]
        particle2.vx = v2_new[0]
        particle2.vy = v2_new[1]


def particleWorldBorderCollision(particle, width, height):
    p = particle
    if p.x <= 0:
        p.x = 0
        p.vx *= -1
    if p.x >= width:
        p.x = width
        p.vx *= -1
    if p.y <= 0:
        p.y = 0
        p.vy *= -1
    if p.y >= height:
        p.y = height
        p.vy *= -1


def worldBorderCollision(physics_object, width, height):
    obj = physics_object
    left = obj.x - obj.radius
    right = obj.x + obj.radius
    top = obj.y + obj.radius
    bottom = obj.y - obj.radius

    if left <= 0:
        obj.x = obj.radius
        obj.vx = 0
    if right >= width:
        obj.x = width - obj.radius
        obj.vx = 0
    if bottom <= 0:
        obj.y = obj.radius
        obj.vy = 0
    if top >= height:
        obj.y = height - obj.radius
        obj.vy = 0      

def step(width, height):
    # always follow move->constraint->interaction

    num_of_particles = len(particles)
    num_of_physics_objects = len(physics_objects)

    # Move all particles
    for i in range(num_of_particles):
        particles[i].move()

    for p in physics_objects:
        p.move()

    # Check for world border for all particles
    for i in range(num_of_particles):
        particleWorldBorderCollision(particles[i], width, height)

    for p in physics_objects:
        worldBorderCollision(p, width, height)

    # Check for collisions between any particles
    for i in range(num_of_particles):
        for j in range(i + 1, num_of_particles):
            collision(particles[i], particles[j])

    """for i in range(num_of_physics_objects):
        for j in range(i + 1, num_of_physics_objects):
            collision(physics_objects[i], physics_objects[j])"""
