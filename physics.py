import math
import numpy as np

# constant for now allow change later
gravity = -9.81

physics_objects = []
environment = []
polygon_points = []

def dot(arr1, arr2):
    if len(arr1) != len(arr2):
        raise ValueError("Cannot apply dot prouct on arrays of different sizes")
    dot_product = 0
    for i,j in zip(arr1, arr2):
        dot_product += i*j
    return dot_product

def vector_length(x, y):
    return math.sqrt(x*x + y*y)

def normalize(x, y):
    distance = vector_length(x,y)

    if distance == 0:
        return [0, 0]
    
    return [x / distance, y / distance]

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
    def __init__(self, mass, x, y, vx, vy, radius):
        self.mass = mass
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius

        # used for position correction
        self.inverse_mass = 1 / mass
    
    def apply_gravity(self, dt):
        # dt is delta time (seconds per frame)
        self.vy += dt * gravity

    def move(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

class polygon:
    def __init__(self, points, mass, vx, vy):
        self.mass = mass
        self.points = points
        self.vx = vx
        self.vy = vy
        center = calculate_centroid(points)
        self.centerx = center[0]
        self.centery = center[1]

        # used for position correction
        self.inverse_mass = 1 / mass
        #self.col = (255,0,0)
        #self.rotation_angle = 0

    def apply_gravity(self, dt):
        # dt is delta time (seconds per frame)
        self.vy += dt * gravity

    def move(self, dt):
        for point in self.points:
            point[0] += self.vx * dt
            point[1] += self.vy * dt
        self.centerx += self.vx * dt
        self.centery += self.vy * dt

def calculate_centroid(points):
    '''points = vertices
    if points[0] != points[-1]:
        points.append(points[0])'''
    twice_area = 0
    x = 0
    y = 0

    npts = len(points)

    i = 0
    j = npts - 1
    for i in range(npts):
        p1 = points[i]
        p2 = points[j]
        
        f = p1[0]*p2[1] - p2[0]*p1[1]

        twice_area += f

        x += ( p1[0] + p2[0] ) * f
        y += ( p1[1] + p2[1] ) * f

        j = i

    f = twice_area * 3
    x = x/f
    y = y/f
    c = [x,y]
    return c

def circle_circle_collision(rigidBody1, rigidBody2):

    rb1 = rigidBody1
    rb2 = rigidBody2

    # d = (dx,dy) distance on x and y axis
    dx = rb1.x - rb2.x
    dy = rb1.y - rb2.y

    distance = vector_length(dx, dy)
    if distance == 0:
        return

    total_radius = rb1.radius + rb2.radius

    if distance <= total_radius:
        # mass
        m1 = rb1.mass
        m2 = rb2.mass

        # v = (vx,vy) original vector
        v1 = np.array([rb1.vx, rb1.vy])
        v2 = np.array([rb2.vx, rb2.vy])

        #normalized vector
        n = normalize(dx,dy)

        # tangent vector
        t = [-n[1], n[0]]

        # Continuing Collision
        v1n = dot(v1, n)
        v1t = dot(v1, t)

        v2n = dot(v2, n)
        v2t = dot(v2, t)

        # 1d collision
        v1n_collided = (((m1 - m2) * v1n) + (2 * m2 * v2n)) / (m1 + m2)
        v2n_collided = (((m1 - m2) * v2n) + (2 * m1 * v1n)) / (m1 + m2)

        v1_new = (v1n_collided * n) + (v1t * t)
        v2_new = (v2n_collided * n) + (v2t * t)

        rigidBody1.vx, rigidBody1.vy = v1_new
        rigidBody2.vx, rigidBody2.vy = v2_new

def polygon_polygon_collision(rigidBody1, rigidBody2):
    rb1 = rigidBody1
    rb2 = rigidBody2

    axes = []

    j1 = len(rb1.points) - 1
    j2 = len(rb2.points) - 1

    for i in range(len(rb1.points)):
        edgex = rb1.points[i][0] - rb1[j1][0]
        edgey = rb1.points[i][1] - rb1[j1][1]

        dist = vector_length(edgex, edgey)

        #in case of identical points
        if dist == 0:
            continue

        n1 = normalize(-edgey, edgex)

        axes.append(n1)

        j1 = i

    for i in range(len(rb2.points)):
        edgex = rb2.points[i][0] - rb2[j2][0]
        edgey = rb2.points[i][1] - rb2[j2][1]

        dist = vector_length(edgex, edgey)

        #in case of identical points
        if dist == 0:
            continue

        n2 = normalize(-edgey, edgex)

        axes.append(n2)

        j2 = i
    
    flag, mtv, axis = sat(axes, rb1, rb2)

    if flag:
        polygon_position_correction(rigidBody1, rigidBody2, axis, mtv)
        impulse_resolution(rigidBody1, rigidBody2, axis)

def impulse_resolution(rigidBody1, rigidBody2, axis):
    #relative velocities
    relative_velocity = [rigidBody2.vx - rigidBody1.vx,
                            rigidBody2.vy - rigidBody1.vy]
        
    #velocity along normal
    normal_velocity = dot(relative_velocity, axis)

    if normal_velocity > 0:
        return
        
    total_inv_mass = rigidBody1.inverse_mass + rigidBody2.inverse_mass
    if total_inv_mass == 0:
        return
        
    #bounce/restitution
    e = 0.8

    #impulse magnitude
    j = -(1 + e) * normal_velocity
    j /= total_inv_mass

    impulse = [j * axis[0],
               j * axis[1]]
        
    rigidBody1.vx -= impulse[0] * rigidBody1.inverse_mass
    rigidBody1.vy -= impulse[1] * rigidBody1.inverse_mass

    rigidBody2.vx += impulse[0] * rigidBody2.inverse_mass
    rigidBody2.vy += impulse[1] * rigidBody2.inverse_mass

def sat(axes, rb1, rb2):

    mtv = float('inf')
    mtv_axis = None

    for axis in axes:

        projection1_min, projection1_max = project_polygon(axis, rb1.points)
        projection2_min, projection2_max = project_polygon(axis, rb2.points)

        if projection1_min > projection2_max or projection2_min > projection1_max:
            return False, None, None
        
        overlap = min(projection1_max,projection2_max) - max(projection1_min, projection2_min)
        if overlap < mtv:
            mtv = overlap
            mtv_axis = axis

    dx = rb2.centerx - rb1.centerx
    dy = rb2.centery - rb1.centery

    d = [dx,dy]

    if dot(d , mtv_axis) < 0:
        mtv_axis[0] = -mtv_axis[0]
        mtv_axis[1] = -mtv_axis[1]

    return True, mtv, mtv_axis

def polygon_position_correction(rigidBody1, rigidBody2, axis, mtv, percentage = 0.8, slop = 0.5):

    total_inverse_mass = rigidBody1.inverse_mass +rigidBody2.inverse_mass
    
    nx = axis[0]
    ny = axis[1]

    correction = max(mtv - slop, 0)/ total_inverse_mass

    cx = correction * nx * percentage
    cy = correction * ny * percentage

    for point in rigidBody1.points:
        point[0] -= cx * rigidBody1.inverse_mass
        point[1] -= cy * rigidBody1.inverse_mass

    rigidBody1.centerx -= cx * rigidBody1.inverse_mass
    rigidBody1.centery -= cy * rigidBody1.inverse_mass
    
    for point in rigidBody2.points:
        point[0] += cx * rigidBody2.inverse_mass
        point[1] += cy * rigidBody2.inverse_mass

    rigidBody2.centerx += cx * rigidBody2.inverse_mass
    rigidBody2.centery += cy * rigidBody2.inverse_mass

def project_polygon(axis, points):
    axis = np.array(axis)
    vertices = np.array(points)

    projection = np.dot(vertices, axis)

    return projection.min(), projection.max()


def position_correction(rigidBody1, rigidBody2, percentage = 0.8, slop = 0.5):

    dx = rigidBody2.x - rigidBody1.x
    dy = rigidBody2.y - rigidBody1.y

    distance = vector_length(dx, dy)
    radius = rigidBody1.radius + rigidBody2.radius
    total_inverse_mass = rigidBody1.inverse_mass +rigidBody2.inverse_mass
    
    penetration = radius - distance

    if distance == 0:
        return

    if penetration <= 0:
        return
    
    n = normalize(dx, dy)

    correction = max(penetration - slop, 0)/ total_inverse_mass

    c = [correction * n[0] * percentage, 
         correction * n[1] * percentage]

    rigidBody1.x -= c[0] * rigidBody1.inverse_mass
    rigidBody1.y -= c[1] * rigidBody1.inverse_mass
    rigidBody2.x += c[0] * rigidBody2.inverse_mass
    rigidBody2.y += c[1] * rigidBody2.inverse_mass

def worldBorderCollision(rigidBody, width, height):
    rb = rigidBody
    left = rb.x - rb.radius
    right = rb.x + rb.radius
    top = rb.y + rb.radius
    bottom = rb.y - rb.radius

    if left <= 0:
        rb.x = rb.radius
        rb.vx = 0
    if right >= width:
        rb.x = width - rb.radius
        rb.vx = 0
    if bottom <= 0:
        rb.y = rb.radius
        rb.vy = 0
    if top >= height:
        rb.y = height - rb.radius
        rb.vy = 0      

def step(dt,width, height):
    # physics pipeline
    # 1 integrate forces (gravity)
    # 2 integrate motion
    # 3 solve collisions
    #    - detect
    #    - correct penetration
    #    - resolve impulse

    # always follow move->constraint->interaction

    num_of_physics_objects = len(physics_objects)

    for p in physics_objects:
        p.apply_gravity(dt)
        p.move(dt)

    for p in physics_objects:
        worldBorderCollision(p, width, height)

    # Check for collisions between any particles
    for i in range(num_of_physics_objects):
        for j in range(i + 1, num_of_physics_objects):
            if isinstance(physics_objects[i], circle) and isinstance(physics_objects[j], circle):
                circle_circle_collision(physics_objects[i], physics_objects[j])
                position_correction(physics_objects[i], physics_objects[j])
            elif isinstance(physics_objects[i], polygon) and isinstance(physics_objects[j], polygon):
                polygon_polygon_collision(physics_objects[i], physics_objects[j])