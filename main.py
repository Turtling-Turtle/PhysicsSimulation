# @author Nathan Ulmen
import math
import time
import pygame


class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vec2({self.x}, {self.y})"

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self

    def __mul__(self, other):
        if isinstance(other, Vec2):
            return Vec2(self.x * other.x, self.y * other.y)
        if isinstance(other, float):
            return Vec2(self.x * other, self.y * other)

    def multiply(self, scalar: float):
        return Vec2(self.x * scalar, self.y * scalar)

    def divide(self, scalar: float):
        return Vec2(self.x / scalar, self.y / scalar)


def compute_distance(left: Vec2, right: Vec2) -> float:
    return math.sqrt((left.x - right.x) ** 2 + (left.y - right.y) ** 2)


def compute_magnitude(vec: Vec2) -> float:
    return math.sqrt(vec.x ** 2 + vec.y ** 2)


def normalize(vec: Vec2) -> Vec2:
    magnitude = compute_magnitude(vec)
    return Vec2(0, 0) if magnitude == 0 else Vec2(vec.x / magnitude, vec.y / magnitude)


class PhysicsObject:
    def __init__(self):
        self.position = Vec2(0, 0)
        self.velocity = Vec2(0, 0)
        self.acceleration = Vec2(0, 0)
        self.mass = 2
        self.radius = 10
        self.rotation = 0
        self.isStatic = False

    def __repr__(self):
        return f"PhysicsObject({self.position}, {self.velocity}, {self.acceleration}) {self.mass}"

    def update(self, delta_time) -> None:
        if self.isStatic:
            return

        # Apply Drag
        # F = drag_coefficient * velocity_mag^2 * unit_velocity
        acceleration = Vec2(0, 0)
        if self.velocity.x != 1 or self.velocity.y != 0:
            drag = 0.47
            velocity_mag = compute_magnitude(self.velocity)
            unit_velocity = normalize(self.velocity)
            drag_force = unit_velocity * (-drag * (velocity_mag ** 2))
            acceleration += drag_force * (1 / self.mass)

        # v = v + a * deltaT
        self.velocity += (self.acceleration + acceleration) * delta_time

        # p = v * deltaT
        self.position += self.velocity * delta_time

    def apply_force(self, force: Vec2) -> None:
        # a = f / m
        self.acceleration += force * (1 / self.mass)


def neg(vec: Vec2) -> Vec2:
    return Vec2(-vec.x, -vec.y)


def solve_collision(obj1: PhysicsObject, obj2: PhysicsObject):
    if obj1.isStatic and obj2.isStatic:
        return

    normal = normalize(obj1.position - obj2.position)

    rad = obj1.radius + obj2.radius
    depth = rad - compute_distance(obj1.position, obj2.position)

    if not obj1.isStatic:
        obj1.position += normal * (depth * (obj2.mass / (obj1.mass + obj2.mass)))

    if not obj2.isStatic:
        obj2.position += neg(normal) * (depth * (obj1.mass / (obj1.mass + obj2.mass)))


def intersects(obj1: PhysicsObject, obj2: PhysicsObject) -> bool:
    distance = compute_distance(obj1.position, obj2.position)
    # Return if they touch or intersect
    return distance <= obj1.radius + obj2.radius or distance == obj1.radius + obj2.radius


OBJECT_COUNT = 12


def main():
    pygame.init()
    screen_width, screen_height = 1600, 1200
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("PhysicsSimulation")

    # Initialize Physics Objects
    gravity = Vec2(0, -9.81)
    physics_objects = []
    for i in range(OBJECT_COUNT):
        for j in range(OBJECT_COUNT):
            physics_object = PhysicsObject()
            physics_object.position = Vec2((j + 10) * 20, (i + 10) * 25)
            physics_object.apply_force(gravity)
            physics_object.isStatic = i == 0
            physics_objects.append(physics_object)

    # Main loop
    last_time = time.perf_counter()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # apply forces based on input
        keys = pygame.key.get_pressed()
        forces = []
        if keys[pygame.K_w]:
            forces.append(Vec2(0, 15))

        if keys[pygame.K_s]:
            forces.append(Vec2(0, -15))

        if keys[pygame.K_a]:
            forces.append(Vec2(-15, 0))

        if keys[pygame.K_d]:
            forces.append(Vec2(15, 0))

        for obj in physics_objects:
            if not obj.isStatic:
                for force in forces:
                    obj.apply_force(force)

        # Calculate Delta
        current_time = time.perf_counter()
        delta_time = current_time - last_time
        last_time = current_time

        # Solve Collisions
        for i in range(len(physics_objects)):
            object1 = physics_objects[i]
            for j in range(i + 1, len(physics_objects)):
                object2 = physics_objects[j]
                if intersects(object1, object2):
                    solve_collision(object1, object2)

        # Update Positions
        for obj in physics_objects:
            obj.apply_force(gravity)
            obj.update(delta_time)

        screen.fill((0, 0, 0))

        for obj in physics_objects:
            screen_pos = (int(obj.position.x), int(screen_height - obj.position.y))  # Invert y-axis
            if obj.isStatic:
                pygame.draw.circle(screen, (0, 0, 255), screen_pos, obj.radius)
            else:
                pygame.draw.circle(screen, (255, 0, 0), screen_pos, obj.radius)

        pygame.display.flip()


if __name__ == "__main__":
    main()
