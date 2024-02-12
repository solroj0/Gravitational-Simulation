import pygame
import random
import math
import time
from pygame.math import Vector2


# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
GRAVITY_CONSTANT = 0.1
PARTICLE_RADIUS = 5
DRAG_COEFFICIENT = 0.02  # Adjust this value for desired drag effect
bounce_factor=1.05
FPS = 60

# Color constants
MIN_COLLISIONS = 0
MAX_COLLISIONS = 30
MIN_COLOR = (0, 0, 255)  # Blue
MAX_COLOR = (255, 0, 0)  # Red
SPAWN_PARTICLE_EVENT = pygame.USEREVENT + 1

angle = 0
r = 200


# Create window
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Particle Vortex")

class Creator:
    def __init__(self) -> None:
        pass

    def rotator(self):
       pass

    def spawn_particle(self, particles):
        
        x = CENTER[0] + r * math.cos(math.radians(angle))
        y = CENTER[1] + r * math.sin(math.radians(angle))
        
        
        position = (x, y)
    
        random_velocity = [random.uniform(-1, 1), random.uniform(-1, 1)]  # Random velocity within range
        particles.append(Particle(position, random_velocity))


# Particle class
class Particle:
    def __init__(self, position, velocity):
        self.position = list(position)
        self.velocity = list(velocity)
        self.collisions = 0
        self.last_collision_time = time.time()

    def update(self):
        # Apply gravity towards the center
        dx = CENTER[0] - self.position[0]
        dy = CENTER[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        acceleration = [dx / distance * GRAVITY_CONSTANT, dy / distance * GRAVITY_CONSTANT]
        self.velocity[0] += acceleration[0]
        self.velocity[1] += acceleration[1]
        
        # Apply drag force
        drag_force = [-self.velocity[0] * DRAG_COEFFICIENT, -self.velocity[1] * DRAG_COEFFICIENT]
        self.velocity[0] += drag_force[0]
        self.velocity[1] += drag_force[1]
        
        # Update position
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]
        
        # Bounce off edges
        if self.position[0] <= PARTICLE_RADIUS or self.position[0] >= WIDTH - PARTICLE_RADIUS:
            self.velocity[0] *= -1
        if self.position[1] <= PARTICLE_RADIUS or self.position[1] >= HEIGHT - PARTICLE_RADIUS:
            self.velocity[1] *= -1

        # Update collision count
        if time.time() - self.last_collision_time > 10:  # Reset after 10 seconds
            self.collisions = 0
            self.last_collision_time = time.time()

    def draw(self):
        # Interpolate color based on collision count
        color_scale = (self.collisions - MIN_COLLISIONS) / (MAX_COLLISIONS - MIN_COLLISIONS)
        if color_scale < 0:
            color_scale = 0
        elif color_scale > 1:
            color_scale = 1
        color = (
            int(MIN_COLOR[0] + color_scale * (MAX_COLOR[0] - MIN_COLOR[0])),
            int(MIN_COLOR[1] + color_scale * (MAX_COLOR[1] - MIN_COLOR[1])),
            int(MIN_COLOR[2] + color_scale * (MAX_COLOR[2] - MIN_COLOR[2]))
        )
        pygame.draw.circle(screen, color, (int(self.position[0]), int(self.position[1])), PARTICLE_RADIUS)

def check_collisions(particles, bounce_factor):
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            particle1 = particles[i]
            particle2 = particles[j]
            dx = particle2.position[0] - particle1.position[0]
            dy = particle2.position[1] - particle1.position[1]
            distance = math.sqrt(dx**2 + dy**2)
            if distance <= PARTICLE_RADIUS * 2:  # Collision detected
                # Increment collision count for particles
                particle1.collisions += 1
                particle2.collisions += 1
                # Calculate normal vector
                normal = [dx / distance, dy / distance]
                # Calculate relative velocity
                relative_velocity = [particle2.velocity[0] - particle1.velocity[0],
                                     particle2.velocity[1] - particle1.velocity[1]]
                # Calculate dot product of relative velocity and normal vector
                dot_product = relative_velocity[0] * normal[0] + relative_velocity[1] * normal[1]
                # Calculate impulse
                impulse = 2 * dot_product / (1 + 1) * bounce_factor # Elastic collision
                # Update velocities
                particle1.velocity[0] += impulse * normal[0]
                particle1.velocity[1] += impulse * normal[1]
                particle2.velocity[0] -= impulse * normal[0]
                particle2.velocity[1] -= impulse * normal[1]


def apply_gravity(particles, gravitational_constant):
    for i in range(len(particles)):
        for j in range(len(particles)):
            if i != j:  # Avoid calculating gravity for the same particle
                particle1 = particles[i]
                particle2 = particles[j]
                dx = particle2.position[0] - particle1.position[0]
                dy = particle2.position[1] - particle1.position[1]
                distance = max(math.sqrt(dx**2 + dy**2), 1)  # Avoid division by zero
                # Calculate gravitational force
                force_magnitude = gravitational_constant / distance**2
                # Apply force components to update velocities
                particle1.velocity[0] += force_magnitude * dx / distance
                particle1.velocity[1] += force_magnitude * dy / distance

def main():
    
    global angle
    particles = []
    dragging = False
    running = True
    
    creator = Creator()
    
     # Set up a timer to trigger the custom event at regular intervals (e.g., every 1000 milliseconds)
    pygame.time.set_timer(SPAWN_PARTICLE_EVENT, 1000)  # 1000 milliseconds = 1 second

    while running:
        screen.fill((0, 0, 0))

        angle += 1
        if angle >= 360:
            angle = 0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == SPAWN_PARTICLE_EVENT:
                # Create a new particle at a random position with random velocity
                creator.spawn_particle(particles)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    dragging = True
                    start_pos = pygame.mouse.get_pos()
                    particles.append(Particle(start_pos, [0, 0])) #particle appended to list
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    end_pos = pygame.mouse.get_pos()
                    particles[-1].velocity = [(end_pos[0] - start_pos[0]) / 10, (end_pos[1] - start_pos[1]) / 10]
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False

        # Apply gravity between particles
        apply_gravity(particles, GRAVITY_CONSTANT)

        # Update particles
        for particle in particles:
            particle.update()
            particle.draw()

        # Check for collisions
        check_collisions(particles, bounce_factor)

        pygame.display.flip()
        pygame.time.Clock().tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()