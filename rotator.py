import pygame
import math

# Initialize Pygame
pygame.init()

# Set up the screen
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Rotating Object")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Main loop
clock = pygame.time.Clock()
angle = 0
radius = 100

running = True
while running:
    screen.fill(WHITE)

    # Calculate the position of the object based on angle and radius
    x = screen_width // 2 + radius * math.cos(math.radians(angle))
    y = screen_height // 2 + radius * math.sin(math.radians(angle))

    # Draw the rotating object (a simple circle)
    pygame.draw.circle(screen, RED, (int(x), int(y)), 20)

    # Increment angle for rotation
    angle += 1
    if angle >= 360:
        angle = 0

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()
