import pygame
import sys

# Initialize Pygame
pygame.init()

# Set up the screen dimension and create a full-screen window
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("Target Positioning")

# Load the target image
target_image = pygame.image.load("target_large_bullseye.png")
image_width, image_height = 300, 300
target_image = pygame.transform.scale(target_image, (image_width, image_height))

# Get the screen dimensions
screen_info = pygame.display.Info()
screen_width, screen_height = screen_info.current_w, screen_info.current_h

# Calculate the position to center the target image
target_position = ((screen_width - image_width) // 2, (screen_height - image_height) // 2)

# Define a clock to manage the frame rate
clock = pygame.time.Clock()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # Allow exit from fullscreen with the Escape key
            if event.key == pygame.K_ESCAPE:
                running = False

    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw the target image at the specified position
    screen.blit(target_image, target_position)

    # Update the display
    pygame.display.flip()

    # Cap the frame rate at ~60 FPS
    clock.tick(60)

# Clean up
pygame.quit()
sys.exit()