import pygame
import math

# Initialize Pygame
pygame.init()

# Set up the drawing window
screen = pygame.display.set_mode([800, 600])

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Define the truck and trailer properties
truck_width, truck_height = 70.3702, 200
trailer_width, trailer_height = 70.3708, 114.2129
pivot_joint = (400, 300)  # Position of the pivot joint (back of the truck)
trailer_angle = 1.5
vehicle_speed = 0
steering_angle = 0
left_wheel_angle = 0
right_wheel_angle = 0

# Font for dashboard
font = pygame.font.SysFont(None, 24)

# Function to draw the truck
def draw_truck(surface, pivot):
    truck_rect = pygame.Rect(0, 0, truck_width, truck_height)
    truck_rect.center = (pivot[0], pivot[1] - truck_height // 2 - 13.8888)  # Position the truck above the pivot joint
    truck_surface = pygame.Surface((truck_width, truck_height))
    truck_surface.fill(BLACK)
    surface.blit(truck_surface, truck_rect.topleft)
    
    truck_p_rect = pygame.Rect(0, 0, 1, 14)
    truck_p_rect.center = (pivot[0], pivot[1] - 14 // 2)
    truck_p_surface = pygame.Surface((4, 14))
    truck_p_surface.fill(BLACK)
    surface.blit(truck_p_surface, truck_p_rect.topleft)

# Function to draw the trailer with rotation
def draw_trailer(surface, pivot, angle):
    trailer_p_center_x = pivot[0] + math.sin(math.radians(angle)) * (39.2889/ 2)
    trailer_p_center_y = pivot[1] + math.cos(math.radians(angle)) * (39.2889/ 2)
    trailer_p_surface = pygame.Surface((4, 39.2889))
    trailer_p_surface.fill(RED)
    trailer_p_surface.set_colorkey(WHITE)
    rotated_p_trailer = pygame.transform.rotate(trailer_p_surface, angle)
    rotated_p_trailer_rect = rotated_p_trailer.get_rect(center=(trailer_p_center_x, trailer_p_center_y))
    surface.blit(rotated_p_trailer, rotated_p_trailer_rect.topleft)
    
    trailer_center_x = pivot[0] + math.sin(math.radians(angle)) * (39.2889 + trailer_height / 2)
    trailer_center_y = pivot[1] + math.cos(math.radians(angle)) * (39.2889 + trailer_height / 2)
    trailer_surface = pygame.Surface((trailer_width, trailer_height))
    trailer_surface.fill(RED)
    trailer_surface.set_colorkey(WHITE)
    rotated_trailer = pygame.transform.rotate(trailer_surface, angle)
    rotated_trailer_rect = rotated_trailer.get_rect(center=(trailer_center_x, trailer_center_y))
    surface.blit(rotated_trailer, rotated_trailer_rect.topleft)

# Function to draw the dashboard on the left side
def draw_left_dashboard(surface, speed, state, steering_angle, left_wheel, right_wheel):
    left_dashboard_rect = pygame.Rect(10, 10, 220, 180)
    pygame.draw.rect(surface, BLACK, left_dashboard_rect, 2)

    draw_text(surface, 'Speed:', (20, 20), font)
    draw_text(surface, f'{speed:.2f}', (180, 20), font)

    draw_text(surface, 'State:', (20, 50), font)
    draw_text(surface, state, (180, 50), font)

    draw_text(surface, 'Steering Angle:', (20, 80), font)
    draw_text(surface, f'{steering_angle:.2f}', (180, 80), font)

    draw_text(surface, 'Left Wheel:', (20, 110), font)
    draw_text(surface, f'{left_wheel:.2f}', (180, 110), font)

    draw_text(surface, 'Right Wheel:', (20, 140), font)
    draw_text(surface, f'{right_wheel:.2f}', (180, 140), font)

# Function to draw the dashboard on the right side
def draw_right_dashboard(surface, articulation_angles):
    right_dashboard_rect = pygame.Rect(570, 10, 220, 180)
    pygame.draw.rect(surface, BLACK, right_dashboard_rect, 2)

    draw_text(surface, 'Articulation Angles:', (580, 20), font)

    draw_text(surface, 'Ground Truth:', (580, 50), font)
    draw_text(surface, f'{articulation_angles["ground_truth"]:.2f}', (720, 50), font)

    draw_text(surface, 'ArUco Marker:', (580, 80), font)
    draw_text(surface, f'{articulation_angles["aruco_marker"]:.2f}', (720, 80), font)

    draw_text(surface, 'Point Cloud:', (580, 110), font)
    draw_text(surface, f'{articulation_angles["point_cloud"]:.2f}', (720, 110), font)

    draw_text(surface, 'Prediction:', (580, 140), font)
    draw_text(surface, f'{articulation_angles["prediction"]:.2f}', (720, 140), font)

# Helper function to draw text on the screen
def draw_text(surface, text, pos, font, color=BLACK):
    text_surface = font.render(text, True, color)
    surface.blit(text_surface, pos)

# Run until the user asks to quit
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Fill the background with white
    screen.fill(WHITE)

    # Get keys pressed
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        steering_angle += 1
        left_wheel_angle += 1
        right_wheel_angle += 1
        trailer_angle += 1  # Rotate the trailer
    if keys[pygame.K_RIGHT]:
        steering_angle -= 1
        left_wheel_angle -= 1
        right_wheel_angle -= 1
        trailer_angle -= 1  # Rotate the trailer
    if keys[pygame.K_UP]:
        vehicle_speed += 0.1
    if keys[pygame.K_DOWN]:
        vehicle_speed -= 0.1

    # Draw the truck and trailer
    draw_truck(screen, pivot_joint)
    draw_trailer(screen, pivot_joint, trailer_angle)

    # Articulation angles (dummy values, replace with actual calculations if needed)
    articulation_angles = {
        "ground_truth": 10.0,
        "aruco_marker": 5.0,
        "point_cloud": 2.5,
        "prediction": 3.0,
    }

    # Draw the dashboards
    draw_left_dashboard(screen, vehicle_speed, "P", steering_angle, left_wheel_angle, right_wheel_angle)
    draw_right_dashboard(screen, articulation_angles)

    # Flip the display
    pygame.display.flip()

    # Cap the frame rate
    pygame.time.Clock().tick(30)

# Done! Time to quit.
pygame.quit()
