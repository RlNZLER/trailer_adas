import pygame
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class TruckHUD(Node):
    def __init__(self):
        super().__init__('truck_hud')
        
        # Define the truck and trailer properties
        self.truck_width, self.truck_height = 70.3702, 200
        self.trailer_width, self.trailer_height = 70.3708, 114.2129
        self.pivot = (400, 300)  # Position of the pivot joint (back of the truck)
        
        self.aa_ground_truth = 0
        self.aa_aruco_marker = 0
        self.aa_range = 0
        self.aa_filtered = 0
        self.aa_prediction = 0
        
        self.trailer_angle = 0
        self.vehicle_speed = 0
        self.state = 'P' # P: Parked, D: Drive, R: Reverse
        
        self.steering_angle = 0
        self.left_wheel_angle = 0
        self.right_wheel_angle = 0

        # Font for dashboard
        

        self.aa_ground_truth_sub_ = self.create_subscription(Float64,'articulation_angle/ground_truth', self.aa_ground_truth_callback, 10)
        self.aa_aruco_marker_sub_ = self.create_subscription(Float64,'articulation_angle/markers', self.aa_aruco_marker_callback, 10)
        self.aa_point_cloud_sub_ = self.create_subscription(Float64,'articulation_angle/range', self.aa_point_cloud_callback, 10)
        self.aa_filtered_sub_ = self.create_subscription(Float64,'articulation_angle/filtered', self.aa_filtered_callback, 10)
        # self.aa_prediction_sub_ = self.create_subscription(Float32,'articulation_angle/prediction', self.aa_prediction_callback, 10)
        
        self.vehicle_status_sub_ = self.create_subscription(Float64MultiArray,'vehicle_status', self.vehicle_status_callback, 10)


    def aa_ground_truth_callback(self, msg):
        self.aa_ground_truth = msg.data
        
    def aa_aruco_marker_callback(self, msg):
        self.aa_aruco_marker = msg.data
        
    def aa_point_cloud_callback(self, msg):
        self.aa_range = msg.data
        
    def aa_filtered_callback(self, msg):
        self.aa_filtered = msg.data
        
    # def aa_prediction_callback(self, msg):
    #     self.aa_prediction = msg.data
    
    def vehicle_status_callback(self, msg):
        if msg.data[0] == 0:
            self.state = 'P'
        elif msg.data[0] > 0:
            self.state = 'D'
        else:
            self.state = 'R'
        self.vehicle_speed = abs(msg.data[0])
        self.steering_angle = msg.data[2]
        self.left_wheel_angle = msg.data[3]
        self.right_wheel_angle = msg.data[4]

    def draw_dashed_line(self, surface, color, start_pos, end_pos, width=1, dash_length=10):
        # Calculate the length of the line
        total_length = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        # Calculate the number of dashes
        num_dashes = int(total_length // dash_length)

        for i in range(num_dashes + 1):
            start = (
                start_pos[0] + (end_pos[0] - start_pos[0]) * i / num_dashes,
                start_pos[1] + (end_pos[1] - start_pos[1]) * i / num_dashes,
            )
            end = (
                start_pos[0] + (end_pos[0] - start_pos[0]) * (i + 0.5) / num_dashes,
                start_pos[1] + (end_pos[1] - start_pos[1]) * (i + 0.5) / num_dashes,
            )
            pygame.draw.line(surface, color, start, end, width)

    # Function to draw the truck
    def draw_truck(self, surface):
        truck_rect = pygame.Rect(0, 0, self.truck_width, self.truck_height)
        truck_rect.center = (self.pivot[0], self.pivot[1] - self.truck_height // 2 - 13.8888)  # Position the truck above the pivot joint
        truck_surface = pygame.Surface((self.truck_width, self.truck_height))
        truck_surface.fill(BLACK)
        surface.blit(truck_surface, truck_rect.topleft)
        
        truck_p_rect = pygame.Rect(0, 0, 1, 14)
        truck_p_rect.center = (self.pivot[0], self.pivot[1] - 14 // 2)
        truck_p_surface = pygame.Surface((4, 14))
        truck_p_surface.fill(BLACK)
        surface.blit(truck_p_surface, truck_p_rect.topleft)

        # Draw dashed lines on both sides of the truck
        left_line_start = (truck_rect.left, truck_rect.bottom)
        left_line_end = (truck_rect.left, truck_rect.bottom + 250)
        right_line_start = (truck_rect.right, truck_rect.bottom)
        right_line_end = (truck_rect.right, truck_rect.bottom + 250)

        self.draw_dashed_line(surface, BLUE, left_line_start, left_line_end, width=2, dash_length=10)
        self.draw_dashed_line(surface, BLUE, right_line_start, right_line_end, width=2, dash_length=10)

    # Function to draw the trailer with rotation
    def draw_trailer(self, surface):
        angle = self.aa_filtered # Change articulation angle later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        trailer_p_center_x = self.pivot[0] + math.sin(angle) * (39.2889/ 2)
        trailer_p_center_y = self.pivot[1] + math.cos(angle) * (39.2889/ 2)
        trailer_p_surface = pygame.Surface((4, 39.2889))
        trailer_p_surface.fill(RED)
        trailer_p_surface.set_colorkey(WHITE)
        rotated_p_trailer = pygame.transform.rotate(trailer_p_surface, math.degrees(angle))
        rotated_p_trailer_rect = rotated_p_trailer.get_rect(center=(trailer_p_center_x, trailer_p_center_y))
        surface.blit(rotated_p_trailer, rotated_p_trailer_rect.topleft)
        
        trailer_center_x = self.pivot[0] + math.sin(angle) * (39.2889 + self.trailer_height / 2)
        trailer_center_y = self.pivot[1] + math.cos(angle) * (39.2889 + self.trailer_height / 2)
        trailer_surface = pygame.Surface((self.trailer_width, self.trailer_height))
        trailer_surface.fill(RED)
        trailer_surface.set_colorkey(WHITE)
        rotated_trailer = pygame.transform.rotate(trailer_surface, math.degrees(angle))
        rotated_trailer_rect = rotated_trailer.get_rect(center=(trailer_center_x, trailer_center_y))
        surface.blit(rotated_trailer, rotated_trailer_rect.topleft)
        
        # Draw the line extending from the back of the trailer
        # Calculate the position of the back center of the trailer
        back_of_trailer_x = trailer_center_x + math.sin(angle) * (self.trailer_height / 2)
        back_of_trailer_y = trailer_center_y + math.cos(angle) * (self.trailer_height / 2)
        
        # Calculate the endpoint of the line extending 50 units from the back of the trailer
        line_end_x = back_of_trailer_x + math.sin(angle) * 50
        line_end_y = back_of_trailer_y + math.cos(angle) * 50
        
        # Draw the line
        pygame.draw.line(surface, GREEN, (back_of_trailer_x, back_of_trailer_y), (line_end_x, line_end_y), 3)

    # Function to draw the dashboard on the left side
    def draw_left_dashboard(self, surface, font):
        left_dashboard_rect = pygame.Rect(10, 10, 220, 180)
        pygame.draw.rect(surface, BLACK, left_dashboard_rect, 2)

        self.draw_text(surface, 'Speed:', (20, 20), font)
        self.draw_text(surface, f'{self.vehicle_speed:.2f}', (180, 20), font)

        self.draw_text(surface, 'State:', (20, 50), font)
        self.draw_text(surface, self.state, (180, 50), font)

        self.draw_text(surface, 'Steering Angle:', (20, 80), font)
        self.draw_text(surface, f'{self.steering_angle:.2f}', (180, 80), font)

        self.draw_text(surface, 'Left Wheel:', (20, 110), font)
        self.draw_text(surface, f'{self.left_wheel_angle:.2f}', (180, 110), font)

        self.draw_text(surface, 'Right Wheel:', (20, 140), font)
        self.draw_text(surface, f'{self.right_wheel_angle:.2f}', (180, 140), font)

    # Function to draw the dashboard on the right side
    def draw_right_dashboard(self, surface, font):
        right_dashboard_rect = pygame.Rect(570, 10, 220, 180)
        pygame.draw.rect(surface, BLACK, right_dashboard_rect, 2)

        self.draw_text(surface, 'Articulation Angles:', (580, 20), font)

        self.draw_text(surface, 'Ground Truth:', (580, 50), font)
        self.draw_text(surface, f'{self.aa_ground_truth:.2f}', (720, 50), font)

        self.draw_text(surface, 'ArUco Marker:', (580, 80), font)
        self.draw_text(surface, f'{self.aa_aruco_marker:.2f}', (720, 80), font)

        self.draw_text(surface, 'Range Sensor:', (580, 110), font)
        self.draw_text(surface, f'{self.aa_range:.2f}', (720, 110), font)

        self.draw_text(surface, 'Kalman Filter:', (580, 140), font)
        self.draw_text(surface, f'{self.aa_filtered:.2f}', (720, 140), font)

    # Helper function to draw text on the screen
    def draw_text(self, surface, text, pos, font, color=BLACK):
        text_surface = font.render(text, True, color)
        surface.blit(text_surface, pos)

    def run_truck_hud(self):
        pygame.init()
        font = pygame.font.SysFont(None, 24)
        screen = pygame.display.set_mode([800, 600])

        running = True
        while running:
            rclpy.spin_once(self, timeout_sec=0)  # Non-blocking spin that allows ROS callbacks to process

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill(WHITE)

            # Draw the truck and trailer
            self.draw_truck(screen)
            self.draw_trailer(screen)

            # Draw the dashboards
            self.draw_left_dashboard(screen, font)
            self.draw_right_dashboard(screen, font)

            # Flip the display
            pygame.display.flip()

            # Cap the frame rate
            pygame.time.Clock().tick(30)

        # Done! Time to quit.
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    truck_hud = TruckHUD()
    
    truck_hud.run_truck_hud()
    
    rclpy.spin(truck_hud)
    truck_hud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()