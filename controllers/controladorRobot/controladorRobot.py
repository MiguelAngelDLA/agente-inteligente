# python_controller_10x10.py

from controller import Robot
import math
import numpy as np

# --- Constants ---
MAX_SPEED = 7.0
WHEEL_RADIUS = 0.0985
AXLE_LENGTH = 0.4044
TARGET_TOLERANCE_DIST = 0.05
TARGET_TOLERANCE_ANGLE = 0.05
FIELD_SIZE_X = 10.0
FIELD_SIZE_Y = 10.0
DROP_OFF_POINT = (0.5, 0.5) # Example drop-off location

MOTOR_NAMES = [
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
    "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint"
]

class TiagoController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # --- Initialize Devices ---
        self.motors = {name: self.robot.getDevice(name) for name in MOTOR_NAMES}
        
        self.motors["head_2_joint"].setPosition(0.7) 
        
        self.left_wheel_sensor = self.robot.getDevice("wheel_left_joint_sensor")
        self.right_wheel_sensor = self.robot.getDevice("wheel_right_joint_sensor")
        self.left_wheel_sensor.enable(self.time_step)
        self.right_wheel_sensor.enable(self.time_step)
        
        self.motors["wheel_left_joint"].setPosition(float('inf'))
        self.motors["wheel_right_joint"].setPosition(float('inf'))
        
        # --- Camera Initialization ---
        self.camera = self.robot.getDevice("Astra rgb")
        self.camera.enable(self.time_step)
        self.image_width = self.camera.getWidth()
        self.image_height = self.camera.getHeight()
        self.camera_fov = self.camera.getFov()

        # --- State Machine & Data ---
        self.state = 'MAPPING'
        self.gnome_locations = []
        self.patrol_points = self.generate_patrol_points(step=1.0)
        self.current_target_index = 0
        
        # --- Odometry ---
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.ps_left_prev = self.left_wheel_sensor.getValue()
        self.ps_right_prev = self.right_wheel_sensor.getValue()
        
        # --- Navigation ---
        self.current_target_pos = None
        self.navigation_substate = 'IDLE'

    def generate_patrol_points(self, step=1.0):
        """Generates a list of waypoints for a lawnmower pattern."""
        points = []
        x = step / 2.0
        # Create a back-and-forth pattern
        while x < FIELD_SIZE_X:
            points.append((x, step / 2.0))
            points.append((x, FIELD_SIZE_Y - step / 2.0))
            x += step
            if x >= FIELD_SIZE_X: break
            points.append((x, FIELD_SIZE_Y - step / 2.0))
            points.append((x, step / 2.0))
            x += step
        return points

    def navigate_to_target(self):
        """A sub-state machine to handle rectilinear navigation."""
        if self.current_target_pos is None:
            return True # Navigation is complete

        target_x, target_y = self.current_target_pos
        
        if self.navigation_substate == 'IDLE':
            self.navigation_substate = 'TURNING_FOR_X'

        elif self.navigation_substate == 'TURNING_FOR_X':
            dx = target_x - self.x
            if abs(dx) < TARGET_TOLERANCE_DIST:
                self.navigation_substate = 'TURNING_FOR_Y'
                return False

            target_angle = 0.0 if dx >= 0 else math.pi
            if abs(self.theta - target_angle) > TARGET_TOLERANCE_ANGLE:
                speed = 1.0 if (target_angle - self.theta) > 0 else -1.0
                self.motors["wheel_left_joint"].setVelocity(-speed)
                self.motors["wheel_right_joint"].setVelocity(speed)
            else:
                self.stop()
                self.start_pos = (self.x, self.y)
                self.navigation_substate = 'MOVING_IN_X'
        
        elif self.navigation_substate == 'MOVING_IN_X':
            dist_traveled = abs(self.x - self.start_pos[0])
            target_dist = abs(target_x - self.start_pos[0])
            if dist_traveled < target_dist:
                self.motors["wheel_left_joint"].setVelocity(MAX_SPEED * 0.5)
                self.motors["wheel_right_joint"].setVelocity(MAX_SPEED * 0.5)
            else:
                self.stop()
                self.navigation_substate = 'TURNING_FOR_Y'

        elif self.navigation_substate == 'TURNING_FOR_Y':
            dy = target_y - self.y
            if abs(dy) < TARGET_TOLERANCE_DIST:
                self.navigation_substate = 'MOVING_IN_Y'
                return False
            
            target_angle = math.pi / 2.0 if dy >= 0 else -math.pi / 2.0
            angle_diff = target_angle - self.theta
            if angle_diff < -math.pi: angle_diff += 2 * math.pi
            if angle_diff > math.pi: angle_diff -= 2 * math.pi
            
            if abs(angle_diff) > TARGET_TOLERANCE_ANGLE:
                speed = 1.0 if angle_diff > 0 else -1.0
                self.motors["wheel_left_joint"].setVelocity(-speed)
                self.motors["wheel_right_joint"].setVelocity(speed)
            else:
                self.stop()
                self.start_pos = (self.x, self.y)
                self.navigation_substate = 'MOVING_IN_Y'

        elif self.navigation_substate == 'MOVING_IN_Y':
            dist_traveled = abs(self.y - self.start_pos[1])
            target_dist = abs(target_y - self.start_pos[1])
            if dist_traveled < target_dist:
                self.motors["wheel_left_joint"].setVelocity(MAX_SPEED * 0.5)
                self.motors["wheel_right_joint"].setVelocity(MAX_SPEED * 0.5)
            else:
                self.stop()
                self.navigation_substate = 'IDLE'
                self.current_target_pos = None
                return True
        
        return False # Navigation is still in progress

    def stop(self):
        self.motors["wheel_left_joint"].setVelocity(0.0)
        self.motors["wheel_right_joint"].setVelocity(0.0)

    def update_odometry(self):
        ps_left = self.left_wheel_sensor.getValue()
        ps_right = self.right_wheel_sensor.getValue()
        dist_left = (ps_left - self.ps_left_prev) * WHEEL_RADIUS
        dist_right = (ps_right - self.ps_right_prev) * WHEEL_RADIUS
        self.ps_left_prev = ps_left
        self.ps_right_prev = ps_right
        delta_dist = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / AXLE_LENGTH
        self.x += delta_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
    def find_gnomes(self):
        """Processes a camera image to find gnomes and adds them to the list."""
        image_data = self.camera.getImage()
        if not image_data: return

        image = np.frombuffer(image_data, np.uint8).reshape((self.image_height, self.image_width, 4))

        # TUNE THESE VALUES for your specific lighting
        RED_THRESHOLD = 150
        GREEN_MAX = 100
        BLUE_MAX = 100
        MIN_BLOB_SIZE = 20

        red_pixels = []
        for y in range(self.image_height):
            for x in range(self.image_width):
                pixel = image[y, x]
                if pixel[2] > RED_THRESHOLD and pixel[1] < GREEN_MAX and pixel[0] < BLUE_MAX:
                    red_pixels.append((x, y))

        if len(red_pixels) < MIN_BLOB_SIZE: return

        center_x = sum([p[0] for p in red_pixels]) / len(red_pixels)
        angle_per_pixel = self.camera_fov / self.image_width
        angle_to_gnome = (center_x - self.image_width / 2.0) * angle_per_pixel
        
        # Simple distance estimation: assume gnomes detected are about 1m away
        DETECTION_DISTANCE = 1.0
        gnome_world_x = self.x + DETECTION_DISTANCE * math.cos(self.theta + angle_to_gnome)
        gnome_world_y = self.y + DETECTION_DISTANCE * math.sin(self.theta + angle_to_gnome)
        
        new_gnome_pos = (gnome_world_x, gnome_world_y)

        # Check for duplicates before adding
        is_duplicate = False
        for existing_pos in self.gnome_locations:
            if math.dist(new_gnome_pos, existing_pos) < 0.5:
                is_duplicate = True
                break
        
        if not is_duplicate:
            print(f"New gnome detected! Estimated Pos: ({gnome_world_x:.2f}, {gnome_world_y:.2f})")
            self.gnome_locations.append(new_gnome_pos)

    def pickup_sequence(self):
        # TODO: Implement the arm and gripper sequence for picking up a gnome.
        print("Executing pickup sequence...")
    
    def dropoff_sequence(self):
        # TODO: Implement the arm and gripper sequence for dropping off a gnome.
        print("Executing dropoff sequence...")

    def run(self):
        """The main control loop of the robot."""
        while self.robot.step(self.time_step) != -1:
            self.update_odometry()
            
            # --- MAPPING PHASE ---
            if self.state == 'MAPPING':
                self.find_gnomes()
                
                if self.current_target_pos is None:
                    if self.current_target_index < len(self.patrol_points):
                        self.current_target_pos = self.patrol_points[self.current_target_index]
                        print(f"Mapping: Navigating to patrol point {self.current_target_index+1}/{len(self.patrol_points)}: {self.current_target_pos}")
                    else:
                        print(f"\nMapping phase complete. Found {len(self.gnome_locations)} gnomes.")
                        if self.gnome_locations:
                            self.state = 'COLLECTING'
                        else:
                            self.state = 'FINISHED'
                        continue

                if self.navigate_to_target():
                    self.current_target_index += 1

            # --- COLLECTING PHASE ---
            elif self.state == 'COLLECTING':
                print(f"Collecting gnome 1/{len(self.gnome_locations)} at {self.gnome_locations[0]}")
                self.current_target_pos = self.gnome_locations[0]
                self.state = 'NAVIGATING_TO_GNOME'

            elif self.state == 'NAVIGATING_TO_GNOME':
                if self.navigate_to_target():
                    print("Arrived at gnome.")
                    self.state = 'PICKING_UP_GNOME'

            elif self.state == 'PICKING_UP_GNOME':
                self.pickup_sequence()
                print("Gnome collected. Delivering to drop-off point.")
                self.current_target_pos = DROP_OFF_POINT
                self.state = 'NAVIGATING_TO_DROPOFF'
                
            elif self.state == 'NAVIGATING_TO_DROPOFF':
                if self.navigate_to_target():
                    print("Arrived at drop-off point.")
                    self.state = 'DROPPING_OFF_GNOME'
            
            elif self.state == 'DROPPING_OFF_GNOME':
                self.dropoff_sequence()
                # Remove the gnome we just delivered from the list
                self.gnome_locations.pop(0)
                if self.gnome_locations:
                    # If there are more gnomes, go back to the COLLECTING state
                    self.state = 'COLLECTING'
                else:
                    # Otherwise, we are finished
                    print("\nAll gnomes collected! Task complete.")
                    self.state = 'FINISHED'

            elif self.state == 'FINISHED':
                self.stop()

# --- Main Execution ---
if __name__ == "__main__":
    controller = TiagoController()
    controller.run()