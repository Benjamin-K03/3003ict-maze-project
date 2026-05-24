from controller import Robot
import heapq
import math
from enum import Enum

# System constants
TIME_STEP = 32
CELL_SIZE = 0.1
MAX_SPEED = 6.28

# Heading: 0=East(+X), 1=North(+Y), 2=West(-X), 3=South(-Y)
DIR_OFFSETS = [(1, 0), (0, 1), (-1, 0), (0, -1)]

# FSM enum
class State(Enum):
    STARTUP = 1
    PLAN = 2
    TURN = 3
    MOVE = 4
    SAFE_REVERSE = 5
    GOAL_REACHED = 6
    FAILSAFE = 7

# D* lite class
class DStarLite:
    def __init__(self, start, goal, cols=100, rows=100):
        self.start = start
        self.goal = goal
        self.km = 0
        self.U = []
        self.rhs = {}
        self.g = {}
        self.obstacles = set()
        self.cols = cols
        self.rows = rows
        
        for x in range(cols):
            for y in range(rows):
                self.rhs[(x, y)] = float("inf")
                self.g[(x, y)] = float("inf")
        
        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (*self.calc_key(self.goal), self.goal))

    def calc_key(self, s):
        min_val = min(self.g[s], self.rhs[s])
        return (min_val + self.heuristic(self.start, s) + self.km, min_val)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, u):
        neighbors = []
        for dx, dy in DIR_OFFSETS:
            nx, ny = u[0] + dx, u[1] + dy
            if 0 <= nx < self.cols and 0 <= ny < self.rows:
                if (nx, ny) not in self.obstacles:
                    neighbors.append((nx, ny))
        return neighbors

    def update_vertex(self, u):
        if u != self.goal:
            min_rhs = float("inf")
            for n in self.get_neighbors(u):
                min_rhs = min(min_rhs, self.g[n] + 1)
            self.rhs[u] = min_rhs
            
        self.U = [item for item in self.U if item[2] != u]
        heapq.heapify(self.U)
        
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (*self.calc_key(u), u))

    def compute_shortest_path(self):
        while self.U and (self.U[0][:2] < self.calc_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            k_old = (self.U[0][0], self.U[0][1])
            u = heapq.heappop(self.U)[2]
            k_new = self.calc_key(u)
            
            if k_old < k_new:
                heapq.heappush(self.U, (*k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for n in self.get_neighbors(u):
                    self.update_vertex(n)
            else:
                self.g[u] = float("inf")
                for n in self.get_neighbors(u) + [u]:
                    self.update_vertex(n)

    def add_obstacle(self, obs):
        if obs not in self.obstacles:
            self.obstacles.add(obs)
            self.update_vertex(obs)
            for n in self.get_neighbors(obs):
                self.update_vertex(n)
            self.compute_shortest_path()

    def get_next_move(self, current):
        min_cost = float("inf")
        best_node = None
        for n in self.get_neighbors(current):
            cost = self.g[n] + 1
            if cost < min_cost:
                min_cost = cost
                best_node = n
        return best_node

# E-puck robot controller
class MazeRunner:
    def __init__(self, gx, gy):
        self.robot = Robot()
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Enable Absolute Positioning Sensors
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(TIME_STEP)

        # IR Sensors for dynamic obstacle detection
        self.ds = []
        for i in range(8):
            sensor = self.robot.getDevice(f"ps{i}")
            sensor.enable(TIME_STEP)
            self.ds.append(sensor)
            
        # Hardware Additions: Camera, LEDs, and Speaker
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(TIME_STEP)
            
        self.led_red = self.robot.getDevice("led0")  # Red LED on outer ring
        self.led_green = self.robot.getDevice("led8") # Green body LED
        self.speaker = self.robot.getDevice("speaker") # Internal Speaker Node

        # State Tracking
        self.x, self.y = 0, 0
        self.target_x, self.target_y = 0, 0
        self.grid_heading = 0 # 0=E, 1=N, 2=W, 3=S
        self.goal = (gx, gy)

        self.dstar = None # Setup dstar in main loop
        self.state = State.STARTUP
        
        # Flags
        self.green_detected_previously = False
        self.last_beep_time = 0

    def get_current_heading(self):
        """
        Converts Webots Compass 3D vector to a global yaw angle.
        East = 0, North = pi/2, West = pi, South = -pi/2
        """
        c = self.compass.getValues()
        theta = math.pi / 2.0 - math.atan2(c[1], c[0])
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_forward_obstacle(self):
        """Checks front IR sensors to detect unmapped walls"""
        if self.ds[0].getValue() > 90 or self.ds[7].getValue() > 90:
            dx, dy = DIR_OFFSETS[self.grid_heading]
            nx, ny = self.x + dx, self.y + dy
            if 0 <= nx < self.dstar.cols and 0 <= ny < self.dstar.rows:
                return (nx, ny)
        return None
        
    def detect_green_object(self):
        """Reads Camera Array to detect green."""
        if not self.camera:
            return False
            
        img = self.camera.getImageArray()
        if not img:
            return False
            
        w, h = self.camera.getWidth(), self.camera.getHeight()
        green_pixels = 0
        
        for x in range(w):
            for y in range(h):
                r, g, b = img[x][y]
                # A pixel is considered green if G is distinctly higher than R and B
                if g > r + 20 and g > b + 20:
                    green_pixels += 1
                    
        # Check if green fills at least 5% of the camera"s FOV
        return green_pixels > (w * h * 0.05)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            
           # State Machine Loop
            if self.state == State.STARTUP:
                pos = self.gps.getValues()
                # Wait 1 step for GPS/Compass to initialize and return non-NaN data
                if not math.isnan(pos[0]):
                    # Set the true starting grid coordinates
                    self.x = round(pos[0] / CELL_SIZE)
                    self.y = round(pos[1] / CELL_SIZE)
                    
                    # Initialize D* Lite with true coordinates and compute path
                    self.dstar = DStarLite((self.x, self.y), self.goal)
                    self.dstar.compute_shortest_path()
                    
                    self.state = State.PLAN
                continue
            
            # Update global observations (Camera)
            green_found = self.detect_green_object()
            if green_found and not self.green_detected_previously:
                 at_target = ((self.x, self.y) == self.goal)
                 print(f"Green object found! At target coordinates: {at_target}")
            self.green_detected_previously = green_found

            pos = self.gps.getValues()
            current_x_m = pos[0]
            current_y_m = pos[1]

            if self.state == State.PLAN:
                # Re-sync exact grid coordinates based on GPS
                self.x = round(current_x_m / CELL_SIZE)
                self.y = round(current_y_m / CELL_SIZE)

                if (self.x, self.y) == self.goal:
                    self.left_motor.setVelocity(MAX_SPEED)
                    self.right_motor.setVelocity(-MAX_SPEED)

                    if green_found:
                        self.state = State.GOAL_REACHED
                        print("Goal Reached Successfully and Green Object Visible! Entered GOAL_STATE.")
                        
                    else:
                        print(f"At correct coordinate {self.goal}, but waiting for green object...")
                    continue

                # Update D* Lite with current position
                self.dstar.start = (self.x, self.y)

                # Check if we are spawned facing a wall immediately
                obs = self.get_forward_obstacle()
                if obs:
                    print(f"Obstacle sensed at {obs}. Replanning...")
                    self.dstar.add_obstacle(obs)

                next_node = self.dstar.get_next_move((self.x, self.y))
                if not next_node:
                    print("No path available! Trapped. Entering FAILSAFE.")
                    self.state = State.FAILSAFE
                    continue
                
                # Determine intended grid direction
                dx = next_node[0] - self.x
                dy = next_node[1] - self.y
                if dx == 1: self.grid_heading = 0
                elif dy == 1: self.grid_heading = 1
                elif dx == -1: self.grid_heading = 2
                elif dy == -1: self.grid_heading = 3

                # Calculate exact physical target coordinates using grid
                self.target_x = next_node[0] * CELL_SIZE
                self.target_y = next_node[1] * CELL_SIZE

                print(f"At Grid ({self.x}, {self.y}). Moving to: {next_node}.")
                self.state = State.TURN

            elif self.state == State.TURN:
                target_angle = math.atan2(self.target_y - current_y_m, self.target_x - current_x_m)
                current_angle = self.get_current_heading()
                
                angle_diff = self.normalize_angle(target_angle - current_angle)

                # If facing target within ~2.8 degrees, start moving
                if abs(angle_diff) < 0.05:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state = State.MOVE
                else:
                    # Proportional turn based on compass error
                    turn_speed = max(min(abs(angle_diff) * 5.0, MAX_SPEED * 0.5), 0.5)
                    if angle_diff > 0: # Turn Left
                        self.left_motor.setVelocity(-turn_speed)
                        self.right_motor.setVelocity(turn_speed)
                    # Turn Right
                    else:
                        self.left_motor.setVelocity(turn_speed)
                        self.right_motor.setVelocity(-turn_speed)

            elif self.state == State.MOVE:
                # Check for unexpected walls mid-transit
                obs = self.get_forward_obstacle()
                if obs:
                    print(f"Wall detected dynamically at {obs}! Reversing to center.")
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.dstar.add_obstacle(obs)
                    self.state = State.SAFE_REVERSE
                    continue

                # Check if reached exact GPS target
                distance = math.hypot(self.target_x - current_x_m, self.target_y - current_y_m)
                if distance < 0.015: # Within 1.5 cm of exact center
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state = State.PLAN
                else:
                    # GPS-guided straight line P-Controller
                    target_angle = math.atan2(self.target_y - current_y_m, self.target_x - current_x_m)
                    current_angle = self.get_current_heading()
                    angle_diff = self.normalize_angle(target_angle - current_angle)

                    base_speed = MAX_SPEED * 0.8
                    # Steer dynamically towards the target GPS coordinate
                    correction = angle_diff * 10.0 
                    
                    vl = base_speed - correction
                    vr = base_speed + correction

                    self.left_motor.setVelocity(max(min(vl, MAX_SPEED), -MAX_SPEED))
                    self.right_motor.setVelocity(max(min(vr, MAX_SPEED), -MAX_SPEED))

            elif self.state == State.SAFE_REVERSE:
                # Calculate original cell center to back into
                center_x = self.x * CELL_SIZE
                center_y = self.y * CELL_SIZE
                
                distance = math.hypot(center_x - current_x_m, center_y - current_y_m)
                if distance < 0.015:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state = State.PLAN
                else:
                    # Reverse blindly straight back
                    self.left_motor.setVelocity(-MAX_SPEED * 0.5)
                    self.right_motor.setVelocity(-MAX_SPEED * 0.5)
                    
            elif self.state == State.GOAL_REACHED:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                
                # Toggle Green LED
                if self.led_green:
                    led_on = (int(self.robot.getTime() * 4) % 2) == 0
                    self.led_green.set(1 if led_on else 0)

                # Play Beep every 2 seconds
                if self.robot.getTime() - self.last_beep_time > 2.0:
                    if self.speaker:
                        try:
                            self.speaker.playSound(self.speaker, self.speaker, "../../sounds/victory.mp3", 1.0, 1.0, 0, False)
                        except Exception:
                            pass
                            
                    self.last_beep_time = self.robot.getTime()

            elif self.state == State.FAILSAFE:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                
                # Toggle Red LED
                if self.led_red:
                    led_on = (int(self.robot.getTime() * 4) % 2) == 0
                    self.led_red.set(1 if led_on else 0)
                    
                # Play Beep every 2 seconds
                if self.robot.getTime() - self.last_beep_time > 2.0:
                    print("[FAILSAFE] No path to goal...")
                    
                    if self.speaker:
                        try:
                            self.speaker.playSound(self.speaker, self.speaker, "../../sounds/fail.mp3", 1.0, 1.0, 0, False)
                        except Exception:
                            pass
                            
                    self.last_beep_time = self.robot.getTime()

if __name__ == "__main__":
    gx, gy = 7, 14      # Normal test
    # gx, gy = 9, 2       # Failsafe test
    controller = MazeRunner(gx=gx, gy=gy) # Set goal coordinates
    controller.run()