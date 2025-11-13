from controller import Robot, Motor, PositionSensor
import time
import math

class FetchController:
    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        print(f"Fetch Controller initialized. Time step: {self.timestep} ms")

        # Robot starting pose
        self.x = -8.22319
        self.y = -6.2543
        qx, qy, qz, qw = 0.0015924, 0.00160815, -0.999997, 2.22521
        self.theta = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        # Define joints and max velocities
        self.joint_info = {
            'shoulder_pan_joint': 1.25,
            'shoulder_lift_joint': 1.45,
            'upperarm_roll_joint': 1.57,
            'elbow_flex_joint': 1.52,
            'forearm_roll_joint': 1.57,
            'wrist_flex_joint': 2.26,
            'wrist_roll_joint': 2.26,
            'l_gripper_finger_joint': 0.04,
            'r_gripper_finger_joint': 0.04,
            'torso_lift_joint': 0.2,
            'l_wheel_joint': 10.0,
            'r_wheel_joint': 10.0,
        }

        # Initialize motors and sensors
        self.motors = {}
        self.sensors = {}
        for name in self.joint_info:
            motor = self.robot.getDevice(name)
            if motor:
                if 'wheel' in name:
                    motor.setPosition(float('inf'))  # velocity control
                else:
                    motor.setPosition(0.0)  # position control
                motor.setVelocity(0.0)
                self.motors[name] = motor

            sensor = self.robot.getDevice(f"{name}_sensor")
            if sensor:
                sensor.enable(self.timestep)
                self.sensors[name] = sensor

        # Commanded positions for all joints
        self.commanded_positions = {name: 0.0 for name in self.joint_info}
        self.wrist_roll_angle = 0.0

        # Wheel properties for odometry
        self.wheel_radius = 0.065
        self.wheel_distance = 0.37476

    # ---------------- Base motion ----------------
    def move(self, linear_velocity, angular_velocity):
        """
        Move the robot base using differential drive.

        Arguments:
        - linear_velocity (float): forward speed in m/s.
        - angular_velocity (float): rotational speed around vertical axis in rad/s.

        Utility:
        - Calculates individual wheel velocities and commands motors.
        - Updates robot odometry (self.x, self.y, self.theta).
        """
        left_speed = (linear_velocity - (angular_velocity * self.wheel_distance / 2)) / self.wheel_radius
        right_speed = (linear_velocity + (angular_velocity * self.wheel_distance / 2)) / self.wheel_radius

        left_speed = max(-10, min(10, left_speed))
        right_speed = max(-10, min(10, right_speed))

        self.motors['l_wheel_joint'].setVelocity(left_speed)
        self.motors['r_wheel_joint'].setVelocity(right_speed)

        # Update odometry
        dt = self.timestep / 1000.0
        v = linear_velocity
        w = angular_velocity
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

    # ---------------- Arm motion ----------------
    def go_to(self, joint_positions):
        """
        Move specified arm joints to target positions.

        Arguments:
        - joint_positions (dict): {joint_name: target_position_in_radians_or_meters}

        Utility:
        - Updates commanded_positions and sets motor positions/velocities.
        - Can move multiple joints simultaneously.
        """
        for name, pos in joint_positions.items():
            if name in self.commanded_positions:
                self.commanded_positions[name] = pos
                if name in self.motors:
                    self.motors[name].setPosition(pos)
                    self.motors[name].setVelocity(self.joint_info.get(name, 5.0))

    def open_gripper(self):
        """
        Open the robot's gripper.

        Utility:
        - Moves left and right finger joints to fully open positions.
        """
        self.go_to({'l_gripper_finger_joint': 0.04, 'r_gripper_finger_joint': 0.04})

    def close_gripper(self):
        """
        Close the robot's gripper.

        Utility:
        - Moves left and right finger joints to fully closed positions.
        """
        self.go_to({'l_gripper_finger_joint': 0.0, 'r_gripper_finger_joint': 0.0})

    def twist_wrist(self, direction="clockwise"):
        """
        Rotate the wrist_roll_joint by 90° per call.

        Arguments:
        - direction (str): "clockwise" or "counterclockwise"

        Utility:
        - Updates self.wrist_roll_angle and commands wrist_roll_joint.
        - Keeps angle normalized within [-pi, pi].
        """
        step_angle = math.pi / 2
        if direction == "clockwise":
            self.wrist_roll_angle -= step_angle
        elif direction == "counterclockwise":
            self.wrist_roll_angle += step_angle
        else:
            print("Invalid direction")
            return
        self.wrist_roll_angle = (self.wrist_roll_angle + math.pi) % (2*math.pi) - math.pi
        self.motors['wrist_roll_joint'].setPosition(self.wrist_roll_angle)
        self.motors['wrist_roll_joint'].setVelocity(self.joint_info['wrist_roll_joint'])
        for _ in range(30):
            self.step()

    # ---------------- Pick & Place ----------------
    def pick_up(self, approach_pose, lower_offset=0.1):
        """
        Move arm to approach_pose, lower slightly, and grasp an object.

        Arguments:
        - approach_pose (dict): joint positions to reach the object
        - lower_offset (float): how much to lower the shoulder_lift_joint before closing gripper

        Utility:
        - Positions arm, opens gripper, lowers arm, then closes gripper.
        """
        for name, pos in approach_pose.items():
            if name in self.motors:
                self.motors[name].setPosition(pos)
                self.motors[name].setVelocity(self.joint_info.get(name, 1.0))
        self.open_gripper()
        if 'shoulder_lift_joint' in approach_pose:
            self.motors['shoulder_lift_joint'].setPosition(approach_pose['shoulder_lift_joint'] + lower_offset)
        self.close_gripper()

    def put_down(self, release_pose, lower_offset=0.1):
        """
        Move arm to release_pose, lower slightly, and release object.

        Arguments:
        - release_pose (dict): joint positions to release the object
        - lower_offset (float): how much to lower shoulder_lift_joint before opening gripper

        Utility:
        - Positions arm, lowers it slightly, then opens gripper to release object.
        """
        for name, pos in release_pose.items():
            if name in self.motors:
                self.motors[name].setPosition(pos)
                self.motors[name].setVelocity(self.joint_info.get(name, 1.0))
        if 'shoulder_lift_joint' in release_pose:
            self.motors['shoulder_lift_joint'].setPosition(release_pose['shoulder_lift_joint'] + lower_offset)
        self.open_gripper()

    # ---------------- Drawer & Closet ----------------
    def open_drawer(self, handle_position):
        """
        Open a drawer by moving to handle, grasping, and pulling outward.

        Arguments:
        - handle_position (dict): joint positions to reach the drawer handle
        - pull_distance (float): how far to pull the drawer

        Utility:
        - Commands arm to handle, closes gripper, pulls drawer, and releases handle.
        """
        self.go_to(handle_position)
        for _ in range(40): self.step()
        self.close_gripper()
        for _ in range(20): self.step()
        if 'shoulder_lift_joint' in self.motors:
            self.motors['shoulder_lift_joint'].setPosition(self.commanded_positions['shoulder_lift_joint'] + 0.1)
        if 'elbow_flex_joint' in self.motors:
            self.motors['elbow_flex_joint'].setPosition(self.commanded_positions['elbow_flex_joint'] - 0.3)
        for _ in range(60): self.step()
        self.open_gripper()

    def close_drawer(self, handle_position):
        """
        Close a drawer by moving to handle, grasping, and pushing inward.

        Arguments:
        - handle_position (dict): joint positions to reach the drawer handle

        Utility:
        - Commands arm to handle, closes gripper, pushes drawer closed, and releases handle.
        """
        self.go_to(handle_position)
        for _ in range(40): self.step()
        self.close_gripper()
        for _ in range(20): self.step()
        if 'shoulder_lift_joint' in self.motors:
            self.motors['shoulder_lift_joint'].setPosition(self.commanded_positions['shoulder_lift_joint'] - 0.1)
        if 'elbow_flex_joint' in self.motors:
            self.motors['elbow_flex_joint'].setPosition(self.commanded_positions['elbow_flex_joint'] + 0.3)
        for _ in range(60): self.step()
        self.open_gripper()

    def open_closet(self, handle_position):
        """
        Open a closet door by reaching the handle, twisting wrist, and pushing arm in an arc.

        Arguments:
        - handle_position (dict): joint positions to reach the closet handle

        Utility:
        - Simulates door opening motion: approach, grasp, wrist twist, arm arc, release.
        """
        self.go_to(handle_position)
        for _ in range(40): self.step()
        self.close_gripper()
        for _ in range(20): self.step()
        self.twist_wrist("clockwise")
        for _ in range(20): self.step()
        if 'shoulder_pan_joint' in self.motors:
            self.motors['shoulder_pan_joint'].setPosition(self.commanded_positions['shoulder_pan_joint'] - 0.5)
        if 'elbow_flex_joint' in self.motors:
            self.motors['elbow_flex_joint'].setPosition(self.commanded_positions['elbow_flex_joint'] - 0.2)
        for _ in range(60): self.step()
        self.open_gripper()

    def close_closet(self, handle_position):
        """
        Close a closet door by reversing the opening motion.

        Arguments:
        - handle_position (dict): joint positions to reach the closet handle

        Utility:
        - Simulates door closing motion: approach, grasp, arm arc, release.
        """
        self.go_to(handle_position)
        for _ in range(40): self.step()
        self.close_gripper()
        for _ in range(20): self.step()
        if 'shoulder_pan_joint' in self.motors:
            self.motors['shoulder_pan_joint'].setPosition(self.commanded_positions['shoulder_pan_joint'] + 0.5)
        if 'elbow_flex_joint' in self.motors:
            self.motors['elbow_flex_joint'].setPosition(self.commanded_positions['elbow_flex_joint'] + 0.2)
        for _ in range(60): self.step()
        self.open_gripper()

    # ---------------- Move to arbitrary location ----------------
    def go_to_location(self, x, y, approach_pose, linear_speed=0.1, angular_speed=0.5):
        """
        Move the robot base to a target (x, y) location, then move arm to approach_pose.

        Arguments:
        - x, y (float): target coordinates in meters
        - approach_pose (dict): joint positions to move arm after arriving
        - linear_speed (float): maximum forward speed (m/s)
        - angular_speed (float): maximum rotational speed (rad/s)

        Utility:
        - Rotates robot toward target, drives straight to it, then moves arm.
        - Updates robot odometry during movement.
        """
        print(f"Moving to location ({x:.2f}, {y:.2f})")

        # Step 1: Rotate toward target
        dx = x - self.x
        dy = y - self.y
        target_theta = math.atan2(dy, dx)
        while abs(target_theta - self.theta) > 0.01:
            angle_error = target_theta - self.theta
            # Normalize to [-pi, pi]
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            self.move(0, angular_speed if angle_error > 0 else -angular_speed)
            self.step()
        self.move(0, 0)
        print(f"  Oriented toward target: θ = {math.degrees(self.theta):.1f}°")

        # Step 2: Move straight toward target
        distance = math.hypot(dx, dy)
        while distance > 0.01:
            self.move(min(linear_speed, distance), 0)
            self.step()
            # Update remaining distance
            dx = x - self.x
            dy = y - self.y
            distance = math.hypot(dx, dy)
        self.move(0, 0)
        print(f"  Arrived at target location: ({self.x:.2f}, {self.y:.2f})")

        # Step 3: Move arm to approach pose
        self.go_to(approach_pose)
        for _ in range(40):
            self.step()
        print("  Arm moved to approach pose")

    # ---------------- Drawer / Closet high-level ----------------
    def interact_with_drawer(self, x, y, handle_pose, action="open"):
        """
        Navigate to a drawer and either open or close it.

        Arguments:
        - x, y (float): target coordinates of drawer
        - handle_pose (dict): joint positions to reach drawer handle
        - action (str): "open" or "close"

        Utility:
        - Combines navigation (go_to_location) and drawer manipulation.
        """
        self.go_to_location(x, y, handle_pose)
        if action == "open": self.open_drawer(handle_pose)
        elif action == "close": self.close_drawer(handle_pose)

    def interact_with_closet(self, x, y, handle_pose, action="open"):
        """
        Navigate to a closet and either open or close it.

        Arguments:
        - x, y (float): target coordinates of closet
        - handle_pose (dict): joint positions to reach closet handle
        - action (str): "open" or "close"

        Utility:
        - Combines navigation (go_to_location) and closet manipulation.
        """
        self.go_to_location(x, y, handle_pose)
        if action == "open": self.open_closet(handle_pose)
        elif action == "close": self.close_closet(handle_pose)

    def go_to_table(self, table_id):
        """
        Move the robot to a specific table.

        table_id: str, either "table1" or "table2"

        Uses the stored position and rotation of the target table, moves the base there,
        and sets the arm to a default neutral approach pose.
        """
        # Define tables
        tables = {
            "table1": {
                "position": (-7.06, -6.29, 0),
                "rotation": (0, 0, 1, 3.14149)
            },
            "table2": {
                "position": (-4.56, -5.53, 0),
                "rotation": (0, 0, -1, 1.5708)
            }
        }

        if table_id not in tables:
            print(f"Unknown table: {table_id}")
            return

        table = tables[table_id]
        x, y, _ = table["position"]

        # Default approach pose for the arm (can adjust later per table if needed)
        approach_pose = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': 0.3,
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 0.5,
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': 0.0,
            'wrist_roll_joint': 0.0
        }

        print(f"Moving to {table_id} at ({x:.2f}, {y:.2f})")
        self.go_to_location(x, y, approach_pose)
        print(f"Arrived at {table_id}")

    # ---------------- Helper ----------------
    def step(self):
        """
        Perform a single simulation step.

        Utility:
        - Advances Webots simulation by self.timestep milliseconds.
        - Returns -1 when simulation ends.
        """
        return self.robot.step(self.timestep)

    def test_all(self):
        """
        Unified test function to exercise all major robot capabilities:
        - Base motion
        - Arm motion
        - Gripper open/close
        - Wrist twisting
        - Pick & place
        - Drawer & closet interaction
        """
        print("=== Starting Full Robot Test ===")

        # ---------------- Base & Arm Motion ----------------
        print("Step 1: Moving to first test location (drawer)...")
        drawer_pose = {
            'shoulder_pan_joint': 0.4,
            'shoulder_lift_joint': 0.5,
            'elbow_flex_joint': 1.2,
            'wrist_flex_joint': -0.4,
            'wrist_roll_joint': 0.0
        }
        self.go_to_location(1.5, 2.0, drawer_pose)

        print("Step 2: Interacting with drawer")
        self.open_drawer(drawer_pose)
        self.close_drawer(drawer_pose)

        # ---------------- Closet Interaction ----------------
        print("Step 3: Moving to closet location...")
        closet_pose = {
            'shoulder_pan_joint': -0.3,
            'shoulder_lift_joint': 0.4,
            'elbow_flex_joint': 1.0,
            'wrist_flex_joint': -0.5,
            'wrist_roll_joint': 0.0
        }
        self.go_to_location(2.0, 3.0, closet_pose)

        print("Step 4: Interacting with closet")
        self.open_closet(closet_pose)
        self.close_closet(closet_pose)

        # ---------------- Pick & Place ----------------
        print("Step 5: Testing pick and place...")
        approach_pose = {
            'shoulder_pan_joint': 0.3,
            'shoulder_lift_joint': 0.5,
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 1.0,
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': -0.5,
            'wrist_roll_joint': 0.0
        }
        release_pose = {
            'shoulder_pan_joint': 0.3,
            'shoulder_lift_joint': 0.4,
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 1.2,
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': -0.6,
            'wrist_roll_joint': 0.0
        }

        # Pick up
        self.pick_up(approach_pose, lower_offset=0.15)
        for _ in range(30): self.step()
        print("  Picked up object")

        # Twist wrist
        for direction in ["clockwise", "clockwise", "counterclockwise", "counterclockwise"]:
            self.twist_wrist(direction)
            time.sleep(0.5)
        print("  Wrist twist test complete")

        # Place down
        self.put_down(release_pose, lower_offset=0.15)
        for _ in range(30): self.step()
        print("  Placed object down")

        # ---------------- Return Arm to Rest ----------------
        print("Step 6: Returning arm to neutral pose...")
        rest_pose = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': 0.2,
            'elbow_flex_joint': 0.3,
            'wrist_flex_joint': 0.0
        }
        self.go_to(rest_pose)
        for _ in range(30): self.step()

        print("=== Full Robot Test Complete ===")


def main():
    controller = FetchController()
    time.sleep(1.0)

    controller.go_to_table("table1")
    controller.go_to_table("table2")


    # Run unified test
    controller.test_all()

    while controller.step() != -1:
        pass



if __name__ == "__main__":
    main()
