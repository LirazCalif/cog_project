from controller import Robot, Motor, PositionSensor
import time

class FetchController:
    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        print(f"Fetch Controller initialized. Time step: {self.timestep} ms")

        # Define joints and their max velocities
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
            'r_wheel_joint': 10.0
        }

        # Initialize motors and sensors
        self.motors = {}
        self.sensors = {}
        for name in self.joint_info:
            motor = self.robot.getDevice(name)
            if motor:
                # Wheels -> velocity control; others -> position control
                if 'wheel' in name:
                    motor.setPosition(float('inf'))  # velocity control
                else:
                    motor.setPosition(0.0)  # position control
                motor.setVelocity(0.0)
                self.motors[name] = motor

            # Try to enable sensors if they exist
            sensor = self.robot.getDevice(f"{name}_sensor")
            if sensor:
                sensor.enable(self.timestep)
                self.sensors[name] = sensor

        # Commanded positions for all joints
        self.commanded_positions = {name: 0.0 for name in self.joint_info}

    def move(self, linear_velocity, angular_velocity):
        wheel_distance = 0.37476
        wheel_radius = 0.065

        left_speed = (linear_velocity - (angular_velocity * wheel_distance / 2)) / wheel_radius
        right_speed = (linear_velocity + (angular_velocity * wheel_distance / 2)) / wheel_radius

        # Clip to max motor velocity
        left_speed = max(-10, min(10, left_speed))
        right_speed = max(-10, min(10, right_speed))

        print(f"Wheel speeds -> Left: {left_speed:.2f}, Right: {right_speed:.2f}")

        self.motors['l_wheel_joint'].setVelocity(left_speed)
        self.motors['r_wheel_joint'].setVelocity(right_speed)

    def go_to(self, joint_positions):

        """
        Moves the Fetch robot's arm.
        """
        for name, pos in joint_positions.items():
            if name in self.commanded_positions:
                self.commanded_positions[name] = pos
                if name in self.motors:
                    self.motors[name].setPosition(pos)
                    self.motors[name].setVelocity(self.joint_info.get(name, 5.0))

    def open_gripper(self):
        self.go_to({'l_gripper_finger_joint': 0.04, 'r_gripper_finger_joint': 0.04})

    def close_gripper(self):
        self.go_to({'l_gripper_finger_joint': 0.0, 'r_gripper_finger_joint': 0.0})

    def pick_up(self, approach_pose, lower_offset=0.1):
        """
        Command the robot to pick up an object.
        """
        # Move arm to approach pose
        for name, pos in approach_pose.items():
            if name in self.motors:
                self.motors[name].setPosition(pos)
                self.motors[name].setVelocity(self.joint_info.get(name, 1.0))

        # Open gripper
        self.motors['l_gripper_finger_joint'].setPosition(0.04)
        self.motors['l_gripper_finger_joint'].setVelocity(self.joint_info['l_gripper_finger_joint'])
        self.motors['r_gripper_finger_joint'].setPosition(0.04)
        self.motors['r_gripper_finger_joint'].setVelocity(self.joint_info['r_gripper_finger_joint'])

        # Lower arm for grasp
        for name, pos in approach_pose.items():
            if name == 'shoulder_lift_joint':
                self.motors[name].setPosition(pos + lower_offset)

        # Close gripper (grasp)
        self.motors['l_gripper_finger_joint'].setPosition(0.0)
        self.motors['r_gripper_finger_joint'].setPosition(0.0)

    def put_down(self, release_pose, lower_offset=0.1):
        """
        Command the robot to put down an object.
        """
        # Move arm to release pose
        for name, pos in release_pose.items():
            if name in self.motors:
                self.motors[name].setPosition(pos)
                self.motors[name].setVelocity(self.joint_info.get(name, 1.0))

        # Lower arm for release
        for name, pos in release_pose.items():
            if name == 'shoulder_lift_joint':
                self.motors[name].setPosition(pos + lower_offset)

        # Open gripper to release object
        self.motors['l_gripper_finger_joint'].setPosition(0.04)
        self.motors['r_gripper_finger_joint'].setPosition(0.04)

    def step(self):
        return self.robot.step(self.timestep)

    def test_pick_and_place(self):
        print("=== Starting Pick and Place Test ===")

        # --- Step 1: Move near the bottle ---
        print("Step 1: Approaching bottle...")
        # Move forward in small increments for control
        for _ in range(200):  # roughly 2.5 meters
            self.move(0.2, 0.0)
            self.step()
        self.move(0, 0)
        print("  Arrived near bottle")

        # --- Step 2: Turn slightly left towards the bottle ---
        print("Step 2: Turning towards bottle...")
        for _ in range(10):  # rotate left
            self.move(0.0, 0.2)
            self.step()
        self.move(0, 0)
        print("  Facing bottle")

        # --- Step 3: Prepare arm to pick up bottle ---
        print("Step 3: Moving arm to approach pose...")
        approach_pose = {
            'shoulder_pan_joint': 0.3,  # Turn arm slightly left
            'shoulder_lift_joint': 0.5,  # Lift shoulder
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 1.0,  # Bend elbow
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': -0.5,  # Lower wrist towards bottle
            'wrist_roll_joint': 0.0
        }
        self.pick_up(approach_pose, lower_offset=0.15)
        for _ in range(30):
            self.step()
        time.sleep(1.0)
        print("  Bottle picked up")

        # --- Step 4: Lift the arm slightly ---
        print("Step 4: Lifting arm with bottle...")
        lift_pose = approach_pose.copy()
        lift_pose['shoulder_lift_joint'] = 0.2  # lift up
        self.go_to(lift_pose)
        for _ in range(30):
            self.step()
        time.sleep(1.0)
        print("  Arm lifted")

        # --- Step 5: Move to new location to put bottle down ---
        print("Step 5: Moving to placement location...")
        for _ in range(30):  # move forward 1.5m
            self.move(0.1, 0.0)
            self.step()
        self.move(0, 0)
        print("  Reached placement area")

        # --- Step 6: Place down bottle ---
        print("Step 6: Putting bottle down...")
        release_pose = {
            'shoulder_pan_joint': 0.3,
            'shoulder_lift_joint': 0.4,
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 1.2,
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': -0.6,
            'wrist_roll_joint': 0.0
        }
        self.put_down(release_pose, lower_offset=0.15)
        for _ in range(30):
            self.step()
        time.sleep(1.0)
        print("  Bottle placed down")

        # --- Step 7: Return to neutral pose ---
        print("Step 7: Returning arm to rest position...")
        self.go_to({
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': 0.2,
            'elbow_flex_joint': 0.3,
            'wrist_flex_joint': 0.0
        })
        for _ in range(30):
            self.step()
        print("  Arm returned to rest")
        print("=== Pick and Place Test Complete ===")


def main():
    controller = FetchController()
    time.sleep(1.0)
    controller.test_pick_and_place()

    while controller.step() != -1:
        pass



if __name__ == "__main__":
    main()
