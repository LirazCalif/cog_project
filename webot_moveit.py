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
        """
        Moves the Fetch robot using the left and right wheel joints.
        """
        wheel_distance = 0.37476  # approximate distance between wheels
        left_speed = linear_velocity - (angular_velocity * wheel_distance / 2)
        right_speed = linear_velocity + (angular_velocity * wheel_distance / 2)

        if 'l_wheel_joint' in self.motors:
            self.motors['l_wheel_joint'].setVelocity(left_speed)
        if 'r_wheel_joint' in self.motors:
            self.motors['r_wheel_joint'].setVelocity(right_speed)

    def go_to(self, joint_positions):
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

    def test_sequence(self):
        print("=== Starting Test Sequence ===")

        # 1. Move backwards several meters
        print("Step 1: Moving backwards 3 meters...")
        total_distance = -0.5
        step_distance = -0.1
        steps = int(abs(total_distance / step_distance))
        for _ in range(steps):
            self.move(step_distance, 0.0)
            self.step()
        self.move(0, 0)
        print(f"  Moved {total_distance} m backwards")

        # 2. Move arm to initial pose
        print("Step 2: Moving arm to initial pose...")
        self.go_to({
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': 0.2,
            'upperarm_roll_joint': 0.0,
            'elbow_flex_joint': 0.3,
            'forearm_roll_joint': 0.0,
            'wrist_flex_joint': 0.0,
            'wrist_roll_joint': 0.0
        })
        self.step()
        time.sleep(1.0)
        print("  Arm moved to initial pose")

        # 3. Open gripper
        print("Step 3: Opening gripper...")
        self.open_gripper()
        self.step()
        time.sleep(1.0)
        print("  Gripper opened")

        # 4. Close gripper
        print("Step 4: Closing gripper...")
        self.close_gripper()
        self.step()
        time.sleep(1.0)
        print("  Gripper closed")

        # 5. Final arm pose
        print("Step 5: Moving arm to final pose...")
        self.go_to({
            'shoulder_pan_joint': 0.5,
            'shoulder_lift_joint': 0.1,
            'elbow_flex_joint': 0.5
        })
        self.step()
        time.sleep(1.0)
        print("  Arm moved to final pose")

        print("=== Test Sequence Complete ===")


def main():
    controller = FetchController()
    time.sleep(1.0)
    controller.test_sequence()

    # Keep robot running
    while controller.step() != -1:
        pass


if __name__ == "__main__":
    main()
