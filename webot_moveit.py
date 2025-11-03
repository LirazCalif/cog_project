import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from webots_ros2_driver.webots_driver import WebotsDriver


# IMPORTANT: This script runs inside the Webots simulation as the robot's controller.
# It uses the webots_ros2_driver and requires ROS 2 to be sourced externally.

class FetchWebotsDriver(Node):
    """
    Acts as the ROS 2 hardware interface for the Fetch robot in Webots.
    It receives joint position commands (from MoveIt/ROS Control) and applies them
    to the Webots motors, while continuously publishing the current joint states.
    """

    def __init__(self):
        super().__init__('fetch_webots_driver')

        # 1. Initialize Webots Robot
        self.robot = WebotsDriver()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.get_logger().info(f"Fetch Webots ROS Driver initialized. Time step: {self.timestep}ms")

        # 2. Define controlled joints (Including wheels and torso from your .proto)
        self.joint_info = {
            # Arm Joints (Rotational Motors)
            'shoulder_pan_joint': 5.0,
            'shoulder_lift_joint': 5.0,
            'upperarm_roll_joint': 5.0,
            'elbow_flex_joint': 5.0,
            'forearm_roll_joint': 5.0,
            'wrist_flex_joint': 5.0,
            'wrist_roll_joint': 5.0,
            # Gripper Joints (Linear Motors - Position in meters)
            'l_gripper_finger_joint': 0.5,  # High velocity for fast opening
            'r_gripper_finger_joint': 0.5,
            # Torso Lift Joint (Linear Motor)
            'torso_lift_joint': 0.2,  # Position in meters, slower velocity
        }

        # 3. Initialize Motors and Position Sensors
        self.motors = {}
        self.position_sensors = {}
        self.all_joint_names = list(self.joint_info.keys())

        for name in self.all_joint_names:
            # Get Motor
            motor = self.robot.getMotor(name)
            if motor:
                motor.setPosition(0.0)
                motor.setVelocity(0.0)
                self.motors[name] = motor
            else:
                self.get_logger().warn(f"Motor '{name}' not found. Check .proto file.")

            # Get Position Sensor
            sensor = self.robot.getPositionSensor(f"{name}_sensor")
            if sensor:
                sensor.enable(self.timestep)
                self.position_sensors[name] = sensor
            else:
                self.get_logger().warn(f"Position Sensor '{name}_sensor' not found.")

        # 4. State Management and ROS I/O
        self.commanded_positions = {name: 0.0 for name in self.all_joint_names}

        # Subscriber for joint position commands (from the MoveIt Client)
        self.subscription = self.create_subscription(
            JointState,
            '/fetch/joint_commands',  # Topic where the MoveIt client will send goals
            self.joint_command_callback,
            10
        )
        self.get_logger().info("Subscribed to /fetch/joint_commands for execution.")

        # Publisher for joint states (for RViz visualization and MoveIt planning)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # 5. Create Webots loop timer
        self.timer = self.create_timer(self.timestep / 1000.0, self.webots_step_loop)

    def joint_command_callback(self, msg):
        """Processes incoming JointState messages carrying the goal positions."""
        # Note: This simple implementation directly uses JointState for position command.
        for name, position in zip(msg.name, msg.position):
            if name in self.motors:
                self.commanded_positions[name] = position

    def publish_joint_states(self):
        """Reads current sensor values and publishes the JointState message."""
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()

        for name in self.all_joint_names:
            js_msg.name.append(name)

            # Position sensor reading
            pos = self.position_sensors[name].getValue() if name in self.position_sensors else 0.0
            js_msg.position.append(pos)

            # Velocity (Placeholder, actual vel would be calculated)
            js_msg.velocity.append(0.0)

        self.joint_state_publisher.publish(js_msg)

    def webots_step_loop(self):
        """
        The main control loop for Webots.
        1. Drive the motors to the commanded position.
        2. Publish the current state back to ROS.
        """
        if self.robot.step(self.timestep) != -1:
            # 1. Apply Commands to Motors
            for name, motor in self.motors.items():
                target_pos = self.commanded_positions[name]
                max_vel = self.joint_info.get(name, 5.0)

                motor.setPosition(target_pos)
                motor.setVelocity(max_vel)

            # 2. Publish Current State
            self.publish_joint_states()


def main(args=None):
    rclpy.init(args=args)
    driver = FetchWebotsDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
