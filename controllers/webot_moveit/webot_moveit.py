# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
#
# # FIX: We are importing the standard Webots API, which IS available internally.
# # This assumes you have access to the 'controller' module when running the script.
# try:
#     from controller import Robot, Motor, PositionSensor
# except ImportError as e:
#     # Fallback/Error for debugging, though this should fix the immediate issue
#     print(f"Webots 'controller' module not found: {e}")
#     exit(1)
#
#
# class FetchWebotsDriver(Node):
#     """
#     Acts as a bridge: uses the native Webots API to control the hardware,
#     but communicates via ROS 2 messages using rclpy.
#     """
#
#     def __init__(self, robot, timestep):
#         super().__init__('fetch_ros_bridge_driver')
#
#         # 1. Store Webots Robot Instance
#         self.robot = robot
#         self.timestep = timestep
#         self.get_logger().info(f"Fetch ROS 2 Bridge Driver initialized. Time step: {self.timestep}ms")
#
#         # 2. Define controlled joints (Check your proto file for accuracy!)
#         # UPDATED: Velocities set to maximum values derived from the .proto file
#         # to eliminate "requested velocity exceeds maxVelocity" warnings.
#         self.joint_info = {
#             # Arm Joints (Rotational Motors) - Max velocity in rad/s
#             'shoulder_pan_joint': 1.256,
#             'shoulder_lift_joint': 1.454,
#             'upperarm_roll_joint': 1.571,
#             'elbow_flex_joint': 1.521,
#             'forearm_roll_joint': 1.571,
#             'wrist_flex_joint': 2.268,
#             'wrist_roll_joint': 2.268,
#             # Gripper Joints (Linear Motors - Max velocity in m/s)
#             'l_gripper_finger_joint': 0.05,
#             'r_gripper_finger_joint': 0.05,
#             # Torso Lift Joint (Linear Motor - Max velocity in m/s)
#             'torso_lift_joint': 0.1,
#             # NOTE: We skip wheels here, as MoveIt typically uses a DiffDriveController which is handled separately
#         }
#
#         # 3. Initialize Motors and Position Sensors
#         self.motors = {}
#         self.position_sensors = {}
#         self.all_joint_names = list(self.joint_info.keys())
#
#         for name in self.all_joint_names:
#             # Get Motor
#             motor = self.robot.getDevice(name)
#             if motor:
#                 motor.setPosition(0.0)
#                 # Setting the initial motor velocity to 0.0, the max velocity will be set in drive_motors()
#                 motor.setVelocity(0.0)
#                 self.motors[name] = motor
#             else:
#                 self.get_logger().warn(f"Motor '{name}' not found.")
#
#             # Get Position Sensor
#             sensor = self.robot.getDevice(f"{name}_sensor")
#             if sensor:
#                 sensor.enable(self.timestep)
#                 self.position_sensors[name] = sensor
#             else:
#                 self.get_logger().warn(f"Position Sensor '{name}_sensor' not found.")
#
#         # 4. State Management and ROS I/O
#         self.commanded_positions = {name: 0.0 for name in self.all_joint_names}
#
#         # Subscriber for joint position commands (from the MoveIt Client)
#         self.create_subscription(
#             JointState,
#             '/fetch/joint_commands',  # Topic where the MoveIt client will send goals
#             self.joint_command_callback,
#             10
#         )
#
#         # Publisher for joint states (for RViz visualization and MoveIt planning)
#         self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
#
#     def joint_command_callback(self, msg):
#         """Processes incoming JointState messages carrying the goal positions."""
#         for name, position in zip(msg.name, msg.position):
#             if name in self.motors:
#                 self.commanded_positions[name] = position
#
#     def publish_joint_states(self):
#         """Reads current sensor values and publishes the JointState message."""
#         js_msg = JointState()
#         js_msg.header.stamp = self.get_clock().now().to_msg()
#
#         for name in self.all_joint_names:
#             js_msg.name.append(name)
#
#             # Position sensor reading
#             pos = self.position_sensors[name].getValue() if name in self.position_sensors else 0.0
#             js_msg.position.append(pos)
#
#             # Velocity (Placeholder)
#             js_msg.velocity.append(0.0)
#
#         self.joint_state_publisher.publish(js_msg)
#
#     def drive_motors(self):
#         """Applies commanded positions and corresponding velocities to the Webots motors."""
#         for name, motor in self.motors.items():
#             target_pos = self.commanded_positions[name]
#             # Use the max allowed velocity from the updated dictionary
#             max_vel = self.joint_info.get(name, 0.0)
#
#             motor.setPosition(target_pos)
#             # Setting velocity ensures the motor follows the trajectory as quickly as possible
#             # while respecting the Webots .proto limits.
#             motor.setVelocity(max_vel)
#
#
# # ----------------------------------------------------
# # Main Webots Loop
# # ----------------------------------------------------
# def main():
#     # 1. Initialize Webots Robot
#     webots_robot = Robot()
#     timestep = int(webots_robot.getBasicTimeStep())
#
#     # 2. Initialize ROS 2 system
#     rclpy.init(args=None)
#
#     # 3. Create the ROS Node, passing the Webots robot instance
#     # We use MultiThreadedExecutor to allow the Webots loop and ROS subscriptions to run in parallel.
#     driver_node = FetchWebotsDriver(webots_robot, timestep)
#
#     # 4. Use a MultiThreadedExecutor to handle ROS subscriptions and the Webots loop
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(driver_node)
#
#     # 5. Main Control Loop
#     while webots_robot.step(timestep) != -1:
#         # Run ROS callbacks (subscriptions)
#         executor.spin_once(timeout_sec=0)
#
#         # Drive motors to the latest commanded positions
#         driver_node.drive_motors()
#
#         # Publish current state for MoveIt/RViz
#         driver_node.publish_joint_states()
#
#     # Cleanup
#     driver_node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# Import the standard threading module
import threading
from rclpy.executors import MultiThreadedExecutor  # Keep for executor functionality
import torch  # **NEW: Required for PyTorch/CUDA check**

# FIX: We are importing the standard Webots API, which IS available internally.
# This assumes you have access to the 'controller' module when running the script.
try:
    from controller import Robot, Motor, PositionSensor
except ImportError as e:
    # Fallback/Error for debugging, though this should fix the immediate issue
    print(f"Webots 'controller' module not found: {e}")
    exit(1)

# Configuration for stabilizing the simulation
# Run control logic (publishing states, driving motors) every N physics steps.
# A value of 4 means control runs every 32 * 4 = 128ms.
CONTROL_FREQUENCY_MULTIPLIER = 4


class FetchWebotsDriver(Node):
    """
    Acts as a bridge: uses the native Webots API to control the hardware,
    but communicates via ROS 2 messages using rclpy.
    """

    def __init__(self, robot, timestep):
        super().__init__('fetch_ros_bridge_driver')

        # 1. Store Webots Robot Instance
        self.robot = robot
        self.timestep = timestep
        self.get_logger().info(f"Fetch ROS 2 Bridge Driver initialized. Time step: {self.timestep}ms")

        # Control management
        self.control_step_counter = 0

        # 2. Define controlled joints (Check your proto file for accuracy!)
        # UPDATED: Velocities set to maximum values derived from the .proto file
        # to eliminate "requested velocity exceeds maxVelocity" warnings.
        self.joint_info = {
            # Arm Joints (Rotational Motors) - Max velocity in rad/s
            'shoulder_pan_joint': 1.256,
            'shoulder_lift_joint': 1.454,
            'upperarm_roll_joint': 1.571,
            'elbow_flex_joint': 1.521,
            'forearm_roll_joint': 1.571,
            'wrist_flex_joint': 2.268,
            'wrist_roll_joint': 2.268,
            # Gripper Joints (Linear Motors - Max velocity in m/s)
            'l_gripper_finger_joint': 0.05,
            'r_gripper_finger_joint': 0.05,
            # Torso Lift Joint (Linear Motor - Max velocity in m/s)
            'torso_lift_joint': 0.1,
            # NOTE: We skip wheels here, as MoveIt typically uses a DiffDriveController which is handled separately
        }

        # 3. Initialize Motors and Position Sensors
        self.motors = {}
        self.position_sensors = {}
        self.all_joint_names = list(self.joint_info.keys())

        for name in self.all_joint_names:
            # Get Motor
            motor = self.robot.getDevice(name)
            if motor:
                # Set motor to zero velocity initially
                motor.setPosition(0.0)
                motor.setVelocity(0.0)
                self.motors[name] = motor
            else:
                self.get_logger().warn(f"Motor '{name}' not found.")

            # Get Position Sensor
            sensor = self.robot.getDevice(f"{name}_sensor")
            if sensor:
                sensor.enable(self.timestep)
                self.position_sensors[name] = sensor
            else:
                self.get_logger().warn(f"Position Sensor '{name}_sensor' not found.")

        # 4. State Management and ROS I/O
        self.commanded_positions = {name: 0.0 for name in self.all_joint_names}

        # Subscriber for joint position commands (from the MoveIt Client)
        self.create_subscription(
            JointState,
            '/fetch/joint_commands',  # Topic where the MoveIt client will send goals
            self.joint_command_callback,
            10
        )

        # Publisher for joint states (for RViz visualization and MoveIt planning)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

    def joint_command_callback(self, msg):
        """Processes incoming JointState messages carrying the goal positions."""
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

            # Velocity (Placeholder)
            js_msg.velocity.append(0.0)

        self.joint_state_publisher.publish(js_msg)

    def drive_motors(self):
        """Applies commanded positions and corresponding velocities to the Webots motors."""
        for name, motor in self.motors.items():
            target_pos = self.commanded_positions[name]
            # Use the max allowed velocity from the updated dictionary
            max_vel = self.joint_info.get(name, 0.0)

            motor.setPosition(target_pos)
            # Setting velocity ensures the motor follows the trajectory as quickly as possible
            # while respecting the Webots .proto limits.
            motor.setVelocity(max_vel)


# ----------------------------------------------------
# ROS 2 Thread Spin Function
# ----------------------------------------------------
def ros_spin(executor, driver_node):
    """Function to run the executor in a separate thread."""
    try:
        executor.spin()
    finally:
        executor.shutdown()
        driver_node.destroy_node()


# ----------------------------------------------------
# Main Webots Loop
# ----------------------------------------------------
def main():
    # 0. CUDA/GPU Setup **NEW**
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] 🚀 Using PyTorch Device: {DEVICE}")

    # 1. Initialize Webots Robot
    webots_robot = Robot()
    timestep = int(webots_robot.getBasicTimeStep())

    # 2. Initialize ROS 2 system
    rclpy.init(args=None)

    # 3. Create the ROS Node, passing the Webots robot instance
    driver_node = FetchWebotsDriver(webots_robot, timestep)

    # 4. Setup MultiThreadedExecutor and start the spin in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)

    # Start ROS 2 spinning in the background
    spin_thread = threading.Thread(target=ros_spin, args=(executor, driver_node))
    spin_thread.start()

    # 5. Main Control Loop (synchronous to Webots time step)
    while webots_robot.step(timestep) != -1:
        # Increment step counter
        driver_node.control_step_counter += 1

        # Decouple Control: Run drive and publish only when the counter hits the multiplier
        if driver_node.control_step_counter % CONTROL_FREQUENCY_MULTIPLIER == 0:
            # Drive motors to the latest commanded positions
            driver_node.drive_motors()

            # Publish current state for MoveIt/RViz
            driver_node.publish_joint_states()

        # Check if the ROS thread is still active (optional safety)
        if not spin_thread.is_alive():
            break

    # 6. Cleanup (only executes if the Webots loop breaks)
    spin_thread.join()  # Wait for the ROS thread to finish
    rclpy.shutdown()


if __name__ == '__main__':
    main()
