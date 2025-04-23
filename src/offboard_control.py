#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)



        # Set Waypoints
        # --------------------------------------------------------------------------------------------------------------------
        self.waypoints = [[1000.0, 10000.0, -30.0]]
        # --------------------------------------------------------------------------------------------------------------------
        
        # Initialize variables
        self.waypoint = self.waypoints[0]
        self.offboard_setpoint_counter = 0
        
        self.waypoint_change_counter = 0
        self.delay_at_waypoint_counter = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # Extract and print position data
        x = vehicle_local_position.x
        y = vehicle_local_position.y
        z = vehicle_local_position.z
        self.get_logger().info(f"Vehicle Position: x={x}, y={y}, z={z}")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish the trajectory setpoint."""
        
        msg = TrajectorySetpoint()
        msg.position = self.waypoint # Hover at 5m altitude
        msg.yaw = 0.0 # 0 degrees yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info("Publishing trajectory setpoint: Hover at 5m")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.change_waypoint()
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def change_waypoint(self):
        """Change the waypoint to a new position."""
        if self.check_pos():
            # don't increament the waypoint_change_counter if it exceeds the length of waypoints
            if self.waypoint_change_counter< len(self.waypoints)-1:
                self.waypoint_change_counter += 1
                self.waypoint = self.waypoints[self.waypoint_change_counter]
        
     

    def check_pos(self):
        # Putting a tolerance of 0.2m
        tolerence = 0.2

        # Check if the vehicle is within the tolerance of the waypoint
        if (abs(self.vehicle_local_position.x - self.waypoint[0]) < tolerence and
                abs(self.vehicle_local_position.y - self.waypoint[1]) < tolerence and
                abs(self.vehicle_local_position.z - self.waypoint[2]) < tolerence):
            
            # Setting delay at waypoints  
            if self.delay_at_waypoint_counter < 3:
                self.delay_at_waypoint_counter += 1

            # Returning the True and setting the delay counter to 0
            else:
                self.delay_at_waypoint_counter = 0
                return True
        
        # Returning False if the vehicle is not within the tolerance        
        else:
            return False
        

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
