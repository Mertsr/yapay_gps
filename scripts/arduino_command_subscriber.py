import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from libs.arduino_socket_client import MultiArduinoSocketClient

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.get_logger().info("Multi Arduino Cmd Vel Subscriber Node initialized.")
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Constants for velocity mapping
        self.MAX_LINEAR_VELOCITY = 70
        self.MAX_ANGULAR_VELOCITY = 180  # degrees
        
        # Initialize multi Arduino client
        self.arduino_client = MultiArduinoSocketClient()
        self.arduino_client.connect_all()
        
        # Publisher for Arduino commands (for debugging/monitoring)
        self.arduino_publisher = self.create_publisher(
            String,
            '/arduino/command',
            10
        )
        
        self.last_cmd_vel = Twist()
        
        # Create timer to check device status
        self.status_timer = self.create_timer(10.0, self.check_device_status)
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages and route to appropriate Arduino devices"""
        self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
        
        # Handle linear motion (BLDC motor)
        if abs(msg.linear.x) > 0.01:  # Small threshold to avoid noise
            # Map linear velocity to motor command
            speed = min(abs(msg.linear.x) * self.MAX_LINEAR_VELOCITY, self.MAX_LINEAR_VELOCITY)
            direction = "FORWARD" if msg.linear.x > 0 else "BACKWARD"
            
            motor_command = f"M,{direction},{int(speed)}"
            success = self.arduino_client.send_command(motor_command)
            
            if success:
                self.get_logger().info(f"Motor command sent: {motor_command}")
                # Also publish for monitoring
                self.arduino_publisher.publish(String(data=motor_command))
            else:
                self.get_logger().warn(f"Failed to send motor command: {motor_command}")
        else:
            # Stop motor if no linear velocity
            stop_command = "M,STOP,0"
            self.arduino_client.send_command(stop_command)
            self.arduino_publisher.publish(String(data=stop_command))
        
        # Handle angular motion (Stepper motor for steering)
        if abs(msg.angular.z) > 0.01:  # Small threshold to avoid noise
            # Map angular velocity to stepper angle
            # Positive angular.z = left turn, negative = right turn
            angle = msg.angular.z * self.MAX_ANGULAR_VELOCITY
            # Clamp angle to reasonable range
            angle = max(-180, min(180, angle))
            
            stepper_command = f"S,ANGLE,{int(angle)}"
            success = self.arduino_client.send_command(stepper_command)
            
            if success:
                self.get_logger().info(f"Stepper command sent: {stepper_command}")
                # Also publish for monitoring
                self.arduino_publisher.publish(String(data=stepper_command))
            else:
                self.get_logger().warn(f"Failed to send stepper command: {stepper_command}")
        
        self.last_cmd_vel = msg
    
    def check_device_status(self):
        """Periodically check and log device connection status"""
        status = self.arduino_client.get_device_status()
        connected_devices = [name for name, connected in status.items() if connected]
        disconnected_devices = [name for name, connected in status.items() if not connected]
        
        if connected_devices:
            self.get_logger().info(f"Connected Arduino devices: {', '.join(connected_devices)}")
        if disconnected_devices:
            self.get_logger().warn(f"Disconnected Arduino devices: {', '.join(disconnected_devices)}")

    def destroy_node(self):
        self.arduino_client.close()
        super().destroy_node()
        self.get_logger().info("Multi Arduino Cmd Vel Subscriber Node destroyed.")
def main():
    rclpy.init()
    node = CmdVelSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
        

