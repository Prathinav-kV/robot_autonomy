import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Base class for walker states
class WalkerState:
    def handle(self, walker):
        raise NotImplementedError  # Forces subclasses to implement this method

# MovingState (robot moves forward)
class MovingState(WalkerState):
    def handle(self, walker):
        if walker.obstacle_detected:
            walker.set_state(TurningState(walker.toggle_turn_direction()))
        else:
            walker.move(walker.linear_velocity, 0.0)  # Use parameterized speed

# TurningState (robot turns)
class TurningState(WalkerState):
    def __init__(self, turn_clockwise):
        self.turn_clockwise = turn_clockwise

    def handle(self, walker):
        if not walker.obstacle_detected:
            walker.set_state(MovingState())  # Switch back to moving
        else:
            walker.move(0.0, -0.5 if self.turn_clockwise else 0.5)  # Turn
            walker.get_logger().info(f"🔄 Turning {'Clockwise' if self.turn_clockwise else 'Anti-Clockwise'}")

# WalkerClass managing the state machine
class WalkerClass(Node):
    def __init__(self):
        super().__init__('walker')

        # Declare parameter for linear velocity (default = 0.2 m/s)
        self.declare_parameter('linear_velocity', 0.2)
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # Subscribe to LIDAR topic
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for movement commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initial state
        self.current_state = MovingState()
        self.obstacle_detected = False
        self.turn_clockwise = True

        self.get_logger().info(f"🚀 Walker Bot Started! Initial Speed: {self.linear_velocity:.2f} m/s")

    def set_state(self, state):
        """Set the current state of the walker."""
        self.current_state = state

    def run(self):
        """Execute the logic of the current state."""
        if self.current_state:
            self.current_state.handle(self)

    def move(self, linear, angular):
        """Publish velocity commands."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def toggle_turn_direction(self):
        """Toggle between clockwise and counterclockwise turns."""
        self.turn_clockwise = not self.turn_clockwise
        return self.turn_clockwise

    def scan_callback(self, msg):
        """Process LIDAR data and detect obstacles."""
        self.obstacle_detected = any(
            msg.ranges[i] < 0.8 or msg.ranges[-(i + 1)] < 0.8
            for i in range(30)
        )

        if self.obstacle_detected:
            self.get_logger().info("🛑 Obstacle Encountered!")

        # Run the current state logic
        self.run()

# Main function
def main(args=None):
    rclpy.init(args=args)
    walker = WalkerClass()
    rclpy.spin(walker)
    walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
