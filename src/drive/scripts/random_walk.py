import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state_time = time.time()
        self.state = 'forward'
        self.duration = 3.0
        self.get_logger().info('Starting random walk exploration!')

    def timer_callback(self):
        msg = Twist()
        now = time.time()

        if now - self.state_time > self.duration:
            self.state_time = now
            if self.state == 'forward':
                choices = ['turn_left', 'turn_right', 'backup']
                self.state = random.choices(choices, weights=[0.4, 0.4, 0.2])[0]
                self.duration = random.uniform(1.0, 3.0) if self.state != 'backup' else random.uniform(1.5, 2.5)
            else:
                self.state = 'forward'
                self.duration = random.uniform(3.0, 6.0)
            self.get_logger().info(f'State: {self.state} for {self.duration:.1f}s')

        if self.state == 'forward':
            msg.linear.x = 0.5
            msg.angular.z = random.uniform(-0.1, 0.1)
        elif self.state == 'turn_left':
            msg.angular.z = 0.6
        elif self.state == 'turn_right':
            msg.angular.z = -0.6
        elif self.state == 'backup':
            msg.linear.x = -0.4
            msg.angular.z = random.uniform(-0.2, 0.2)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rw = RandomWalk()
    try:
        rclpy.spin(rw)
    except KeyboardInterrupt:
        pass
    rw.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
