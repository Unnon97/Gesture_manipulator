#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class gestureSubscriber(Node):
    def __init__(self):
        super.__init__("Gesture Subscriber")
        self.subscription = self.create_subscription(String, "/gesture",self.listener_callback, 10)
        self.publisher = self.create_publisher(String, "/move_manipulator",10)

    def listener_callback(self, msg):
        incoming_msg = msg.data
        self.get_logger().info(f'Received gesture: "{incoming_msg}"')

        if incoming_msg == "go":
            self.publisher.publish(String(data="run_stage1"))
        else:
            self.get_logger().info("Unknown command received")


def main(args=None):
    rclpy.init(args=args)
    camera_gesture_sub = gestureSubscriber()
    rclpy.spin(camera_gesture_sub)
    camera_gesture_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()