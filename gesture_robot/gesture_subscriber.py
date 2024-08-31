#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class gestureSubscriber(Node):
    def __init__(self):
        super().__init__("Gesture_Subscriber")
        self.subscription = self.create_subscription(String, "/gesture/handsign",self.listener_callback, 10)
        self.publisher = self.create_publisher(String, "/move_manipulator",10)

    def listener_callback(self, msg):
        incoming_msg = msg.data

        if incoming_msg == "Go":
            self.get_logger().info(f'Received gesture: "{incoming_msg}"')
            self.publisher.publish(String(data="open_gripper"))
        elif incoming_msg == "Turn Left":
            self.get_logger().info(f'Received gesture: "{incoming_msg}"')
            self.publisher.publish(String(data="pick_object"))
        elif incoming_msg == "Turn Right":
            self.get_logger().info(f'Received gesture: "{incoming_msg}"')
            self.publisher.publish(String(data="turn_right"))
        elif incoming_msg == "Forward":
            self.get_logger().info(f'Received gesture: "{incoming_msg}"')
            self.publisher.publish(String(data="move_forward"))
        


def main(args=None):
    rclpy.init(args=args)
    camera_gesture_sub = gestureSubscriber()
    rclpy.spin(camera_gesture_sub)
    camera_gesture_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()