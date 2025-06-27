
import rclpy
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_control_msgs.msg import AckermannControlCommand

import os
import multiprocessing

def write_to_file(queue):
    filename = os.path.expanduser('~/autoware_node/autoware_log/ros2_data.txt')
    while True:
        data = queue.get()
        if data is None:
            break
        with open(filename, 'a') as file:
            file.write(data+ '\n')


class HazardLightsListener(Node):
    def __init__(self, queue):
        super().__init__('hazard_lights_listener')

        # 'hazard_lights_command' 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i=0
        self.queue = queue

    def listener_callback(self, msg):
        self.queue.put(str(msg.longitudinal_velocity))

def main(args=None):

    write_queue = multiprocessing.Queue()
    writer_process = multiprocessing.Process(target=write_to_file, args=(write_queue,))
    writer_process.start()


    rclpy.init(args=args)
    node = HazardLightsListener(write_queue)
    rclpy.spin(node)

    # 노드를 종료합니다.
    node.destroy_node()
    rclpy.shutdown()
