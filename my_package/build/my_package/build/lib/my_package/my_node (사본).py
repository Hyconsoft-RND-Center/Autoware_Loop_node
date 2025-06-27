import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import multiprocessing.process
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand, TurnIndicatorsCommand, HazardLightsCommand
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
import multiprocessing
import os
from std_msgs.msg import String  # 필요한 메시지 타입에 맞춰 변경 가능



def write_to_file(queue):
    filename = os.path.expanduser('~/ros2_data.txt')
    while True:
        data = queue.get()
        if data is None:
            break
        with open(filename, 'a') as file:
            file.write(data+ '\n')
        #time.sleep(1)

class TopicDataSaver(Node):
    def __init__(self, queue):
        super().__init__('topic_data_saver')
        # 구독할 토픽 이름과 메시지 타입 설정
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            String,  # 메시지 타입
            'AckermannControlCommand',  # 구독할 토픽 이름으로 변경
             self.listener_callback,
        qos_profile)

        self.subscription  # prevent unused variable warning
        self.i=0
        self.queue = queue




    def listener_callback(self, msg):

        msg = String()
        msg.data = 'injection message: %d' % self.i
        #self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        self.queue.put(msg.data)


def main(args=None):

    rclpy.init(args=args)

    write_queue = multiprocessing.Queue()
    writer_process = multiprocessing.Process(target=write_to_file, args=(write_queue,))
    writer_process.start()

    autowaresub = TopicDataSaver(write_queue)

    try:
        rclpy.spin(autowaresub)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autowaresub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
