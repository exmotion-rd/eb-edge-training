import rclpy
import datetime
from datetime import datetime, timedelta
from rclpy.node import Node
from thermo_msgs.msg import Temperature


class ThermoMonitor(Node):

    def __init__(self):
        super().__init__('thermo_monitor')
        self.subscription = self.create_subscription(Temperature,
                                                     'temperature',
                                                     self.listener_callback,
                                                     10)

    def to_datestr(self, stamp):
        sec = stamp.sec
        msec = stamp.nanosec / 1_000_000
        time = datetime.fromtimestamp(sec) + timedelta(milliseconds=msec)
        return "{0:%Y-%m-%d %H:%M:%S}".format(time)

    def listener_callback(self, msg):
        time = self.to_datestr(msg.header.stamp)
        temp = msg.temperature
        self.get_logger().info('[%s] temperature=%.2f' % (time, temp))


def main(args=None):
    rclpy.init(args=args)

    subscriber = ThermoMonitor()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
