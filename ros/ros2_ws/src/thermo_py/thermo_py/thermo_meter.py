import rclpy
from datetime import datetime, timedelta
from rclpy.node import Node
from std_msgs.msg import Header
from thermo_msgs.msg import Temperature

# センサーのランダム値生成用
import random


class DummySensor:
    '''温度を返すダミーのセンサー'''

    @property
    def value(self):
        return random.random() * 100


class ThermoMeter(Node):

    def __init__(self, sensor):
        super().__init__('thermo_meter')
        self._sensor = sensor
        # パブリッシャの作成
        self._publisher = self.create_publisher(Temperature,
                                                topic='temperature',
                                                qos_profile=10)
        # コールバックタイマーの設定
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._id = 0

    def timer_callback(self):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._id)
        msg.temperature = self._sensor.value
        self._publisher.publish(msg)
        self.get_logger().info(
            f'publish temperature: id={msg.header.frame_id}, temperature={msg.temperature}'
        )
        self._id += 1


def main(args=None):
    rclpy.init(args=args)

    thermo_meter = ThermoMeter(sensor=DummySensor())
    rclpy.spin(thermo_meter)

    # 終了処理
    # 記述しない場合はガーベッジコレクタによってノードの破棄が実行される
    thermo_meter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
