"""
robot_bridge: Raspberry Pi <-> Arduino serial bridge for the IMU lab.

- Subscribes to imu/data, converts the quaternion to yaw (deg, CCW positive,
  -180..180) and sends it to the Arduino as text lines:  "Y<yaw>\n"
- Everything you type in this terminal is forwarded to the Arduino as
  key commands (e.g. type  a  + Enter to start the anticlockwise hexagon).
- Everything the Arduino prints is shown here.

Run:
  ros2 run sensor_visualize robot_bridge --ros-args -p port:=/dev/ttyACM0
Parameters:
  port (str)        serial device of the Arduino     default /dev/ttyACM0
  baud (int)        must match Serial.begin()        default 115200
  rate_hz (float)   yaw messages per second          default 50.0
  invert_yaw (bool) flip sign if the IMU is mounted upside down / yaw grows
                    when turning RIGHT                default False
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    print('pyserial missing: sudo apt install python3-serial')
    raise


class RobotBridge(Node):

    def __init__(self):
        super().__init__('robot_bridge')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baud = self.declare_parameter('baud', 115200).value
        rate = self.declare_parameter('rate_hz', 50.0).value
        self.invert = self.declare_parameter('invert_yaw', False).value

        self.ser = serial.Serial(port, baud, timeout=0)
        self.get_logger().info(f'Opened {port} @ {baud}. Waiting for Arduino reset...')
        time.sleep(2.0)   # opening the port resets the Arduino (DTR line)

        self.lock = threading.Lock()
        self.yaw = None
        self.rx_buffer = ''

        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.create_timer(1.0 / rate, self.send_yaw)
        self.create_timer(0.01, self.read_arduino)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        self.get_logger().info('Type a command letter + Enter (a, w, n, i, x, p, ...)')

    def imu_callback(self, msg):
        q = msg.orientation
        if q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z < 0.5:
            return   # invalid quaternion (sensor not ready)
        # Yaw about the z axis from a quaternion (ZYX convention), CCW positive
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.yaw = -yaw if self.invert else yaw

    def send_yaw(self):
        if self.yaw is not None:
            self.write(f'Y{self.yaw:.2f}\n')

    def write(self, text):
        with self.lock:   # never interleave a yaw line with a command
            self.ser.write(text.encode('ascii'))

    def read_arduino(self):
        n = self.ser.in_waiting
        if n:
            self.rx_buffer += self.ser.read(n).decode('ascii', errors='replace')
            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                print(f'[arduino] {line.rstrip()}', flush=True)

    def keyboard_loop(self):
        for line in sys.stdin:
            for ch in line.strip():
                if ch != 'Y':        # 'Y' is reserved for yaw messages
                    self.write(ch)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write('s')   # stop servos on exit
        node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
