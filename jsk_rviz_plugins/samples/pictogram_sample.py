#!/usr/bin/env python3

from jsk_rviz_plugins.msg import Pictogram, PictogramArray
import rclpy
from rclpy.node import Node


PICTOGRAMS = ['phone', 'mobile', 'mouse', 'address', 'mail', 'paper-plane',
              'pencil', 'feather', 'attach', 'inbox', 'reply', 'forward',
              'user', 'users', 'location', 'map', 'heart', 'star', 'home',
              'search', 'bell', 'flag', 'cog', 'tools', 'camera', 'eye']


class PictogramSample(Node):
    """Publish the pictograms shown by config/pictogram_sample.rviz."""

    def __init__(self):
        super().__init__('pictogram_sample')
        self.pictogram_pub = self.create_publisher(Pictogram, '/pictogram', 1)
        self.array_pub = self.create_publisher(
            PictogramArray, '/pictogram_array', 1)
        self.count = 0
        self.timer = self.create_timer(1.0, self._timer_cb)

    def _make_pictogram(self, character, x, y):
        pictogram = Pictogram()
        pictogram.header.frame_id = 'base_link'
        pictogram.header.stamp = self.get_clock().now().to_msg()
        pictogram.mode = Pictogram.PICTOGRAM_MODE
        pictogram.action = Pictogram.JUMP
        pictogram.character = character
        pictogram.pose.position.x = float(x)
        pictogram.pose.position.y = float(y)
        # rotate the square so that it faces +x of base_link
        pictogram.pose.orientation.w = 0.7
        pictogram.pose.orientation.x = 0.0
        pictogram.pose.orientation.y = -0.7
        pictogram.pose.orientation.z = 0.0
        pictogram.size = 1.0
        pictogram.color.r = 25 / 255.0
        pictogram.color.g = 255 / 255.0
        pictogram.color.b = 240 / 255.0
        pictogram.color.a = 1.0
        return pictogram

    def _timer_cb(self):
        self.pictogram_pub.publish(
            self._make_pictogram(
                PICTOGRAMS[self.count % len(PICTOGRAMS)], 0, 0))

        array = PictogramArray()
        array.header.frame_id = 'base_link'
        array.header.stamp = self.get_clock().now().to_msg()
        for i in range(5):
            character = PICTOGRAMS[(self.count + i + 1) % len(PICTOGRAMS)]
            array.pictograms.append(self._make_pictogram(character, 0, i + 2))
        self.array_pub.publish(array)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PictogramSample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
