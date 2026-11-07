"""Save one rgb8 image from a ROS 2 topic as a PNG file.

Usage: grab_frame.py <image topic> <output png>

Waits up to 60 seconds, prints the width, height and encoding of the frame and
the mean of each colour channel, and exits with 0. Exits with 1 on timeout.
"""

import os
import sys
import time

import numpy as np
from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TIMEOUT_S = 60.0


class FrameGrabber(Node):
    """Keeps the first rgb8 frame that arrives on the topic."""

    def __init__(self, topic):
        """Subscribe to the topic."""
        super().__init__('grab_frame')
        self.frame = None
        self.create_subscription(Image, topic, self.on_image, 10)

    def on_image(self, msg):
        """Store the first frame that is rgb8."""
        if self.frame is None and msg.encoding == 'rgb8':
            self.frame = msg


def to_array(msg):
    """Return the frame as an H x W x 3 uint8 array, dropping row padding."""
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = rows.reshape((msg.height, msg.step))
    return rows[:, :msg.width * 3].reshape((msg.height, msg.width, 3))


def main(argv):
    """Grab one frame and save it. Returns the process exit code."""
    if len(argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    topic, path = argv[1], argv[2]

    rclpy.init()
    node = FrameGrabber(topic)
    start = time.monotonic()
    try:
        while node.frame is None:
            if time.monotonic() - start > TIMEOUT_S:
                print('timeout: no rgb8 frame on %s' % topic, file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)
        msg = node.frame
    finally:
        node.destroy_node()
        rclpy.shutdown()

    pixels = to_array(msg)
    folder = os.path.dirname(path)
    if folder:
        os.makedirs(folder, exist_ok=True)
    PilImage.fromarray(pixels, mode='RGB').save(path)

    means = pixels.reshape(-1, 3).mean(axis=0)
    print('%d %d %s' % (msg.width, msg.height, msg.encoding))
    print('channel means R G B: %.1f %.1f %.1f' % tuple(means))
    print('saved %s' % path)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
