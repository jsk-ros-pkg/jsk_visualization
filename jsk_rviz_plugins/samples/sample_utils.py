# -*- coding: utf-8 -*-
"""Helpers of the sample scripts of this directory, which import it from
their own directory (installed next to them on ROS 1 and ROS 2)."""

import contextlib
import os


@contextlib.contextmanager
def exit_on_shutdown():
    """For the main loops of the samples: on ROS 2, SIGINT shuts the rclpy
    context down from its signal handler and raises KeyboardInterrupt in the
    main thread, so the loop ends with KeyboardInterrupt, or with an
    RCLError of a publish() after the context went down. Both are a normal
    exit then. Nothing changes on ROS 1 (rospy)."""
    if os.environ.get('ROS_VERSION', '1') != '2':
        yield
        return
    import rclpy
    import signal
    try:
        yield
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    # Ctrl-C of a terminal reaches the samples twice under ros2 launch
    # (from the terminal and from launch): ignore the second one, which
    # would break the shutdown after the loop
    signal.signal(signal.SIGINT, signal.SIG_IGN)


# The ROS 1 devel space has a relay of this file next to the relays of the
# samples (catkin_install_python), which runs this file in a dict of its own
# instead of the namespace of the module being imported; put the helpers
# there as well, so that `from sample_utils import ...` works through it.
import sys  # noqa: E402
sys.modules[__name__].exit_on_shutdown = exit_on_shutdown
