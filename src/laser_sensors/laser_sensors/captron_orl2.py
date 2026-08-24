#!/usr/bin/env python3
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Captron ORL2-40T laser cross (TCP measurement) sensor ROS2 node.

The physical ORL2-40T-2PS6 is a laser *cross* Tool-Center-Point measuring unit:
two perpendicular laser lines meet at an intersection inside a 40 mm window and
the device exposes ``2x PNP-NO`` digital outputs, one per beam. An output is
active whenever an object (typically a robot tool tip) interrupts that beam at
the intersection.

Device datasheet and models: https://www.captron.com/products/detail/orl2-40t-2ps6/
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.publisher import Publisher
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from laser_sensors.msg import BeamTriggered


class CaptronORL2(Node):
    """ROS2 driver node for the Captron ORL2-40T laser cross sensor."""

    def __init__(self, node_name: str = 'captron_orl2') -> None:
        """Declare parameters and set up publishers (and, in sim, subscribers)."""
        super().__init__(node_name)

        self._simulated: bool = self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(
                description='Run against a simulated Gazebo sensor (True) or '
                'real hardware (False). Defaults to True.',
            ),
        ).get_parameter_value().bool_value

        self._detection_distance: float = self.declare_parameter(
            'detection_distance',
            0.04,
            ParameterDescriptor(
                description='Maximum beam range, in metres, at which a beam is '
                'considered interrupted. Defaults to 0.04 m (the 40 mm window).',
            ),
        ).get_parameter_value().double_value

        # Publish each PNP-NO output as its own stamped boolean so consumers see
        # an identical topic layout whether the sensor is real or simulated and
        # can rely on the header timestamp for each triggered state.
        self._output_1_pub = self.create_publisher(BeamTriggered, f'{self.get_name()}/x_axis_triggered', 10)
        self._output_2_pub = self.create_publisher(BeamTriggered, f'{self.get_name()}/y_axis_triggered', 10)

        if self._simulated:
            self._setup_simulated()
        else:
            self.get_logger().warning(
                'Real hardware mode is not implemented yet; no outputs will be '
                'published. Set the "simulated" parameter to True.'
            )

    def _setup_simulated(self) -> None:
        """Subscribe to the Gazebo-bridged single-ray beam sensors."""
        self.create_subscription(
            LaserScan,
            'simulated_laser_cross/beam_x_axis',
            self._beam_1_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            'simulated_laser_cross/beam_y_axis',
            self._beam_2_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Simulated Captron ORL2 ready (detection_distance="
            f"{self._detection_distance:.3f} m): "
            f"'simulated_laser_cross/beam_x_axis' -> '{self.get_name()}/x_axis_triggered', "
            f"'simulated_laser_cross/beam_y_axis' -> '{self.get_name()}/y_axis_triggered'"
        )

    def _beam_1_callback(self, scan: LaserScan) -> None:
        """Publish output 1 from the first simulated beam scan."""
        self._publish_output(self._output_1_pub, scan)

    def _beam_2_callback(self, scan: LaserScan) -> None:
        """Publish output 2 from the second simulated beam scan."""
        self._publish_output(self._output_2_pub, scan)

    def _publish_output(self, publisher: Publisher, scan: LaserScan) -> None:
        """Convert a single-ray scan into a beam-interrupted boolean output."""
        interrupted = any(
            scan.range_min <= distance <= self._detection_distance
            for distance in scan.ranges
        )
        msg = BeamTriggered()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = scan.header.frame_id
        msg.triggered = interrupted
        publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Spin the Captron ORL2 sensor node until shutdown."""
    rclpy.init(args=args)
    node = CaptronORL2()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()