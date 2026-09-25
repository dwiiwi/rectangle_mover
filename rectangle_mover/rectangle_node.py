# Copyright 2026 dwislam
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 node that drives a robot in a rectangular trajectory using TF feedback."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import tf2_ros


class RectangleMover(Node):
    """Move a robot along a rectangular path using odom -> base_footprint TF."""

    def __init__(self):
        """Initialize publisher, TF listener, control timer and state."""
        super().__init__('rectangle_mover')

        # Publisher untuk mengirim perintah ke robot
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # TF buffer dan listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Control loop 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # =========================
        # PARAMETER
        # =========================

        self.linear_speed = 0.5
        self.angular_speed = 0.5

        # Ukuran persegi panjang
        self.long_distance = 3.0
        self.short_distance = 2.0

        # Sudut target
        self.turn_angle = math.pi / 2.0

        # =========================
        # ROBOT STATE
        # =========================

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.state = 'WAITING'

        self.get_logger().info(
            'Rectangle Mover started.'
        )

    def update_robot_pose(self):
        """
        Update current pose from the odom -> base_footprint TF.

        Returns True if the TF lookup succeeded, False otherwise.
        """
        try:

            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time()
            )

            # Posisi
            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y

            # Quaternion
            q = transform.transform.rotation

            # Quaternion -> yaw
            sin_yaw = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cos_yaw = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            self.current_yaw = math.atan2(
                sin_yaw,
                cos_yaw
            )

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            return False

    def normalize_angle(self, angle):
        """Wrap an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def distance_from_start(self):
        """Return Euclidean distance from the pose recorded at start_forward()."""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y

        return math.sqrt(
            dx * dx +
            dy * dy
        )

    def start_forward(self):
        """Record the current pose as the origin for a forward segment."""
        self.start_x = self.current_x
        self.start_y = self.current_y

    def start_turn(self):
        """Record the current yaw as the origin for a turn segment."""
        self.start_yaw = self.current_yaw

    def turn_completed(self):
        """Return True if the accumulated turn has reached turn_angle."""
        angle = self.normalize_angle(
            self.current_yaw - self.start_yaw
        )

        return abs(angle) >= self.turn_angle

    def control_loop(self):
        """Run one FSM step: read TF, compute command, publish, transition state."""
        # Update posisi robot dari TF
        if not self.update_robot_pose():
            return

        msg = Twist()

        # =====================================================
        # WAITING
        # =====================================================

        if self.state == 'WAITING':

            self.start_forward()

            self.state = 'FORWARD_LONG'

            self.get_logger().info(
                'TF received. Starting rectangle.'
            )

        # =====================================================
        # FORWARD LONG 1
        # =====================================================

        elif self.state == 'FORWARD_LONG':

            distance = self.distance_from_start()

            if distance < self.long_distance:

                msg.linear.x = self.linear_speed

            else:

                self.start_turn()

                self.state = 'TURN_1'

                self.get_logger().info(
                    'Long side completed -> turning 90 degrees.'
                )

        # =====================================================
        # TURN 1
        # =====================================================

        elif self.state == 'TURN_1':

            if not self.turn_completed():

                msg.angular.z = self.angular_speed

            else:

                self.start_forward()

                self.state = 'FORWARD_SHORT'

                self.get_logger().info(
                    '90 degrees completed -> short side.'
                )

        # =====================================================
        # FORWARD SHORT 1
        # =====================================================

        elif self.state == 'FORWARD_SHORT':

            distance = self.distance_from_start()

            if distance < self.short_distance:

                msg.linear.x = self.linear_speed

            else:

                self.start_turn()

                self.state = 'TURN_2'

                self.get_logger().info(
                    'Short side completed -> turning 90 degrees.'
                )

        # =====================================================
        # TURN 2
        # =====================================================

        elif self.state == 'TURN_2':

            if not self.turn_completed():

                msg.angular.z = self.angular_speed

            else:

                self.start_forward()

                self.state = 'FORWARD_LONG_2'

                self.get_logger().info(
                    '90 degrees completed -> long side.'
                )

        # =====================================================
        # FORWARD LONG 2
        # =====================================================

        elif self.state == 'FORWARD_LONG_2':

            distance = self.distance_from_start()

            if distance < self.long_distance:

                msg.linear.x = self.linear_speed

            else:

                self.start_turn()

                self.state = 'TURN_3'

                self.get_logger().info(
                    'Long side completed -> turning 90 degrees.'
                )

        # =====================================================
        # TURN 3
        # =====================================================

        elif self.state == 'TURN_3':

            if not self.turn_completed():

                msg.angular.z = self.angular_speed

            else:

                self.start_forward()

                self.state = 'FORWARD_SHORT_2'

                self.get_logger().info(
                    '90 degrees completed -> short side.'
                )

        # =====================================================
        # FORWARD SHORT 2
        # =====================================================

        elif self.state == 'FORWARD_SHORT_2':

            distance = self.distance_from_start()

            if distance < self.short_distance:

                msg.linear.x = self.linear_speed

            else:

                self.start_turn()

                self.state = 'TURN_4'

                self.get_logger().info(
                    'Short side completed -> final 90 degrees.'
                )

        # =====================================================
        # TURN 4
        # =====================================================

        elif self.state == 'TURN_4':

            if not self.turn_completed():

                msg.angular.z = self.angular_speed

            else:

                self.state = 'DONE'

                self.get_logger().info(
                    'Rectangle completed.'
                )

        # =====================================================
        # DONE
        # =====================================================

        elif self.state == 'DONE':

            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(msg)


def main(args=None):
    """Entry point: spin the RectangleMover node until interrupted."""
    rclpy.init(args=args)

    node = RectangleMover()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Pastikan robot berhenti
        stop_msg = Twist()

        node.cmd_pub.publish(stop_msg)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
