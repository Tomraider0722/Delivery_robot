"""Tell whether something is on the robot's deck, from its load sensor.

The deck has a contact sensor (a thin plate that reports contact while an
object stands on it). This node reads it and publishes

    /cargo_present   std_msgs/Bool, latched. True while an object is on the deck.

The state only changes after it has been steady for a moment (`settle`
seconds), so an object that is being put down or lifted off does not make it
flicker. `deliver` and `courier` use it to know that an object was put on the
robot or taken off.

    ros2 run delivery_robot_perception load_monitor
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (qos_profile_sensor_data, QoSDurabilityPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool

ROBOT = 'delivery_robot'


def touched_by_other(msg):
    """Return True when the plate touches something that is not the robot."""
    for contact in msg.contacts:
        names = (contact.collision1.name, contact.collision2.name)
        if any(n and not n.startswith(ROBOT) for n in names):
            return True
    return False


class LoadMonitor(Node):
    """Publish /cargo_present from the deck contact sensor."""

    def __init__(self):
        super().__init__(
            'load_monitor',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('settle', 1.0)
        self.settle = self.get_parameter('settle').value
        self.last_touch = -1e9       # sim time of the last contact message
        self.present = False
        self.candidate = False
        self.candidate_since = 0.0

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, '/cargo_present', latched)
        self.create_subscription(Contacts, '/deck/contact', self.on_contact,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.pub.publish(Bool(data=False))

    def now(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def on_contact(self, msg):
        """Remember when something last touched the plate."""
        if touched_by_other(msg):
            self.last_touch = self.now()

    def tick(self):
        """Publish a change once the state has been steady for `settle` s."""
        now = self.now()
        # The sensor reports every 0.1 s while touched; a gap means no load.
        touched = now - self.last_touch < 0.5
        if touched != self.candidate:
            self.candidate, self.candidate_since = touched, now
        if (self.candidate != self.present
                and now - self.candidate_since >= self.settle):
            self.present = self.candidate
            self.pub.publish(Bool(data=self.present))
            self.get_logger().info(
                'Object on the deck' if self.present else 'Deck is empty')


def main(args=None):
    """Run the load monitor."""
    rclpy.init(args=args)
    node = LoadMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
