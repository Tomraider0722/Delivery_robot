"""Small slider window that sets the Nav2 speed limit.

Publishes nav2_msgs/SpeedLimit on /speed_limit, which Nav2's controller
server applies to the regulated pure pursuit controller. The value is
re-published every couple of seconds so it also applies after Nav2 is
restarted. Other nodes (e.g. a delivery mission) can use the same topic.
"""

import tkinter as tk

from nav2_msgs.msg import SpeedLimit
import rclpy
from rclpy.node import Node


class SpeedControl(Node):
    """Publish the speed limit chosen in the window."""

    def __init__(self):
        super().__init__('speed_control')

        self.declare_parameter('max_speed', 0.3)
        self.max_speed = self.get_parameter('max_speed').value

        self.publisher = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.percent = 100.0

    def publish(self, percent):
        """Publish a speed limit as a percentage of the normal top speed."""
        self.percent = float(percent)
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = self.percent
        self.publisher.publish(msg)

    def republish(self):
        """Publish the current limit again."""
        self.publish(self.percent)


class SpeedWindow:
    """Tk window with a slider and preset buttons."""

    def __init__(self, node):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Robot speed')
        self.root.attributes('-topmost', True)

        self.label = tk.Label(self.root, font=('TkDefaultFont', 14))
        self.label.pack(padx=16, pady=(12, 4))

        self.slider = tk.Scale(
            self.root, from_=10, to=100, orient=tk.HORIZONTAL, length=300,
            command=self.on_slider)
        self.slider.set(100)
        self.slider.pack(padx=16, pady=4)

        buttons = tk.Frame(self.root)
        buttons.pack(padx=16, pady=(4, 12))
        for text, value in (('Slow 25%', 25), ('Half 50%', 50),
                            ('Normal 100%', 100)):
            tk.Button(
                buttons, text=text,
                command=lambda v=value: self.slider.set(v)
            ).pack(side=tk.LEFT, padx=4)

        self.update_label(100)
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.tick()

    def update_label(self, percent):
        """Show the limit and the approximate resulting top speed."""
        speed = self.node.max_speed * float(percent) / 100.0
        self.label.config(text=f'{float(percent):.0f}%  (about {speed:.2f} m/s)')

    def on_slider(self, value):
        """Handle a slider change."""
        self.update_label(value)
        self.node.publish(value)

    def tick(self):
        """Re-publish the limit every 2 s (so it survives a Nav2 restart)."""
        self.node.republish()
        self.root.after(2000, self.tick)


def main(args=None):
    """Run the speed control window."""
    rclpy.init(args=args)
    node = SpeedControl()
    try:
        SpeedWindow(node).root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
