"""Small window to start deliveries with buttons.

Pick the places (in order) with the "Add" buttons, press Deliver, and the
robot drives to each one. At every stop press "Delivered" once the cargo
has been taken and the robot moves on; at the end it returns to the dock.
Cancel stops the robot and ends the mission.

Needs the simulation, localization and Nav2 running (bringup.launch.py).
"""

import os
import sys
import threading
import tkinter as tk

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args

from delivery_robot_controller.deliver import Deliver, load_locations


class DeliveryPanel:
    """Tk window driving a Deliver node from a worker thread."""

    def __init__(self, node, locations, home):
        self.node = node
        self.locations = locations
        self.home = home
        self.queue = []
        self.running = False
        self.waiting = False
        self.status = 'Pick the places to deliver to'
        node.on_status = self.set_status

        self.root = tk.Tk()
        self.root.title('Delivery')
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        places = tk.Frame(self.root)
        places.pack(padx=10, pady=(10, 4))
        tk.Label(places, text='Add a stop:').pack(side='left')
        for name in locations:
            if name != home:
                tk.Button(places, text=name,
                          command=lambda n=name: self.add(n)
                          ).pack(side='left', padx=2)

        self.queue_label = tk.Label(self.root, width=40, anchor='w')
        self.queue_label.pack(padx=10)

        row = tk.Frame(self.root)
        row.pack(padx=10, pady=4)
        self.deliver_btn = tk.Button(row, text='Deliver',
                                     command=self.start)
        self.deliver_btn.pack(side='left', padx=2)
        self.done_btn = tk.Button(row, text='Delivered',
                                  command=self.delivered)
        self.done_btn.pack(side='left', padx=2)
        self.undo_btn = tk.Button(row, text='Undo', command=self.undo)
        self.undo_btn.pack(side='left', padx=2)
        self.clear_btn = tk.Button(row, text='Clear', command=self.clear)
        self.clear_btn.pack(side='left', padx=2)
        self.cancel_btn = tk.Button(row, text='Cancel', command=self.cancel)
        self.cancel_btn.pack(side='left', padx=2)

        self.status_label = tk.Label(self.root, width=40, anchor='w',
                                     wraplength=300, justify='left')
        self.status_label.pack(padx=10, pady=(0, 10))

        self.refresh()
        self.root.after(200, self.refresh)

    # ---- called from the worker thread (only sets a string) ----------

    def set_status(self, text):
        """Remember the latest status line."""
        self.status = text
        self.waiting = text.startswith('Waiting for confirmation')

    # ---- buttons ------------------------------------------------------

    def add(self, name):
        """Queue a stop."""
        if not self.running:
            self.queue.append(name)

    def undo(self):
        """Remove the last queued stop."""
        if not self.running and self.queue:
            self.queue.pop()

    def clear(self):
        """Empty the queue."""
        if not self.running:
            self.queue.clear()

    def start(self):
        """Run the queued stops in a worker thread."""
        if self.running or not self.queue:
            return
        self.running = True
        self.status = 'Starting...'
        stops = list(self.queue)
        threading.Thread(target=self.mission, args=(stops,),
                         daemon=True).start()

    def delivered(self):
        """The cargo was taken: let the robot move on."""
        if self.waiting:
            self.node.confirmed = True

    def cancel(self):
        """Stop the robot and end the mission."""
        if self.running:
            self.status = 'Cancelling...'
            self.node.cancel()

    # ---- mission ------------------------------------------------------

    def mission(self, stops):
        """Worker thread: run the whole delivery."""
        try:
            if not self.node.client.wait_for_server(timeout_sec=5.0):
                self.status = 'Nav2 is not available yet, try again soon'
                return
            delivered, failed, returned = self.node.run_mission(
                stops, self.locations, self.home, False, True, 0.0, 600.0)
            if self.node.cancel_requested:
                self.status = 'Cancelled'
            elif failed or not returned:
                self.status = (f'Done. Delivered {delivered}; could not '
                               f'reach {failed or "the dock"}')
            else:
                self.status = f'Done. Delivered {delivered} and back at dock'
                self.queue.clear()
        finally:
            self.waiting = False
            self.running = False

    # ---- display ------------------------------------------------------

    def refresh(self):
        """Update labels and enable only the buttons that make sense."""
        text = ' -> '.join(self.queue) if self.queue else '(no stops yet)'
        self.queue_label.config(text='Stops: ' + text)
        self.status_label.config(text=self.status)

        idle = 'normal' if not self.running else 'disabled'
        self.deliver_btn.config(
            state='normal' if self.queue and not self.running else 'disabled')
        self.undo_btn.config(state=idle)
        self.clear_btn.config(state=idle)
        self.done_btn.config(state='normal' if self.waiting else 'disabled')
        self.cancel_btn.config(state='normal' if self.running else 'disabled')
        self.root.after(200, self.refresh)


def main(args=None):
    """Open the delivery window."""
    argv = sys.argv if args is None else args
    locations_file = os.path.join(
        get_package_share_directory('delivery_robot_controller'),
        'config', 'locations.yaml')
    locations = load_locations(locations_file)
    home = 'dock'

    rclpy.init(args=argv)
    node = Deliver(
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    panel = DeliveryPanel(node, locations, home)
    try:
        panel.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(remove_ros_args(sys.argv))
