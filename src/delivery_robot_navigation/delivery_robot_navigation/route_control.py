"""Click several points in RViz, then send the robot through all of them.

In RViz pick the "Publish Point" tool once (its "Single click" option is off
in the bundled config, so it stays active) and click the stops in order.
Each click is added to the route and shown as a numbered marker. Press "Go"
(or call /route/go) and the robot drives to every point in turn using Nav2's
FollowWaypoints. Other controls: Undo last, Clear, Cancel.

Services (std_srvs/Trigger): /route/go, /route/undo, /route/clear,
/route/cancel.
"""

import math
import threading
import tkinter as tk

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class RouteControl(Node):
    """Collect clicked points and drive through them with Nav2."""

    def __init__(self):
        super().__init__('route_control')

        self.lock = threading.Lock()
        self.points = []
        self.status = 'Click points in RViz (Publish Point tool)'
        self.busy = False
        self.goal_handle = None

        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked, 10)

        marker_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/route_markers', marker_qos)

        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.create_service(Trigger, '/route/go', self.srv_go)
        self.create_service(Trigger, '/route/undo', self.srv_undo)
        self.create_service(Trigger, '/route/clear', self.srv_clear)
        self.create_service(Trigger, '/route/cancel', self.srv_cancel)

        self.publish_markers()

    # ------------------------------------------------------------------
    # Route editing
    # ------------------------------------------------------------------

    def clicked(self, msg):
        """Add a clicked point to the route."""
        if msg.header.frame_id != 'map':
            self.set_status(
                f'Ignored click in frame "{msg.header.frame_id}" '
                '(set the RViz Fixed Frame to map)')
            return
        with self.lock:
            if self.busy:
                self.status = 'Route is running; wait or Cancel first'
                return
            self.points.append((msg.point.x, msg.point.y))
            self.status = f'{len(self.points)} point(s) - press Go'
        self.publish_markers()

    def undo(self):
        """Remove the last point."""
        with self.lock:
            if self.busy or not self.points:
                return False
            self.points.pop()
            self.status = f'{len(self.points)} point(s)'
        self.publish_markers()
        return True

    def clear(self):
        """Remove all points."""
        with self.lock:
            if self.busy:
                return False
            self.points = []
            self.status = 'Route cleared'
        self.publish_markers()
        return True

    def set_status(self, text):
        """Update the status line."""
        with self.lock:
            self.status = text

    # ------------------------------------------------------------------
    # Running the route
    # ------------------------------------------------------------------

    def make_poses(self):
        """Build the pose list, each facing the direction of travel."""
        poses = []
        for i, (x, y) in enumerate(self.points):
            if i + 1 < len(self.points):
                nx, ny = self.points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif i > 0:
                px, py = self.points[i - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def go(self):
        """Send the route to Nav2. Returns (ok, message)."""
        with self.lock:
            if self.busy:
                return False, 'Route already running'
            if not self.points:
                return False, 'No points to visit'
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.set_status('Nav2 follow_waypoints is not available')
            return False, 'Nav2 follow_waypoints is not available'

        with self.lock:
            goal = FollowWaypoints.Goal()
            goal.poses = self.make_poses()
            self.busy = True
            count = len(goal.poses)
            self.status = f'Sending {count} point(s) to Nav2...'

        future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_response)
        return True, f'Route of {count} point(s) sent'

    def goal_response(self, future):
        """Handle Nav2 accepting or rejecting the route."""
        handle = future.result()
        if not handle.accepted:
            with self.lock:
                self.busy = False
                self.status = 'Nav2 rejected the route'
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.result)

    def feedback(self, msg):
        """Show which point the robot is heading to."""
        with self.lock:
            total = len(self.points)
            self.status = (
                f'Driving to point {msg.feedback.current_waypoint + 1} '
                f'of {total}')

    def result(self, future):
        """Handle the route finishing."""
        res = future.result()
        missed = list(res.result.missed_waypoints)
        with self.lock:
            self.busy = False
            self.goal_handle = None
            if res.status == 4 and not missed:
                self.status = 'Route complete'
                self.points = []
            elif res.status == 5:
                self.status = 'Route cancelled'
            else:
                self.status = f'Route finished, missed points: {missed}'
        self.publish_markers()

    def cancel(self):
        """Cancel the running route."""
        if self.goal_handle is None:
            return False
        self.goal_handle.cancel_goal_async()
        return True

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def srv_go(self, request, response):
        """Service: send the route."""
        response.success, response.message = self.go()
        return response

    def srv_undo(self, request, response):
        """Service: remove the last point."""
        response.success = self.undo()
        response.message = 'ok' if response.success else 'nothing to undo'
        return response

    def srv_clear(self, request, response):
        """Service: remove all points."""
        response.success = self.clear()
        response.message = 'ok' if response.success else 'route is running'
        return response

    def srv_cancel(self, request, response):
        """Service: cancel the running route."""
        response.success = self.cancel()
        response.message = 'ok' if response.success else 'no route running'
        return response

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------

    def publish_markers(self):
        """Publish numbered spheres and a line through the route."""
        array = MarkerArray()

        wipe = Marker()
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)

        with self.lock:
            points = list(self.points)

        for i, (x, y) in enumerate(points):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.ns = 'route_points'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.6, 0.0
            sphere.color.a = 1.0
            sphere.lifetime = Duration()
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = 'map'
            label.ns = 'route_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(i + 1)
            array.markers.append(label)

        if len(points) > 1:
            line = Marker()
            line.header.frame_id = 'map'
            line.ns = 'route_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.pose.orientation.w = 1.0
            line.scale.x = 0.05
            line.color.r, line.color.g, line.color.b = 1.0, 0.6, 0.0
            line.color.a = 0.8
            for x, y in points:
                p = PointStamped().point
                p.x, p.y, p.z = x, y, 0.1
                line.points.append(p)
            array.markers.append(line)

        self.marker_pub.publish(array)


class RouteWindow:
    """Small Tk window with Go / Undo / Clear / Cancel."""

    def __init__(self, node):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Route')
        self.root.attributes('-topmost', True)
        self.root.geometry('+20+170')

        self.status = tk.Label(
            self.root, text='', wraplength=300, justify=tk.LEFT,
            font=('TkDefaultFont', 11))
        self.status.pack(padx=16, pady=(12, 6))

        buttons = tk.Frame(self.root)
        buttons.pack(padx=16, pady=(0, 12))
        for text, action in (('Go', self.go), ('Undo', node.undo),
                             ('Clear', node.clear), ('Cancel', node.cancel)):
            tk.Button(buttons, text=text, width=7,
                      command=action).pack(side=tk.LEFT, padx=3)

        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.refresh()

    def go(self):
        """Send the route and show any problem."""
        ok, message = self.node.go()
        if not ok:
            self.node.set_status(message)

    def refresh(self):
        """Show the node's status text."""
        with self.node.lock:
            text = self.node.status
        self.status.config(text=text)
        self.root.after(200, self.refresh)


def main(args=None):
    """Run the route control node and window."""
    rclpy.init(args=args)
    node = RouteControl()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        RouteWindow(node).root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
