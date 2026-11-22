"""Find the blocks on the tables and racks with the perception camera.

Uses the RGB-D camera on the robot's mast (/perception_camera/...). Every
depth pixel is turned into a 3D point in the map frame. Points that stand just
above a known surface (a table top or a rack shelf, see config/places.yaml)
inside that place's area belong to blocks, so brick walls, chairs and the
table itself are ignored. Each group of such points is one block. Its colour
comes from the image, and its shape (cube, bar, cylinder) and size from the
top face of the 3D points.

Publishes:
    /blocks               std_msgs/String, latched. JSON list of the blocks
                          now in view: colour, shape, size, dimensions, place,
                          surface and position in the map frame.
    /blocks/markers       visualization_msgs/MarkerArray, for RViz.
    /blocks/debug_image   sensor_msgs/Image, the camera image with labels.

Needs the simulation and a map frame (localization or SLAM). Blocks are only
reported for the places the camera can see, so stand in front of a table or
rack first (for example: ros2 run delivery_robot_controller deliver table1).

    ros2 run delivery_robot_perception block_detector
"""

from collections import Counter, deque
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (qos_profile_sensor_data, QoSDurabilityPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

# OpenCV hue (0..179) ranges of the colour names.
HUES = {
    'red': [(0, 6), (173, 179)], 'orange': [(7, 23)], 'yellow': [(24, 38)],
    'green': [(40, 80)], 'cyan': [(82, 98)], 'blue': [(105, 125)],
    'purple': [(128, 148)], 'magenta': [(150, 170)],
}
DRAW = {  # RGB 0..1, for the debug image and the markers
    'red': (0.9, 0.05, 0.05), 'orange': (1.0, 0.5, 0.0),
    'yellow': (1.0, 0.9, 0.05), 'green': (0.1, 0.7, 0.1),
    'cyan': (0.0, 0.8, 0.8), 'blue': (0.1, 0.2, 0.9),
    'purple': (0.5, 0.1, 0.7), 'magenta': (0.9, 0.1, 0.6),
    'unknown': (0.6, 0.6, 0.6),
}
MIN_HEIGHT_ABOVE = 0.012   # points closer to the surface than this are the surface
MAX_HEIGHT_ABOVE = 0.16    # and taller than this is not a block
MIN_PIXELS = 25
TOP_FACE_BAND = 0.008      # points within this of the highest point
SMALL, MEDIUM = 0.055, 0.10
RECT_FILL_MIN = 0.86       # a cylinder top fills less of its box (0.81-0.84, cubes 0.88-0.99)
RASTER_STEP = 0.002        # metres per cell of the rectified top face


def quat_to_matrix(q):
    """3x3 rotation matrix of a quaternion message."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def colour_name(hue, sat):
    """Name of an OpenCV hue, or unknown for grey and unlisted hues."""
    if sat < 90:
        return 'unknown'
    for name, ranges in HUES.items():
        if any(a <= hue <= b for a, b in ranges):
            return name
    return 'unknown'


def size_class(biggest):
    """small, medium or large from the longest edge in metres."""
    return 'small' if biggest <= SMALL else (
        'medium' if biggest <= MEDIUM else 'large')


def fit_rect(top_xy_m):
    """Footprint of a block from its top-face points.

    Returns (long_side, short_side) in metres; this is the accurate
    measurement (the points lie exactly on the surface).
    """
    pts = (top_xy_m * 1000.0).astype(np.float32)
    (_, _), (w, h), _ = cv2.minAreaRect(pts)
    return max(w, h) / 1000.0, max(1e-3, min(w, h)) / 1000.0


def raster_fill(raster):
    """How much of its bounding rectangle a top-down raster fills (0..1).

    A cylinder's top is a circle and fills about 0.79; a cube or bar fills
    nearly all of it. Returns None when the raster is empty.
    """
    contours, _ = cv2.findContours(
        raster.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 4:
        return None
    (_, _), (w, h), _ = cv2.minAreaRect(cnt)
    return cv2.contourArea(cv2.convexHull(cnt)) / max(1e-3, w * h)


class BlockDetector(Node):
    """Detect, classify and locate blocks in the perception camera view."""

    def __init__(self):
        super().__init__(
            'block_detector',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        share = get_package_share_directory('delivery_robot_perception')
        self.declare_parameter(
            'places_file', os.path.join(share, 'config', 'places.yaml'))
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('max_rate', 3.0)
        # The map is 2D (z = 0 at base_link) but the robot's base_link rests
        # this far above the real floor, so add it to every height.
        self.declare_parameter('base_link_height', 0.05)
        # Frames in a row that must agree before the list is published.
        self.declare_parameter('confirm_frames', 3)
        # Largest time gap between the colour and the depth image (s). The
        # colour only names the block, so a gap is harmless while standing still.
        self.declare_parameter('max_gap', 1.0)
        self.max_gap = self.get_parameter('max_gap').value
        self.base_height = self.get_parameter('base_link_height').value
        self.confirm = self.get_parameter('confirm_frames').value
        self.target = self.get_parameter('target_frame').value
        self.min_period = 1.0 / self.get_parameter('max_rate').value
        with open(self.get_parameter('places_file').value) as f:
            places = yaml.safe_load(f)['places']
        # list of (place, surface, x range, y range, z)
        self.surfaces = [
            (place, surface, s['x'], s['y'], s['z'])
            for place, surfaces in places.items()
            for surface, s in surfaces.items()]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.info = None
        self.depth = None
        self.last_time = -1e9
        self.refresh_count = 0
        self.stats = {'frames': 0, 'blank': 0, 'skipped': 0, 'used': 0}
        self.create_timer(10.0, self.log_stats)
        self.tracks = []           # blocks seen recently, with shape votes
        self.pending = None        # signature seen in the latest frames
        self.pending_count = 0
        self.published = None      # signature of the list last published

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.blocks_pub = self.create_publisher(String, '/blocks', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/blocks/markers', 1)
        self.debug_pub = self.create_publisher(Image, '/blocks/debug_image', 1)
        self.create_service(Trigger, '/blocks/refresh', self.on_refresh)
        self.create_subscription(
            CameraInfo, '/perception_camera/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/perception_camera/depth_image', self.on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/perception_camera/image', self.on_image,
            qos_profile_sensor_data)

    # ---- inputs --------------------------------------------------------

    def log_stats(self):
        """Log how many camera frames were used or skipped (every 10 s)."""
        st = self.stats
        if st['frames']:
            self.get_logger().info(
                f"Last 10 s: {st['frames']} colour frames, {st['used']} "
                f"used, {st['blank']} blank depth, {st['skipped']} skipped "
                '(no depth, late, no transform)')
        for key in st:
            st[key] = 0

    def on_refresh(self, request, response):
        """Publish the next list that three fresh frames agree on.

        The list is normally only published when it changes. After a block
        has been moved, a task calls this to get a new list that is based
        on frames taken from now on, even if it looks the same. The response
        message is a number; lists published after this call carry it (or a
        larger one) in their "refresh" field, so a caller can tell them from
        lists that were already on their way.
        """
        self.pending, self.pending_count, self.published = None, 0, None
        self.refresh_count += 1
        response.success = True
        # Lists published from now on carry this number in "refresh".
        response.message = str(self.refresh_count)
        return response

    def on_info(self, msg):
        """Camera intrinsics (fx, fy, cx, cy)."""
        self.info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        """Keep the latest depth image (metres, float32)."""
        arr = np.frombuffer(bytes(msg.data), np.float32)
        self.depth = (msg.header.stamp, arr.reshape(msg.height, msg.width))

    def on_image(self, msg):
        """Process a colour image together with the latest depth image."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.info is None or self.depth is None \
                or now - self.last_time < self.min_period:
            return
        self.stats['frames'] += 1
        stamp_d, depth = self.depth
        gap = abs(Time.from_msg(stamp_d).nanoseconds
                  - Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        if gap > self.max_gap:
            self.get_logger().warn(
                f'Colour and depth images are {gap:.2f} s apart, skipping',
                throttle_duration_sec=5.0)
            self.stats['skipped'] += 1
            return
        # The 3D points come from the depth image, so use the camera pose at
        # the depth image's time (fall back to the newest pose).
        tf = None
        for when in (Time.from_msg(stamp_d), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target, msg.header.frame_id, when,
                    timeout=Duration(seconds=0.2))
                break
            except Exception as err:  # no map frame yet
                last_error = err
        if tf is None:
            self.get_logger().warn(
                f'No transform {self.target} <- {msg.header.frame_id}: '
                f'{last_error}', throttle_duration_sec=5.0)
            self.stats['skipped'] += 1
            return
        self.last_time = now
        rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(
            msg.height, msg.width, 3)
        blocks, labelled = self.detect(rgb, depth, tf)
        if blocks is None:
            self.stats['blank'] += 1
            return
        self.stats['used'] += 1
        self.smooth(blocks, now)
        base = self.lookup_base(Time.from_msg(stamp_d))
        if base is not None:
            self.add_relative(blocks, base)
        self.publish(blocks, labelled, msg.header)

    # ---- detection -----------------------------------------------------

    def smooth(self, blocks, now):
        """Report each block's most common shape and size of the last frames.

        A block whose top face is borderline (a small cube or cylinder) can
        change shape from frame to frame. Without this the list would never
        be the same for several frames in a row.
        """
        for b in blocks:
            track = next(
                (t for t in self.tracks
                 if t['colour'] == b['colour'] and t['place'] == b['place']
                 and math.hypot(t['x'] - b['x'], t['y'] - b['y']) < 0.08),
                None)
            if track is None:
                track = {'colour': b['colour'], 'place': b['place'],
                         'votes': deque(maxlen=9)}
                self.tracks.append(track)
            track.update(x=b['x'], y=b['y'], seen=now)
            track['votes'].append((b['shape'], b['size']))
            (shape, size), count = Counter(track['votes']).most_common(1)[0]
            b['shape'], b['size'] = shape, size
            b['id'] = f"{b['colour']}_{shape}_{size}"
            share = count / len(track['votes'])
            b['shape_confident'] = bool(b['shape_confident'] and share >= 0.75)
        self.tracks = [t for t in self.tracks if now - t['seen'] < 5.0]

    def lookup_base(self, when):
        """Pose of base_link in the target frame (rotation, translation)."""
        for stamp in (when, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target, 'base_link', stamp,
                    timeout=Duration(seconds=0.1))
                t = tf.transform.translation
                return (quat_to_matrix(tf.transform.rotation),
                        np.array([t.x, t.y, t.z + self.base_height]))
            except Exception:  # not available at that time
                continue
        return None

    @staticmethod
    def add_relative(blocks, base):
        """Add each block's position relative to the robot (base_link).

        x forward, y left, z up from the base_link origin. This does not
        depend on how well the robot is localized, only on the camera, so it
        is the number to use for reaching a block (for example with an arm).
        """
        rot, trans = base
        for b in blocks:
            rel = rot.T @ (np.array([b['x'], b['y'], b['z']]) - trans)
            b['relative'] = {'x': round(float(rel[0]), 3),
                             'y': round(float(rel[1]), 3),
                             'z': round(float(rel[2]), 3)}

    def to_map(self, depth, tf):
        """3D point (map frame) of every depth pixel, shape (H, W, 3)."""
        fx, fy, cx, cy = self.info
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth.astype(np.float64)
        pts = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)
        rot = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        trans = np.array([t.x, t.y, t.z + self.base_height])
        self.cam_rot, self.cam_trans = rot, trans
        out = pts.reshape(-1, 3) @ rot.T + trans
        return out.reshape(h, w, 3)

    def rectify_top(self, top_pixels, xy, z_top):
        """Top face as a dense top-down raster (cells of RASTER_STEP).

        A grid on the plane z = z_top is projected into the camera image and
        each cell takes the value of the pixel it lands on, which avoids the
        gaps that the sparse depth samples leave at a shallow viewing angle.
        Returns (raster, x0, y0) with the map position of cell (0, 0).
        """
        fx, fy, cx, cy = self.info
        lo = xy.min(axis=0) - 0.015
        hi = xy.max(axis=0) + 0.015
        xs = np.arange(lo[0], hi[0], RASTER_STEP)
        ys = np.arange(lo[1], hi[1], RASTER_STEP)
        gx, gy = np.meshgrid(xs, ys)
        plane = np.stack([gx, gy, np.full_like(gx, z_top)], -1).reshape(-1, 3)
        cam = (plane - self.cam_trans) @ self.cam_rot
        ok = cam[:, 2] > 0.05
        u = np.round(fx * cam[:, 0] / np.where(ok, cam[:, 2], 1) + cx)
        v = np.round(fy * cam[:, 1] / np.where(ok, cam[:, 2], 1) + cy)
        h, w = top_pixels.shape
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
        raster = np.zeros(len(plane), np.uint8)
        raster[ok] = top_pixels[v[ok].astype(int), u[ok].astype(int)]
        return raster.reshape(gy.shape), lo[0], lo[1]

    def detect(self, rgb, depth, tf):
        """Return (blocks, image labels) for one frame."""
        valid = np.isfinite(depth) & (depth > 0.2) & (depth < 5.0)
        if valid.mean() < 0.2:
            # A blank frame (the simulated camera sometimes delivers one under
            # load) says nothing about the blocks: ignore it.
            self.get_logger().warn('Blank depth frame, skipping',
                                   throttle_duration_sec=5.0)
            return None, rgb
        pts = self.to_map(np.where(valid, depth, 0.0), tf)

        # pixels above a known surface, and which surface they belong to
        surface_id = np.full(depth.shape, -1, np.int16)
        for i, (_, _, xr, yr, zs) in enumerate(self.surfaces):
            inside = (valid
                      & (pts[..., 0] >= xr[0]) & (pts[..., 0] <= xr[1])
                      & (pts[..., 1] >= yr[0]) & (pts[..., 1] <= yr[1])
                      & (pts[..., 2] >= zs + MIN_HEIGHT_ABOVE)
                      & (pts[..., 2] <= zs + MAX_HEIGHT_ABOVE))
            surface_id[inside] = i
        mask = (surface_id >= 0).astype(np.uint8)
        t = self.cam_trans
        yaw = math.atan2(self.cam_rot[1, 0], self.cam_rot[0, 0])
        self.get_logger().debug(
            f'camera at ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) yaw-ish '
            f'{yaw:.2f}, {int(mask.sum())} pixels above a surface')
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        blocks = []
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS:
                continue
            sel = labels == i
            ids = surface_id[sel]
            sid = int(np.bincount(ids[ids >= 0]).argmax())
            place, surface, _, _, zs = self.surfaces[sid]
            p = pts[sel]
            h_pix = hsv[sel]
            colour = colour_name(int(np.median(h_pix[:, 0])),
                                 int(np.median(h_pix[:, 1])))
            if colour == 'unknown':
                continue            # grey or unlisted colour, e.g. a rack post
            z_top = float(np.percentile(p[:, 2], 98))
            top = p[p[:, 2] >= z_top - TOP_FACE_BAND][:, :2]
            if len(top) < 8:
                top = p[:, :2]
            top_pixels = sel & (pts[..., 2] >= z_top - TOP_FACE_BAND)
            raster, _, _ = self.rectify_top(top_pixels, top, z_top)
            long_side, short_side = fit_rect(top)
            fill = raster_fill(raster)
            if fill is None:
                continue
            aspect = long_side / short_side
            shape = ('bar' if aspect >= 1.6 else
                     'cylinder' if fill <= RECT_FILL_MIN else 'cube')
            hue = int(np.median(h_pix[:, 0]))
            height = z_top - zs
            if long_side > 0.20 or max(long_side, height) < 0.02:
                continue            # not a block (too big or too small)
            cx, cy = top.mean(axis=0)
            dims = [round(long_side, 3), round(short_side, 3),
                    round(height, 3)]
            x, y, w, h = (int(stats[i, cv2.CC_STAT_LEFT]),
                          int(stats[i, cv2.CC_STAT_TOP]),
                          int(stats[i, cv2.CC_STAT_WIDTH]),
                          int(stats[i, cv2.CC_STAT_HEIGHT]))
            blocks.append({
                'colour': colour, 'shape': shape,
                'size': size_class(max(long_side, height)),
                'dimensions_m': dims, 'place': place, 'surface': surface,
                # False when a cube and a cylinder are hard to tell apart
                # (top face fill close to the cut-off, small block far away)
                'shape_confident': bool(
                    shape == 'bar' or abs(fill - RECT_FILL_MIN) > 0.04),
                'x': round(float(cx), 3), 'y': round(float(cy), 3),
                'z': round(zs + height / 2.0, 3),
                'pixels': int(stats[i, cv2.CC_STAT_AREA]),
                # numbers behind the decisions, for checking and tuning
                'features': {'hue': hue, 'top_pixels': int(top_pixels.sum()),
                             'fill': round(float(fill), 2),
                             'aspect': round(aspect, 2)},
                '_box': (x, y, w, h)})
        blocks.sort(key=lambda b: (b['place'], b['x'], b['y']))
        for n, b in enumerate(blocks, 1):
            b['id'] = f"{b['colour']}_{b['shape']}_{b['size']}"
        return blocks, rgb

    # ---- outputs -------------------------------------------------------

    def publish(self, blocks, rgb, header):
        """Publish the JSON list, the markers and the debug image."""
        public = [{k: v for k, v in b.items() if not k.startswith('_')}
                  for b in blocks]
        # Only publish a list after several frames in a row agree, so a
        # moving robot or one bad frame does not produce wrong lists.
        signature = tuple((b['id'], b['place'], b['x'], b['y'])
                          for b in public)
        if self.same_list(signature, self.pending):
            self.pending_count += 1
        else:
            self.pending, self.pending_count = signature, 1
        if (self.pending_count >= self.confirm
                and not self.same_list(signature, self.published)):
            self.published = signature
            self.blocks_pub.publish(String(data=json.dumps(
                {'frame': self.target, 'refresh': self.refresh_count,
                 'blocks': public})))
            names = ', '.join(
                f"{b['id']}@{b['place']}" for b in public) or 'none'
            self.get_logger().info(f'Blocks in view: {names}')
        self.publish_markers(public)
        if self.debug_pub.get_subscription_count():
            self.publish_debug(rgb, blocks, header)

    @staticmethod
    def same_list(a, b):
        """Say whether two block lists hold the same blocks (within 6 cm)."""
        if a is None or b is None or len(a) != len(b):
            return False
        return all(p[0] == q[0] and p[1] == q[1]
                   and abs(p[2] - q[2]) < 0.06 and abs(p[3] - q[3]) < 0.06
                   for p, q in zip(a, b))

    def publish_markers(self, blocks):
        """Publish RViz markers, a coloured shape and a label per block."""
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for n, b in enumerate(blocks):
            r, g, bl = DRAW.get(b['colour'], DRAW['unknown'])
            m = Marker()
            m.header.frame_id = self.target
            m.header.stamp = stamp
            m.ns, m.id = 'blocks', n
            m.type = (Marker.CYLINDER if b['shape'] == 'cylinder'
                      else Marker.CUBE)
            m.pose.position = Point(x=b['x'], y=b['y'], z=b['z'])
            m.pose.orientation.w = 1.0
            long_side, short_side, height = b['dimensions_m']
            m.scale.x = long_side
            m.scale.y = long_side if b['shape'] == 'cylinder' else short_side
            m.scale.z = height
            m.color = ColorRGBA(r=r, g=g, b=bl, a=0.9)
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id = 'block_labels', n
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position = Point(x=b['x'], y=b['y'], z=b['z'] + 0.12)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.05
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"{b['colour']} {b['shape']}\n{b['size']}, {b['place']}"
            arr.markers.append(t)
        self.marker_pub.publish(arr)

    def publish_debug(self, rgb, blocks, header):
        """Camera image with a box and a label around every block."""
        img = rgb.copy()
        for b in blocks:
            x, y, w, h = b['_box']
            r, g, bl = DRAW.get(b['colour'], DRAW['unknown'])
            cv2.rectangle(img, (x - 3, y - 3), (x + w + 3, y + h + 3),
                          (255, 255, 255), 1)
            label = f"{b['colour']} {b['shape']} {b['size']}"
            cv2.putText(img, label, (max(0, x - 20), max(12, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2,
                        cv2.LINE_AA)
            cv2.putText(img, label, (max(0, x - 20), max(12, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (int(r * 255), int(g * 255), int(bl * 255)), 1,
                        cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = img.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    """Run the block detector."""
    rclpy.init(args=args)
    node = BlockDetector()
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
