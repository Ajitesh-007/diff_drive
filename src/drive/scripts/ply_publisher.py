#!/usr/bin/env python3
"""
Publishes a PLY point cloud file as a latched PointCloud2 on /rtabmap/cloud_map.
Uses use_sim_time=true so the header stamp matches the simulation clock.
"""
import sys
import os
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_ply_vertices(filepath):
    """Parse a binary or ASCII PLY file and return Nx6 float32 array of [x, y, z, r, g, b]."""
    with open(filepath, 'rb') as f:
        # --- Parse header ---
        line = f.readline().decode('ascii').strip()
        if line != 'ply':
            raise ValueError(f"Not a PLY file: {filepath}")

        fmt = None
        vertex_count = 0
        properties = []
        in_vertex = False

        while True:
            line = f.readline().decode('ascii').strip()
            if not line: break
            if line.startswith('format'):
                fmt = line.split()[1]
            elif line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
                in_vertex = True
            elif line.startswith('element'):
                in_vertex = False
            elif line.startswith('property') and in_vertex:
                properties.append(line.split())
            elif line == 'end_header':
                break

        prop_names = [p[-1] for p in properties]
        
        has_color = all(c in prop_names for c in ['red', 'green', 'blue'])

        if fmt == 'ascii':
            # Position indices
            xi, yi, zi = prop_names.index('x'), prop_names.index('y'), prop_names.index('z')
            ri, gi, bi = (prop_names.index('red'), prop_names.index('green'), prop_names.index('blue')) if has_color else (-1,-1,-1)
            
            points = np.zeros((vertex_count, 6), dtype=np.float32)
            for i in range(vertex_count):
                vals = f.readline().decode('ascii').split()
                points[i, 0:3] = [float(vals[xi]), float(vals[yi]), float(vals[zi])]
                if has_color:
                    points[i, 3:6] = [float(vals[ri]), float(vals[gi]), float(vals[bi])]
        elif fmt in ('binary_little_endian', 'binary_big_endian'):
            endian = '<' if fmt == 'binary_little_endian' else '>'
            type_map = {
                'float': 'f4', 'double': 'f8',
                'uchar': 'u1', 'char': 'i1',
                'ushort': 'u2', 'short': 'i2',
                'uint': 'u4', 'int': 'i4',
                'float32': 'f4', 'float64': 'f8',
                'uint8': 'u1', 'int8': 'i1',
                'uint16': 'u2', 'int16': 'i2',
                'uint32': 'u4', 'int32': 'i4',
            }
            dt = np.dtype([(p[-1], endian + type_map[p[1]]) for p in properties])
            raw = np.frombuffer(f.read(vertex_count * dt.itemsize), dtype=dt, count=vertex_count)
            
            points = np.zeros((vertex_count, 6), dtype=np.float32)
            points[:, 0] = raw['x']
            points[:, 1] = raw['y']
            points[:, 2] = raw['z']
            if has_color:
                points[:, 3] = raw['red']
                points[:, 4] = raw['green']
                points[:, 5] = raw['blue']
        else:
            raise ValueError(f"Unknown PLY format: {fmt}")

    return points


class PlyMapPublisher(Node):
    def __init__(self, ply_path):
        super().__init__('ply_map_publisher')
        self.get_logger().info(f'Loading PLY: {ply_path}')

        points = load_ply_vertices(ply_path)
        self.get_logger().info(f'Loaded {len(points)} points')

        # Build the PointCloud2 message once
        self.cloud_msg = self._make_cloud_msg(points)

        # Latched publisher (Transient Local)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, '/rtabmap/cloud_map', qos)

        # Publish immediately
        self._publish()

        # Re-publish every 15 s
        self.timer = self.create_timer(15.0, self._publish)

    def _publish(self):
        self.cloud_msg.header.stamp.sec = 0
        self.cloud_msg.header.stamp.nanosec = 0
        self.pub.publish(self.cloud_msg)

    @staticmethod
    def _make_cloud_msg(points):
        import struct
        msg = PointCloud2()
        msg.header.frame_id = 'map'

        num_points = len(points)
        msg.height = 1
        msg.width = num_points
        
        # fields: x, y, z (float32) + rgb (packed float32)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * num_points
        msg.is_dense = True
        
        # Pack data: [x, y, z, 0, r, g, b]
        buffer = np.zeros(num_points, dtype=[
            ('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('padding', 'u1'), ('b', 'u1'), ('g', 'u1'), ('r', 'u1')
        ])
        
        buffer['x'] = points[:, 0]
        buffer['y'] = points[:, 1]
        buffer['z'] = points[:, 2]
        buffer['r'] = points[:, 3].astype(np.uint8)
        buffer['g'] = points[:, 4].astype(np.uint8)
        buffer['b'] = points[:, 5].astype(np.uint8)
        
        msg.data = buffer.tobytes()
        return msg


def main():
    rclpy.init()

    # Accept PLY path as first positional arg or --ply_file param
    ply_path = None
    for i, arg in enumerate(sys.argv):
        if arg == '--ply' and i + 1 < len(sys.argv):
            ply_path = sys.argv[i + 1]
            break

    # Also check ROS params
    if ply_path is None:
        # Fallback: look for it in drive/maps
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg = get_package_share_directory('drive')
            ply_path = os.path.join(pkg, 'maps', 'rtabmap_cloud.ply')
        except Exception:
            pass

    if ply_path is None or not os.path.dirname(ply_path):
        print(f'ERROR: PLY file path is invalid: {ply_path}', file=sys.stderr)
        sys.exit(1)

    # --- Auto-Export Logic ---
    db_path = os.path.expanduser('~/.ros/rtabmap.db')
    output_dir = os.path.dirname(ply_path)

    if os.path.isfile(db_path):
        needs_export = False
        db_mtime = os.path.getmtime(db_path)
        
        if not os.path.isfile(ply_path):
            needs_export = True
            print(f"[{'ply_map_publisher'}] 3D map {ply_path} not found. Will generate it from database.")
        else:
            ply_mtime = os.path.getmtime(ply_path)
            if db_mtime > ply_mtime:
                needs_export = True
                print(f"[{'ply_map_publisher'}] Database (~/.ros/rtabmap.db) is newer than the 3D map. Re-exporting...")

        if needs_export:
            print(f"[{'ply_map_publisher'}] Running rtabmap-export... This may take a few seconds.")
            try:
                # Ensure the output directory exists
                os.makedirs(output_dir, exist_ok=True)
                # Run the export tool
                subprocess.run([
                    'rtabmap-export',
                    '--output_dir', output_dir,
                    '--db', db_path
                ], check=True)
                print(f"[{'ply_map_publisher'}] Successfully auto-exported 3D map.")
            except subprocess.CalledProcessError as e:
                print(f"[{'ply_map_publisher'}] ERROR: Failed to auto-export 3D map from database: {e}", file=sys.stderr)
            except FileNotFoundError:
                print(f"[{'ply_map_publisher'}] ERROR: 'rtabmap-export' command not found. Is RTAB-Map installed properly?", file=sys.stderr)

    if not os.path.isfile(ply_path):
        print(f'ERROR: PLY file not found and could not be generated: {ply_path}', file=sys.stderr)
        sys.exit(1)

    node = PlyMapPublisher(ply_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
