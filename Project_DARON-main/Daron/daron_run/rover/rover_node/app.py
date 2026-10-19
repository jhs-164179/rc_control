"""ROS node lifecycle."""
import sys
import rclpy
from .node import RoverNode, ProfileMissing

def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = RoverNode()
    except ProfileMissing as e:
        print(f"\n[프로파일 없음] {e}\n", file=sys.stderr)
        rclpy.shutdown()
        return 2
    except Exception as e:
        print(f"[ERROR] {type(e).__name__}: {e}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

