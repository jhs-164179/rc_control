#!/usr/bin/env python3
"""depth 16UC1 -> 컬러맵 rgb8 릴레이 (RViz 에서 눈으로 보기 위한 것).

16UC1 을 RViz Image 디스플레이에 그대로 띄우면 거의 검게 나온다. 유효 범위
[dmin, dmax] 만 잘라 turbo 컬러맵으로 펴서 다시 발행한다. **시각화 전용**이며
매핑 경로(live_tsdf_node)는 원본 16UC1 을 그대로 쓴다.

출력: ~/image (sensor_msgs/Image, rgb8)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from depth_color import range_to_rgb   # 3D 맵(live_tsdf_node)과 같은 함수 — 같은 거리 = 같은 색


class DepthColorize(Node):
    def __init__(self):
        super().__init__("depth_colorize")
        P = self.declare_parameter
        self.src = P("depth_topic", "/camera/camera/depth/image_rect_raw").value
        self.scale = P("depth_scale", 1e-3).value
        self.dmin = P("dmin", 0.07).value
        self.dmax = P("dmax", 1.0).value
        self.every = int(P("every", 3).value)      # 30Hz -> 10Hz. 시각화에 30Hz는 낭비다
        self.n = 0
        self.create_subscription(Image, self.src, self.on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, "~/image", 1)
        self.get_logger().info(f"{self.src} -> ~/image  범위 {self.dmin}~{self.dmax} m")

    def on_depth(self, m):
        self.n += 1
        if self.n % self.every:
            return
        if m.encoding not in ("16UC1", "mono16"):
            self.get_logger().error(f"지원하지 않는 encoding: {m.encoding}", once=True)
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        z = d.astype(np.float32) * self.scale
        img, valid = range_to_rgb(z, self.dmin, self.dmax)
        img = img.copy()
        img[~valid] = 0                               # 범위 밖 / 무효는 검정
        out = Image(header=m.header, height=m.height, width=m.width,
                    encoding="rgb8", is_bigendian=0, step=m.width * 3)
        out.data = img.astype(np.uint8).tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = DepthColorize()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
