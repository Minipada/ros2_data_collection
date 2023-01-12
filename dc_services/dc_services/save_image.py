"""Collects cpu data."""

import pathlib

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from dc_interfaces.srv import SaveImage


class SaveImageNode(Node):
    """Node that save an image from a SaveImage request."""

    def __init__(self, node_name: str) -> None:
        """Initialize the node and create the save_image service."""
        super().__init__(node_name=node_name)
        self.srv = self.create_service(SaveImage, "save_image", self.save_callback)

    def save_callback(self, request, response):
        try:
            cv_br = CvBridge()
            # Transform image in cv frame
            cv_frame = cv_br.imgmsg_to_cv2(request.frame)
            img_path = pathlib.Path(request.path)
            img_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(img_path.as_posix(), cv_frame)
            response.success = True
            self.get_logger().debug(f"Image saved to {img_path.as_posix()}")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Image could not be saved to {img_path.as_posix()}: {e}")

        return response


def main(args: list[str] | None = None) -> None:
    """Start ROS node.

    Args:
        args (list[str] | None, optional): List of arguments to pass to the node. Defaults to None.
    """
    rclpy.init(args=args)
    try:
        save_image_node = SaveImageNode(node_name="save_image")
        rclpy.spin(save_image_node)
    except KeyboardInterrupt:
        print("save_image stopped")  # noqa: T201
    finally:
        save_image_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
