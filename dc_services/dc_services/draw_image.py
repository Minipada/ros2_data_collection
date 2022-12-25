"""Collects cpu data."""

from typing import List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from dc_interfaces.srv import DrawImage
from pydantic import BaseModel, Field
from rclpy.node import Node


class ColorConfig(BaseModel):
    # Set default values in case they are not passed
    color: Optional[Tuple[int, int, int]] = Field(
        default=(255, 0, 0),
        description="Color of bounding box, polygon and text in RBG",
    )
    font_txt: Optional[int] = Field(
        default=cv2.FONT_HERSHEY_SIMPLEX, description="Font to use"
    )
    font_thickness: Optional[int] = Field(
        default=2, description="Thickness of lines used to render the text."
    )
    font_scale: Optional[float] = Field(
        default=0.8,
        description="Font scale factor that is multiplied by the font-specific base size.",
    )


class DrawImageNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.srv = self.create_service(
            DrawImage, "draw_image", self.draw_image_callback
        )

    def draw_image_callback(self, request, response):
        cv_br = CvBridge()
        cv_frame = cv_br.imgmsg_to_cv2(request.frame)
        color_config = ColorConfig(
            color=tuple(request.color),
            font_txt=request.font_txt,
            font_thickness=request.font_thickness,
            font_scale=request.font_scale,
        )
        try:
            if request.shape == "rectangle":
                cv2.rectangle(
                    img=cv_frame,
                    pt1=(request.box_left, request.box_top),
                    pt2=(
                        request.box_left + request.box_width,
                        request.box_top + request.box_height,
                    ),
                    color=color_config.color,
                    thickness=request.box_thickness,
                )
                cv2.putText(
                    img=cv_frame,
                    text=request.text,
                    org=(request.box_left, request.box_top - 9),
                    fontFace=color_config.font_txt,
                    fontScale=color_config.font_scale,
                    color=color_config.color,
                    thickness=color_config.font_thickness,
                )
                response.frame = cv_br.cv2_to_imgmsg(cv_frame)
                self.get_logger().debug(f"Added rectangle to image")
                response.success = True
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Could not add rectangle to image: {e}")
        return response


def main(args: Optional[List[str]] = None) -> None:

    rclpy.init(args=args)

    try:
        draw_image = DrawImageNode(node_name="draw_image")
        rclpy.spin(draw_image)
    except KeyboardInterrupt:
        print("draw_image stopped")  # noqa: T201
    finally:
        draw_image.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
