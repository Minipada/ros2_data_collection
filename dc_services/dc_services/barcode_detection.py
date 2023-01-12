"""Collects cpu data."""


import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import rclpy
from cv_bridge import CvBridge
from pydantic import BaseModel, Field, validator
from rclpy.node import Node

from dc_interfaces.msg import Barcode
from dc_interfaces.srv import DetectBarcode


class DetectedObject(BaseModel):
    """Model of a detected object, does some validation and automatic fix."""

    top: int = Field(description="Top pixel")
    left: int = Field(description="Left pixel")
    width: int = Field(description="Box width")
    height: int = Field(description="Box height")

    @validator("left")
    def left_cant_be_negative(cls, value: int):
        """Sometimes, left index is equal to -1, we put it back to 0 to be able to draw it.

        Args:
            value (int): Left border in pixel in the image

        Returns:
            int: Value of the border left pixel
        """
        if value < 0:
            return 0
        return value


class DetectedBarcode(DetectedObject):
    """Barcode model containing data and type"""

    data: str = Field(description="Decoded data")
    type: str = Field(description="Barcode type")  # noqa: A003, VNE003


class BarcodeDetection(Node):
    """Receive barcode detection request and respond with metadata."""

    def __init__(self, node_name: str) -> None:
        """Initialize detect_barcodes service."""
        super().__init__(node_name=node_name)
        self.srv = self.create_service(DetectBarcode, "detect_barcodes", self.detect_callback)

    def detect_callback(self, request, response):
        """Do the detection, data validation and respond with metadata and data read.

        Args:
            request (DetectBarcode_Request): Service request
            response (DetectBarcode_Response): Service response

        Returns:
            DetectBarcode_Response: Service response
        """
        cv_br = CvBridge()
        cv_frame = cv_br.imgmsg_to_cv2(request.frame)

        # preprocessing with opencv https://stackoverflow.com/a/54465855/1717026
        img_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        kernel = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])
        image_sharp = cv2.filter2D(src=img_gray, ddepth=-1, kernel=kernel)
        decoded_objects = pyzbar.decode(image_sharp)
        self.detected: list[DetectedBarcode] | None = []

        for barcode in decoded_objects:
            self.detected.append(
                DetectedBarcode(
                    data=barcode.data.decode(),
                    type=barcode.type,
                    width=barcode.rect.width,
                    height=barcode.rect.height,
                    top=barcode.rect.top,
                    left=barcode.rect.left,
                )
            )

        response.barcodes = [
            Barcode(
                data=x.data,
                type=x.type,
                top=x.top,
                left=x.left,
                width=x.width,
                height=x.height,
            )
            for x in self.detected
        ]
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    try:
        barcode_detection = BarcodeDetection(node_name="barcode_detection")
        rclpy.spin(barcode_detection)
    except KeyboardInterrupt:
        print("barcode_detection stopped")  # noqa: T201
    finally:
        barcode_detection.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
