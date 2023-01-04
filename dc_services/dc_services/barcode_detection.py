"""Collects cpu data."""

from typing import List, Optional

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import rclpy
from cv_bridge import CvBridge
from dc_interfaces.msg import Barcode
from dc_interfaces.srv import DetectBarcode
from pydantic import BaseModel, Field, validator
from rclpy.node import Node


class DetectedObject(BaseModel):
    top: int = Field(description="Top pixel")
    left: int = Field(description="Left pixel")
    width: int = Field(description="Box width")
    height: int = Field(description="Box height")

    @validator("left")
    def left_cant_be_negative(cls, v):
        if v < 0:
            return 0
        return v


class DetectedBarcode(DetectedObject):
    data: str = Field(description="Decoded data")
    type: str = Field(description="Barcode type")


class BarcodeDetection(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.srv = self.create_service(
            DetectBarcode, "detect_barcodes", self.detect_callback
        )

    def detect_callback(self, request, response):
        cv_br = CvBridge()
        cv_frame = cv_br.imgmsg_to_cv2(request.frame)

        # preprocessing with opencv https://stackoverflow.com/a/54465855/1717026
        img_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        kernel = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])
        image_sharp = cv2.filter2D(src=img_gray, ddepth=-1, kernel=kernel)
        decoded_objects = pyzbar.decode(image_sharp)
        self.detected: Optional[List[DetectedBarcode]] = []

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


def main(args: Optional[List[str]] = None) -> None:

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
