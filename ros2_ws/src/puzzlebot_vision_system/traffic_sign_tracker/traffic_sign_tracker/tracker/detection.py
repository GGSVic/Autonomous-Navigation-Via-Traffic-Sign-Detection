"""
Traffic Sign Detection Container.

Defines the core data structure for raw visual detections. Handles coordinate
extraction and intrinsic classification mapping into functional categories.
"""

from typing import Any, Tuple, Dict, List
from puzzlebot_interfaces.msg import TrafficSignDetection


class Detection:
    """
    Data container representing a single object detection from a vision (YOLOv8) model.

    This class encapsulates bounding box geometry, confidence metrics, and 
    functional categorization logic. 

    NOTE ON TRACKING STRATEGY:
    The 'category' and 'family' attributes act as a semantic filter for the 
    CentroidTracker. By segregating detections into functional groups, the 
    tracker restricts its search space, ensuring that a 'STOP' sign (REGULATORY) 
    is never accidentally associated with a 'Traffic Light' (TRAFFIC_LIGHT), 
    even if their spatial coordinates overlap or are adjacent.
    """

    # Static mapping to categorize signs into functional groups for FSM logic
    # Direction -> 1, TrafficLight -> 2, Regulatory -> 3, Caution -> 4
    CATEGORY_MAP: Dict[str, int] = {
        "go_ahead": TrafficSignDetection.DIRECTION,
        "turn_left": TrafficSignDetection.DIRECTION,
        "turn_right": TrafficSignDetection.DIRECTION,
        "roundabout": TrafficSignDetection.DIRECTION,
        "tf_red": TrafficSignDetection.TRAFFIC_LIGHT,
        "tf_green": TrafficSignDetection.TRAFFIC_LIGHT,
        "tf_yellow": TrafficSignDetection.TRAFFIC_LIGHT,
        "stop": TrafficSignDetection.REGULATORY,
        "give_way": TrafficSignDetection.REGULATORY,
        "roadwork": TrafficSignDetection.CAUTION,
        "max_speed": TrafficSignDetection.CAUTION,
    }

    def __init__(self, box: Any, name: str, conf: float):
        """
        Initializes a Detection instance by parsing raw data.
        """
        self.name: str = name
        self.confidence: float = conf

        # Categorization for logic-based tracking filters
        self.category: int = self.CATEGORY_MAP.get(self.name, 0)

        # Family grouping: Ensures that a traffic light maintains the same identity 
        # for the tracker even if its state (color) changes during execution.
        self.family: str = "traffic_light" if self.name.startswith("tf") else self.name

        # Bounding Box Geometry
        coords = box.xyxy[0].cpu().numpy()
        self.x1, self.y1, self.x2, self.y2 = map(int, coords)

        # Spatial Descriptors
        self.width: int = self.x2 - self.x1
        self.height: int = self.y2 - self.y1
        self.centroid: Tuple[int, int] = (
            int(self.x1 + (self.width / 2)),
            int(self.y1 + (self.height / 2)),
        )

    @property
    def area(self) -> int:
        """Computes detection pixel area for distance heuristics."""
        return self.width * self.height

    @staticmethod
    def from_yolo_results(results: Any, model_names: dict) -> List["Detection"]:
        """
        Factory method to parse and filter raw YOLO output.

        Applies differential thresholding logic to ensure high-quality 
        inputs for the tracking and control layers.
        """
        detections: List[Detection] = []

        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = model_names[cls]

            # Stricter thresholds for regulatory signs to avoid false positives 
            # in critical control decisions (e.g., stopping at a phantom sign).

            # Traffic lights use a lower confidence threshold (0.60) because the model was 
            # trained on multi-directional perspectives. This increased variance introduces 
            # more noise, necessitating a higher sensitivity to avoid false negatives.
            threshold = 0.60 if name.startswith("tf") else 0.80


            if conf >= threshold:
                detections.append(Detection(box, name, conf))

        return detections